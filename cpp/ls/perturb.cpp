#include <algorithm>
#include <utility>

#include "core/queries.h"
#include "ls/insertion.h"
#include "ls/perturb.h"
#include "ls/rng.h"

namespace kayros {

namespace {

// One ruin+recreate attempt on `states` (no SearchState bookkeeping — the
// caller stamps epochs on success). Returns false when some removed client
// could not be feasibly replaced (caller restores the snapshot).
bool attempt(const Instance& inst, const NeighbourLists& nb,
             std::vector<RouteState>& states, std::mt19937_64& rng,
             std::int32_t target_removals, std::vector<std::int32_t>& removed,
             std::int32_t* new_routes, bool dissolve) {
    const std::int32_t n = inst.num_customers;

    // Ruin: seeds in random order, each dragging its granular neighbours.
    std::vector<char> is_removed(static_cast<std::size_t>(n) + 1, 0);
    std::vector<std::int32_t> order(static_cast<std::size_t>(n));
    for (std::int32_t c = 1; c <= n; ++c) order[static_cast<std::size_t>(c - 1)] = c;
    fisher_yates(order, rng);
    removed.clear();
    const auto remove_one = [&](std::int32_t c) {
        if (is_removed[static_cast<std::size_t>(c)]) return;
        is_removed[static_cast<std::size_t>(c)] = 1;
        removed.push_back(c);
    };
    // Route-dissolve seed (M7 FleetCostDuration): every client of one
    // smallest route (random among ties) goes out first, whole; the normal
    // seed walk then tops the kick up to target_removals if there is room.
    if (dissolve) {
        std::size_t min_size = states[0].vertices.size();
        for (const RouteState& s : states) {
            min_size = std::min(min_size, s.vertices.size());
        }
        std::vector<std::size_t> ties;
        for (std::size_t k = 0; k < states.size(); ++k) {
            if (states[k].vertices.size() == min_size) ties.push_back(k);
        }
        const std::size_t pick = ties[static_cast<std::size_t>(
            draw(rng, static_cast<std::uint64_t>(ties.size())))];
        for (const std::int32_t v : states[pick].vertices) remove_one(v);
    }
    for (const std::int32_t u : order) {
        if (static_cast<std::int32_t>(removed.size()) >= target_removals) break;
        remove_one(u);
        if (nb.restricted()) {
            for (const std::int32_t* v = nb.neighbours_begin(u);
                 v != nb.neighbours_end(u); ++v) {
                if (static_cast<std::int32_t>(removed.size()) >= target_removals)
                    break;
                remove_one(*v);
            }
        }
    }

    // Rebuild the ruined routes (a subsequence of a feasible route stays
    // feasible), dropping emptied ones.
    for (std::size_t k = states.size(); k-- > 0;) {
        std::vector<std::int32_t> kept;
        kept.reserve(states[k].vertices.size());
        bool changed = false;
        for (const std::int32_t v : states[k].vertices) {
            if (is_removed[static_cast<std::size_t>(v)]) {
                changed = true;
            } else {
                kept.push_back(v);
            }
        }
        if (!changed) continue;
        if (kept.empty()) {
            states.erase(states.begin() + static_cast<std::ptrdiff_t>(k));
        } else if (!build_route_state(inst, std::move(kept), states[k])) {
            return false;  // never expected; the caller restores
        }
    }

    // Recreate: random order, best tree-ranked feasible position, committed
    // through the checker-fold rebuild (next-best on a fold disagreement).
    std::vector<std::int32_t> insert_order = removed;
    fisher_yates(insert_order, rng);
    const Pwlf dep = departure_identity(inst);
    for (const std::int32_t c : insert_order) {
        bool placed = false;
        for (const InsertionCandidate& cand :
             insertion_candidates(inst, states, c, dep)) {
            std::vector<std::int32_t> next = states[cand.route].vertices;
            next.insert(next.begin() + static_cast<std::ptrdiff_t>(cand.position),
                        c);
            RouteState rebuilt;
            if (build_route_state(inst, std::move(next), rebuilt)) {
                states[cand.route] = std::move(rebuilt);
                placed = true;
                break;
            }
        }
        if (!placed) {
            // Fallback (design decision, Onyr 2026-07-07): open a singleton
            // route when the fleet bound allows.
            const bool fleet_ok =
                inst.num_vehicles < 0 ||
                states.size() < static_cast<std::size_t>(inst.num_vehicles);
            RouteState singleton;
            if (fleet_ok && build_route_state(inst, {c}, singleton)) {
                states.push_back(std::move(singleton));
                if (new_routes) ++(*new_routes);
                placed = true;
            }
        }
        if (!placed) return false;  // undo + redraw at the caller
    }
    return true;
}

}  // namespace

PerturbOutcome perturb(const Instance& inst, const NeighbourLists& nb,
                       SearchState& ss, std::mt19937_64& rng,
                       const PerturbParams& params) {
    PerturbOutcome outcome;
    const std::int32_t n = inst.num_customers;
    if (n == 0 || ss.states.empty()) return outcome;

    // Snapshot for the undo path (vertex vectors only; states are rebuilt).
    std::vector<std::vector<std::int32_t>> snapshot;
    snapshot.reserve(ss.states.size());
    for (const RouteState& s : ss.states) snapshot.push_back(s.vertices);

    std::vector<std::int32_t> removed;
    for (std::int32_t attempt_idx = 0; attempt_idx <= params.max_redraws;
         ++attempt_idx) {
        const std::int32_t span =
            params.max_removals - params.min_removals + 1;
        const std::int32_t target = std::min(
            n, params.min_removals +
                   static_cast<std::int32_t>(draw(
                       rng, static_cast<std::uint64_t>(span > 0 ? span : 1))));
        // Dissolve arming is gated on fixed_route_cost > 0 BEFORE any draw:
        // under Duration the rng stream is bitwise the pre-M7 one.
        const bool dissolve =
            inst.fixed_route_cost > 0.0 && params.dissolve_pct > 0 &&
            ss.states.size() >= 2 &&
            draw(rng, 100) <
                static_cast<std::uint64_t>(params.dissolve_pct);
        std::int32_t new_routes = 0;
        if (attempt(inst, nb, ss.states, rng, target, removed, &new_routes,
                    dissolve)) {
            outcome.applied = true;
            outcome.removed = static_cast<std::int32_t>(removed.size());
            outcome.new_routes = new_routes;
            outcome.dissolved = dissolve;
            break;
        }
        // Undo: restore every route from the snapshot (deterministic rebuild).
        ++outcome.redraws;
        ss.states.assign(snapshot.size(), RouteState{});
        for (std::size_t k = 0; k < snapshot.size(); ++k) {
            const bool ok = build_route_state(inst, snapshot[k], ss.states[k]);
            (void)ok;  // snapshot routes were feasible by invariant
        }
    }
    if (!outcome.applied) return outcome;

    // Stamp the kick: one epoch for the whole batch; touched = every removed
    // client + every client sharing a pre-kick or post-kick route with one.
    const std::int64_t epoch = ++ss.epoch;
    std::vector<char> hit(static_cast<std::size_t>(n) + 1, 0);
    for (const std::int32_t c : removed) hit[static_cast<std::size_t>(c)] = 1;
    const auto stamp_routes =
        [&](const auto& route_vertices_of) {
            for (std::size_t k = 0; k < route_vertices_of.size(); ++k) {
                const auto& verts = route_vertices_of[k];
                bool any = false;
                for (const std::int32_t v : verts) {
                    if (hit[static_cast<std::size_t>(v)]) {
                        any = true;
                        break;
                    }
                }
                if (!any) continue;
                for (const std::int32_t v : verts) {
                    ss.touched[v] = epoch;
                }
            }
        };
    stamp_routes(snapshot);
    std::vector<std::vector<std::int32_t>> current;
    current.reserve(ss.states.size());
    for (const RouteState& s : ss.states) current.push_back(s.vertices);
    stamp_routes(current);
    for (const std::int32_t c : removed) ss.touched[c] = epoch;
    return outcome;
}

}  // namespace kayros
