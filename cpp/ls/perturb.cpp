#include <algorithm>
#include <utility>

#include "core/queries.h"
#include "ls/perturb.h"

namespace kayros {

namespace {

// Platform-stable integer draw in [0, n): modulo over the full 64-bit stream
// (std::uniform_int_distribution is implementation-defined; bias at these
// ranges is negligible and determinism wins).
std::uint64_t draw(std::mt19937_64& rng, std::uint64_t n) { return rng() % n; }

void fisher_yates(std::vector<std::int32_t>& v, std::mt19937_64& rng) {
    for (std::size_t i = v.size(); i > 1; --i) {
        std::swap(v[i - 1], v[static_cast<std::size_t>(draw(rng, i))]);
    }
}

struct InsertionCandidate {
    double delta;
    std::size_t route;
    std::int64_t position;
};

// Tree-ranked feasible insertion candidates of client c, best-first
// (shared-prefix scan per route, capacity-screened). Ranking only.
std::vector<InsertionCandidate> insertion_candidates(
    const Instance& inst, const std::vector<RouteState>& states,
    std::int32_t c, const Pwlf& dep) {
    std::vector<InsertionCandidate> out;
    for (std::size_t b = 0; b < states.size(); ++b) {
        const RouteState& recv = states[b];
        if (recv.load + inst.demands[c] > inst.vehicle_capacity) continue;
        const std::int64_t mb = static_cast<std::int64_t>(recv.vertices.size());
        Pwlf prefix = dep;
        for (std::int64_t p = 0; p <= mb; ++p) {
            if (p > 0) {
                prefix = compose(view(recv.tree.leaf(p - 1)), view(prefix));
                if (prefix.xs.empty()) break;
            }
            const std::int32_t before =
                p > 0 ? recv.vertices[static_cast<std::size_t>(p - 1)] : 0;
            const Pwlf in_bridge = bridge_leaf(inst, before, c);
            if (in_bridge.xs.empty()) continue;
            Pwlf acc = compose(view(in_bridge), view(prefix));
            if (acc.xs.empty()) continue;
            if (p < mb) {
                const Pwlf out_bridge = bridge_leaf(
                    inst, c, recv.vertices[static_cast<std::size_t>(p)]);
                if (out_bridge.xs.empty()) continue;
                acc = compose(view(out_bridge), view(acc));
                if (acc.xs.empty()) continue;
                const Pwlf rest = recv.tree.query(p + 1, mb);
                if (rest.xs.empty()) continue;
                acc = compose(view(rest), view(acc));
            } else {
                const Pwlf ret = return_leaf(inst, c);
                if (ret.xs.empty()) continue;
                acc = compose(view(ret), view(acc));
            }
            if (acc.xs.empty()) continue;
            const MinShift s = min_shifted_image(view(acc));
            out.push_back({s.value - recv.duration, b, p});
        }
    }
    std::sort(out.begin(), out.end(),
              [](const InsertionCandidate& x, const InsertionCandidate& y) {
                  if (x.delta != y.delta) return x.delta < y.delta;
                  if (x.route != y.route) return x.route < y.route;
                  return x.position < y.position;
              });
    return out;
}

// One ruin+recreate attempt on `states` (no SearchState bookkeeping — the
// caller stamps epochs on success). Returns false when some removed client
// could not be feasibly replaced (caller restores the snapshot).
bool attempt(const Instance& inst, const NeighbourLists& nb,
             std::vector<RouteState>& states, std::mt19937_64& rng,
             std::int32_t target_removals, std::vector<std::int32_t>& removed,
             std::int32_t* new_routes) {
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
        std::int32_t new_routes = 0;
        if (attempt(inst, nb, ss.states, rng, target, removed, &new_routes)) {
            outcome.applied = true;
            outcome.removed = static_cast<std::int32_t>(removed.size());
            outcome.new_routes = new_routes;
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
