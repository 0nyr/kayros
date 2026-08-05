#include "ls/fleet_descent.h"

#include <algorithm>
#include <limits>

#include "ls/insertion.h"
#include "ls/rng.h"
#include "ls/squeeze.h"

namespace kayros {

namespace {

// All-or-nothing undo: unconditional deterministic rebuild from the vertices
// snapshot (route indices shift after a dissolve, so the ILS snapshot's
// identity-check fast path does not apply here; this mirrors the perturb
// redraw undo).
void restore_states(const Instance& inst, std::vector<RouteState>& states,
                    const std::vector<std::vector<std::int32_t>>& snapshot) {
    states.assign(snapshot.size(), RouteState{});
    for (std::size_t k = 0; k < snapshot.size(); ++k) {
        const bool ok = build_route_state(inst, snapshot[k], states[k]);
        (void)ok;  // snapshot routes were feasible by invariant
    }
}

}  // namespace

bool fleet_descent(const Instance& inst, const NeighbourLists& nb,
                   SearchState& ss, std::mt19937_64& rng,
                   const FleetDescentParams& params,
                   std::vector<std::int32_t>& pcount,
                   const std::chrono::steady_clock::time_point* deadline,
                   std::int64_t* work_budget, FdStats* stats) {
    using Clock = std::chrono::steady_clock;
    const std::size_t K = ss.states.size();
    if (K < 2) return false;
    if (stats) ++stats->attempts;
    // One route pricing = one unit of drain work, charged against the
    // per-trigger fd_work_cap budget and recorded in FdStats::evaluated.
    const auto charge = [&](std::int64_t units) {
        if (stats) stats->evaluated += units;
        if (work_budget != nullptr) *work_budget -= units;
    };

    std::vector<std::vector<std::int32_t>> snapshot;
    snapshot.reserve(K);
    for (const RouteState& s : ss.states) snapshot.push_back(s.vertices);

    // Victim route: uniform random (NBRMH's choice) or smallest (the dissolve
    // kick's choice), uniform among ties.
    std::size_t victim;
    if (params.route_choice == 1) {
        std::size_t min_size = ss.states[0].vertices.size();
        for (const RouteState& s : ss.states) {
            min_size = std::min(min_size, s.vertices.size());
        }
        std::vector<std::size_t> ties;
        for (std::size_t k = 0; k < K; ++k) {
            if (ss.states[k].vertices.size() == min_size) ties.push_back(k);
        }
        victim = ties[static_cast<std::size_t>(
            draw(rng, static_cast<std::uint64_t>(ties.size())))];
    } else {
        victim = static_cast<std::size_t>(
            draw(rng, static_cast<std::uint64_t>(K)));
    }

    // Ejection pool, LIFO (route order in, reverse order out).
    std::vector<std::int32_t> pool = ss.states[victim].vertices;
    ss.states.erase(ss.states.begin() + static_cast<std::ptrdiff_t>(victim));

    const Pwlf dep = departure_identity(inst);
    std::int64_t budget = params.ep_budget;

    while (!pool.empty()) {
        if (work_budget != nullptr && *work_budget <= 0) {
            if (stats) ++stats->rollbacks_work;
            restore_states(inst, ss.states, snapshot);
            return false;
        }
        if (deadline != nullptr && Clock::now() >= *deadline) {
            if (stats) ++stats->rollbacks_time;
            restore_states(inst, ss.states, snapshot);
            return false;
        }
        if (budget-- <= 0) {
            if (stats) ++stats->rollbacks_budget;
            restore_states(inst, ss.states, snapshot);
            return false;
        }

        std::size_t pick = pool.size() - 1;  // LIFO
        if (params.pop_order == 1) {
            for (std::size_t i = 0; i < pool.size(); ++i) {
                if (pcount[static_cast<std::size_t>(pool[i])] >
                    pcount[static_cast<std::size_t>(pool[pick])]) {
                    pick = i;
                }
            }
        }
        const std::int32_t c = pool[pick];
        pool.erase(pool.begin() + static_cast<std::ptrdiff_t>(pick));
        if (stats) ++stats->pops;

        // Ladder step 1: feasible best-insert (tree-ranked, fold-committed,
        // next-best on a fold disagreement). NO singleton fallback: reopening
        // a route is exactly the failure mode this phase exists to remove.
        bool placed = false;
        const std::vector<InsertionCandidate> cands =
            insertion_candidates(inst, ss.states, c, dep);
        charge(static_cast<std::int64_t>(cands.size()));
        for (const InsertionCandidate& cand : cands) {
            std::vector<std::int32_t> next = ss.states[cand.route].vertices;
            next.insert(next.begin() + static_cast<std::ptrdiff_t>(cand.position),
                        c);
            RouteState rebuilt;
            charge(1);
            if (build_route_state(inst, std::move(next), rebuilt)) {
                ss.states[cand.route] = std::move(rebuilt);
                placed = true;
                if (stats) ++stats->step1_inserts;
                break;
            }
        }
        if (placed) continue;

        // Plan-15 S1: squeeze strictly between feasible-insert failure and
        // ejection (the RMH order); the p-count increments only after BOTH
        // fail. Inert at sq_ladder=false: the increment below then sits
        // exactly where it always did.
        if (params.sq_ladder) {
            if (stats) ++stats->ladder_squeezes;
            std::vector<std::vector<std::int32_t>> cur;
            cur.reserve(ss.states.size());
            for (const RouteState& s : ss.states) cur.push_back(s.vertices);
            std::int64_t unlimited = std::numeric_limits<std::int64_t>::max() / 2;
            std::int64_t* wb = work_budget != nullptr ? work_budget : &unlimited;
            SqueezeStats sst;
            const bool rescued = squeeze_insert(inst, nb, cur, c,
                                                params.sq_penalty, wb, rng, &sst);
            if (stats) stats->squeeze_evaluated += sst.evaluated;
            if (rescued) {
                // Adopt: rebuild the RouteStates from the zero-warp result
                // (all-or-nothing: only swap in when every route rebuilds).
                std::vector<RouteState> next(cur.size());
                bool ok = true;
                for (std::size_t k = 0; k < cur.size(); ++k) {
                    if (!build_route_state(inst, cur[k], next[k])) {
                        ok = false;
                        break;
                    }
                }
                if (ok) {
                    ss.states = std::move(next);
                    if (stats) ++stats->ladder_rescues;
                    continue;
                }
            }
        }
        ++pcount[static_cast<std::size_t>(c)];

        // Ladder step 2 (NBRMH step 3): contiguous-window insertion-with-
        // ejection. One splice evaluation prices "replace recv[i..j] by [c]";
        // among feasible windows pick min sum of p-counts, tie-broken by the
        // splice-ranked delta, then (route, i). The commit reprices through
        // the checker fold; ejected clients return to the pool.
        RouteState c_single;
        charge(1);
        if (!build_route_state(inst, {c}, c_single)) {
            // Unreachable for clients of a previously feasible route (a
            // subsequence of a feasible route is feasible); guarded anyway.
            if (stats) ++stats->rollbacks_deadend;
            restore_states(inst, ss.states, snapshot);
            return false;
        }
        struct Best {
            bool found = false;
            std::size_t route = 0;
            std::int64_t i = 0;
            std::int64_t j = 0;
            std::int64_t psum = 0;
            double delta = 0.0;
        } best;
        for (std::size_t b = 0; b < ss.states.size(); ++b) {
            const RouteState& recv = ss.states[b];
            if (nb.restricted()) {
                bool near = false;
                for (const std::int32_t v : recv.vertices) {
                    if (nb.is_neighbour(c, v)) {
                        near = true;
                        break;
                    }
                }
                if (!near) continue;
            }
            const std::int64_t m =
                static_cast<std::int64_t>(recv.vertices.size());
            for (std::int64_t i = 0; i < m; ++i) {
                std::int64_t wload = 0;
                for (std::int64_t j = i;
                     j < m && j - i + 1 <= params.k_max; ++j) {
                    wload += inst.demands[static_cast<std::size_t>(
                        recv.vertices[static_cast<std::size_t>(j)])];
                    if (recv.load - wload + inst.demands[static_cast<std::size_t>(c)] >
                        inst.vehicle_capacity) {
                        continue;  // more ejection (larger j) can still fit
                    }
                    charge(1);
                    const RouteEval e =
                        evaluate_splice(inst, recv, i, j, c_single, 0, 0);
                    if (!e.feasible) continue;
                    std::int64_t psum = 0;
                    for (std::int64_t t = i; t <= j; ++t) {
                        psum += pcount[static_cast<std::size_t>(
                            recv.vertices[static_cast<std::size_t>(t)])];
                    }
                    const double delta = e.duration - recv.duration;
                    const bool better =
                        !best.found || psum < best.psum ||
                        (psum == best.psum &&
                         (delta < best.delta ||
                          (delta == best.delta &&
                           (b < best.route ||
                            (b == best.route && i < best.i)))));
                    if (better) {
                        best.found = true;
                        best.route = b;
                        best.i = i;
                        best.j = j;
                        best.psum = psum;
                        best.delta = delta;
                    }
                }
            }
        }
        if (!best.found) {
            if (stats) ++stats->rollbacks_deadend;
            restore_states(inst, ss.states, snapshot);
            return false;
        }
        const RouteState& recv = ss.states[best.route];
        std::vector<std::int32_t> ejected(
            recv.vertices.begin() + static_cast<std::ptrdiff_t>(best.i),
            recv.vertices.begin() + static_cast<std::ptrdiff_t>(best.j + 1));
        std::vector<std::int32_t> next;
        next.reserve(recv.vertices.size() - ejected.size() + 1);
        next.insert(next.end(), recv.vertices.begin(),
                    recv.vertices.begin() + static_cast<std::ptrdiff_t>(best.i));
        next.push_back(c);
        next.insert(next.end(),
                    recv.vertices.begin() + static_cast<std::ptrdiff_t>(best.j + 1),
                    recv.vertices.end());
        RouteState rebuilt;
        charge(1);
        if (!build_route_state(inst, std::move(next), rebuilt)) {
            // Tree-ranked feasible but fold-rejected: rare; V0 treats it as a
            // dead end rather than re-scanning for the next-best window.
            if (stats) ++stats->rollbacks_deadend;
            restore_states(inst, ss.states, snapshot);
            return false;
        }
        ss.states[best.route] = std::move(rebuilt);
        if (stats) stats->ejections += static_cast<std::int64_t>(ejected.size());
        for (const std::int32_t v : ejected) pool.push_back(v);
    }

    if (stats) ++stats->successes;
    return true;
}

}  // namespace kayros
