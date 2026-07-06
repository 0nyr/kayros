#include <algorithm>
#include <utility>

#include "core/queries.h"
#include "heuristics/heuristics.h"
#include "ls/ls.h"

namespace kayros {

namespace {

// Screening threshold on tree-ranked deltas: ulp-class association noise
// (P4.1) must not trigger commits; genuine improvements on MAMUT instances
// are orders of magnitude above this.
constexpr double kScreenEps = 1e-9;

Pwlf departure_identity(const Instance& inst) {
    double dep_lo = inst.horizon_start;
    double dep_hi = inst.horizon_end;
    if (inst.has_time_windows) {
        dep_lo = std::max(dep_lo, inst.tw_earliest[0]);
        dep_hi = std::min(dep_hi, inst.tw_latest[0]);
    }
    if (dep_lo > dep_hi) return {};
    return identity(dep_lo, dep_hi);
}

// Commit path (memo Decision 3): rebuild the changed routes from scratch —
// leaves, tree and the sequential checker-fold repricing — and accept only if
// the repriced sum of the touched routes strictly improves on their old sum.
// An empty vertex vector drops the route. Returns true when committed.
bool commit_two(const Instance& inst, std::vector<RouteState>& states,
                std::size_t a, std::vector<std::int32_t> new_a, std::size_t b,
                std::vector<std::int32_t> new_b, LsStats* stats) {
    const double old_sum =
        states[a].duration + (b == a ? 0.0 : states[b].duration);
    RouteState cand_a, cand_b;
    double new_sum = 0.0;
    if (!new_a.empty()) {
        if (!build_route_state(inst, std::move(new_a), cand_a)) {
            if (stats) ++stats->reverted;
            return false;
        }
        new_sum += cand_a.duration;
    }
    if (b != a) {
        if (!new_b.empty()) {
            if (!build_route_state(inst, std::move(new_b), cand_b)) {
                if (stats) ++stats->reverted;
                return false;
            }
            new_sum += cand_b.duration;
        }
    }
    if (!(new_sum < old_sum)) {  // the fold is the accountant, strictly
        if (stats) ++stats->reverted;
        return false;
    }
    // Commit: replace in place, drop emptied routes (higher index first).
    std::size_t drop_first = states.size(), drop_second = states.size();
    if (cand_a.vertices.empty()) drop_first = a; else states[a] = std::move(cand_a);
    if (b != a) {
        if (cand_b.vertices.empty()) drop_second = b; else states[b] = std::move(cand_b);
    }
    if (drop_first > drop_second) std::swap(drop_first, drop_second);
    if (drop_second < states.size())
        states.erase(states.begin() + static_cast<std::ptrdiff_t>(drop_second));
    if (drop_first < states.size() && drop_first != drop_second)
        states.erase(states.begin() + static_cast<std::ptrdiff_t>(drop_first));
    if (stats) ++stats->applied;
    return true;
}

// Inter-route relocate, receiver-major with the shared-prefix insertion scan
// (memo Decision 4): the left-fold prefix advances once per position and is
// shared across every donor candidate at that position.
bool relocate_pass(const Instance& inst, std::vector<RouteState>& states,
                   LsStats* stats) {
    const std::size_t num_routes = states.size();
    if (num_routes < 2) return false;

    // Deletion rankings, one per donor position (0.0 when the route empties).
    std::vector<std::vector<double>> del_dur(num_routes);
    for (std::size_t a = 0; a < num_routes; ++a) {
        const std::int64_t m = static_cast<std::int64_t>(states[a].vertices.size());
        del_dur[a].assign(static_cast<std::size_t>(m), 0.0);
        if (m == 1) continue;
        for (std::int64_t i = 0; i < m; ++i) {
            const RouteEval eval =
                evaluate_splice(inst, states[a], i, i, states[a], 1, 0);
            // Removing a stop never tightens: treat a (never observed)
            // infeasible removal as unusable.
            del_dur[a][static_cast<std::size_t>(i)] =
                eval.feasible ? eval.duration : kInfeasible;
        }
    }

    const Pwlf dep = departure_identity(inst);
    for (std::size_t b = 0; b < num_routes; ++b) {
        RouteState& recv = states[b];
        const std::int64_t mb = static_cast<std::int64_t>(recv.vertices.size());
        Pwlf prefix = dep;
        for (std::int64_t p = 0; p <= mb; ++p) {
            if (p > 0) {
                prefix = compose(view(recv.tree.leaf(p - 1)), view(prefix));
                if (prefix.xs.empty()) break;
            }
            const std::int32_t before =
                p > 0 ? recv.vertices[static_cast<std::size_t>(p - 1)] : 0;
            for (std::size_t a = 0; a < num_routes; ++a) {
                if (a == b) continue;
                const std::int64_t ma =
                    static_cast<std::int64_t>(states[a].vertices.size());
                for (std::int64_t i = 0; i < ma; ++i) {
                    const std::int32_t c =
                        states[a].vertices[static_cast<std::size_t>(i)];
                    if (recv.load + inst.demands[c] > inst.vehicle_capacity)
                        continue;
                    const double gain =
                        states[a].duration - del_dur[a][static_cast<std::size_t>(i)];
                    if (!(gain > kScreenEps)) continue;  // also skips kInfeasible
                    // Insertion of c before position p, on the shared prefix.
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
                    const double delta = (s.value - recv.duration) - gain;
                    if (delta < -kScreenEps) {
                        std::vector<std::int32_t> new_a = states[a].vertices;
                        new_a.erase(new_a.begin() + static_cast<std::ptrdiff_t>(i));
                        std::vector<std::int32_t> new_b = recv.vertices;
                        new_b.insert(new_b.begin() + static_cast<std::ptrdiff_t>(p), c);
                        if (commit_two(inst, states, a, std::move(new_a), b,
                                       std::move(new_b), stats)) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}

bool intra_pass(const Instance& inst, std::vector<RouteState>& states,
                LsStats* stats) {
    for (std::size_t a = 0; a < states.size(); ++a) {
        const std::int64_t m = static_cast<std::int64_t>(states[a].vertices.size());
        if (m < 2) continue;
        for (std::int64_t i = 0; i < m; ++i) {
            for (std::int64_t p = 0; p <= m; ++p) {
                if (p == i || p == i + 1) continue;
                const RouteEval eval =
                    evaluate_intra_relocate(inst, states[a], i, p);
                if (!eval.feasible) continue;
                if (eval.duration - states[a].duration < -kScreenEps) {
                    std::vector<std::int32_t> next = states[a].vertices;
                    const std::int32_t c = next[static_cast<std::size_t>(i)];
                    next.erase(next.begin() + static_cast<std::ptrdiff_t>(i));
                    const std::int64_t q = p < i ? p : p - 1;
                    next.insert(next.begin() + static_cast<std::ptrdiff_t>(q), c);
                    if (commit_two(inst, states, a, std::move(next), a, {},
                                   stats)) {
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

bool swap_pass(const Instance& inst, std::vector<RouteState>& states,
               LsStats* stats) {
    for (std::size_t a = 0; a + 1 < states.size(); ++a) {
        for (std::size_t b = a + 1; b < states.size(); ++b) {
            const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
            const std::int64_t mb = static_cast<std::int64_t>(states[b].vertices.size());
            for (std::int64_t i = 0; i < ma; ++i) {
                const std::int32_t c1 = states[a].vertices[static_cast<std::size_t>(i)];
                for (std::int64_t j = 0; j < mb; ++j) {
                    const std::int32_t c2 =
                        states[b].vertices[static_cast<std::size_t>(j)];
                    if (states[a].load - inst.demands[c1] + inst.demands[c2] >
                            inst.vehicle_capacity ||
                        states[b].load - inst.demands[c2] + inst.demands[c1] >
                            inst.vehicle_capacity) {
                        continue;
                    }
                    const RouteEval ea =
                        evaluate_splice(inst, states[a], i, i, states[b], j, j);
                    if (!ea.feasible) continue;
                    const RouteEval eb =
                        evaluate_splice(inst, states[b], j, j, states[a], i, i);
                    if (!eb.feasible) continue;
                    const double delta = (ea.duration - states[a].duration) +
                                         (eb.duration - states[b].duration);
                    if (delta < -kScreenEps) {
                        std::vector<std::int32_t> new_a = states[a].vertices;
                        std::vector<std::int32_t> new_b = states[b].vertices;
                        new_a[static_cast<std::size_t>(i)] = c2;
                        new_b[static_cast<std::size_t>(j)] = c1;
                        if (commit_two(inst, states, a, std::move(new_a), b,
                                       std::move(new_b), stats)) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}

bool two_opt_star_pass(const Instance& inst, std::vector<RouteState>& states,
                       LsStats* stats) {
    for (std::size_t a = 0; a + 1 < states.size(); ++a) {
        for (std::size_t b = a + 1; b < states.size(); ++b) {
            const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
            const std::int64_t mb = static_cast<std::int64_t>(states[b].vertices.size());
            // Prefix demand loads: pa[i] = load of A[0..i].
            std::vector<std::int64_t> pa(static_cast<std::size_t>(ma));
            std::vector<std::int64_t> pb(static_cast<std::size_t>(mb));
            std::int64_t acc_load = 0;
            for (std::int64_t i = 0; i < ma; ++i) {
                acc_load += inst.demands[states[a].vertices[static_cast<std::size_t>(i)]];
                pa[static_cast<std::size_t>(i)] = acc_load;
            }
            acc_load = 0;
            for (std::int64_t j = 0; j < mb; ++j) {
                acc_load += inst.demands[states[b].vertices[static_cast<std::size_t>(j)]];
                pb[static_cast<std::size_t>(j)] = acc_load;
            }
            for (std::int64_t i = 0; i < ma; ++i) {
                for (std::int64_t j = 0; j < mb; ++j) {
                    if (i == ma - 1 && j == mb - 1) continue;  // no-op
                    const std::int64_t head_a = pa[static_cast<std::size_t>(i)];
                    const std::int64_t head_b = pb[static_cast<std::size_t>(j)];
                    if (head_a + (states[b].load - head_b) > inst.vehicle_capacity ||
                        head_b + (states[a].load - head_a) > inst.vehicle_capacity) {
                        continue;
                    }
                    const RouteEval ea = evaluate_splice(
                        inst, states[a], i + 1, ma - 1, states[b], j + 1, mb - 1);
                    if (!ea.feasible) continue;
                    const RouteEval eb = evaluate_splice(
                        inst, states[b], j + 1, mb - 1, states[a], i + 1, ma - 1);
                    if (!eb.feasible) continue;
                    const double delta = (ea.duration - states[a].duration) +
                                         (eb.duration - states[b].duration);
                    if (delta < -kScreenEps) {
                        std::vector<std::int32_t> new_a(
                            states[a].vertices.begin(),
                            states[a].vertices.begin() + static_cast<std::ptrdiff_t>(i + 1));
                        new_a.insert(new_a.end(),
                                     states[b].vertices.begin() + static_cast<std::ptrdiff_t>(j + 1),
                                     states[b].vertices.end());
                        std::vector<std::int32_t> new_b(
                            states[b].vertices.begin(),
                            states[b].vertices.begin() + static_cast<std::ptrdiff_t>(j + 1));
                        new_b.insert(new_b.end(),
                                     states[a].vertices.begin() + static_cast<std::ptrdiff_t>(i + 1),
                                     states[a].vertices.end());
                        if (commit_two(inst, states, a, std::move(new_a), b,
                                       std::move(new_b), stats)) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    return false;
}

}  // namespace

double local_search(const Instance& inst,
                    std::vector<std::vector<std::int32_t>>& routes,
                    LsStats* stats) {
    if (routes.empty()) return kInfeasible;
    std::vector<RouteState> states(routes.size());
    for (std::size_t k = 0; k < routes.size(); ++k) {
        if (!build_route_state(inst, routes[k], states[k])) return kInfeasible;
    }

    // First-improvement VND: any committed move restarts the operator cycle.
    // Terminates: every commit strictly decreases the (checker-fold) total.
    bool improved = true;
    while (improved) {
        improved = relocate_pass(inst, states, stats) ||
                   intra_pass(inst, states, stats) ||
                   swap_pass(inst, states, stats) ||
                   two_opt_star_pass(inst, states, stats);
    }

    routes.clear();
    routes.reserve(states.size());
    for (RouteState& s : states) routes.push_back(std::move(s.vertices));
    return solution_duration(inst, routes);
}

}  // namespace kayros
