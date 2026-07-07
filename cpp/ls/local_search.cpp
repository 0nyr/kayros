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

// Operator indices into SearchState::last_tested.
enum Op : std::size_t { kRelocate = 0, kIntra = 1, kSwap = 2, kTwoOptStar = 3 };

// Commit path (memo Decision 3): rebuild the changed routes from scratch —
// leaves, tree and the sequential checker-fold repricing — and accept only if
// the repriced sum of the touched routes strictly improves on their old sum.
// An empty vertex vector drops the route. Returns true when committed.
// M7.0: a commit bumps the epoch, stamps both routes' last_modified and marks
// every client of the post-move routes touched (moves preserve the client
// partition, so the union of the new routes covers the union of the old).
bool commit_two(const Instance& inst, SearchState& ss, std::size_t a,
                std::vector<std::int32_t> new_a, std::size_t b,
                std::vector<std::int32_t> new_b, LsStats* stats) {
    std::vector<RouteState>& states = ss.states;
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
    const std::int64_t epoch = ++ss.epoch;
    for (const std::int32_t v : cand_a.vertices) ss.touched[v] = epoch;
    for (const std::int32_t v : cand_b.vertices) ss.touched[v] = epoch;
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

// Per-pass retest mask (M7.0 staleness). Client c is (re)enumerated by
// operator op iff its own context changed since its stamp, or — for the
// inter-route operators — some granular neighbour's context did. With
// exhaustive lists the neighbour term is "anything changed anywhere"
// (max over touched), which only ever skips work at a proven fixed point.
std::vector<char> compute_retest(const NeighbourLists& nb,
                                 const SearchState& ss, Op op) {
    const std::size_t n1 = ss.touched.size();
    const std::vector<std::int64_t>& lt = ss.last_tested[op];
    std::vector<char> retest(n1, 0);
    std::int64_t max_touched = 0;
    if (op != kIntra && !nb.restricted()) {
        for (std::size_t c = 1; c < n1; ++c) {
            max_touched = std::max(max_touched, ss.touched[c]);
        }
    }
    for (std::size_t c = 1; c < n1; ++c) {
        if (ss.touched[c] > lt[c]) {
            retest[c] = 1;
            continue;
        }
        if (op == kIntra) continue;  // own-route context only
        if (!nb.restricted()) {
            retest[c] = max_touched > lt[c] ? 1 : 0;
            continue;
        }
        const std::int32_t ci = static_cast<std::int32_t>(c);
        for (const std::int32_t* v = nb.neighbours_begin(ci);
             v != nb.neighbours_end(ci); ++v) {
            if (ss.touched[*v] > lt[c]) {
                retest[c] = 1;
                break;
            }
        }
    }
    return retest;
}

// A pass that completed without committing stamps the clients it enumerated.
void stamp_clean_pass(SearchState& ss, Op op, const std::vector<char>& retest,
                      std::int64_t pass_epoch) {
    std::vector<std::int64_t>& lt = ss.last_tested[op];
    for (std::size_t c = 1; c < retest.size(); ++c) {
        if (retest[c]) lt[c] = pass_epoch;
    }
}

// Deletion rankings for every donor position of a route (0.0 when the route
// empties), cached on the RouteState until the next rebuild.
const std::vector<double>& deletion_rankings(const Instance& inst,
                                             RouteState& state) {
    if (state.del_valid) return state.del_dur;
    const std::int64_t m = static_cast<std::int64_t>(state.vertices.size());
    state.del_dur.assign(static_cast<std::size_t>(m), 0.0);
    if (m > 1) {
        for (std::int64_t i = 0; i < m; ++i) {
            const RouteEval eval = evaluate_splice(inst, state, i, i, state, 1, 0);
            // Removing a stop never tightens: treat a (never observed)
            // infeasible removal as unusable.
            state.del_dur[static_cast<std::size_t>(i)] =
                eval.feasible ? eval.duration : kInfeasible;
        }
    }
    state.del_valid = true;
    return state.del_dur;
}

// Inter-route relocate, receiver-major with the shared-prefix insertion scan
// (memo Decision 4): the left-fold prefix advances once per position and is
// shared across every donor candidate at that position. Granular restriction:
// the donor must be a granular neighbour of an insertion-seam client.
bool relocate_pass(const Instance& inst, const NeighbourLists& nb,
                   SearchState& ss, LsStats* stats) {
    std::vector<RouteState>& states = ss.states;
    const std::size_t num_routes = states.size();
    if (num_routes < 2) return false;

    const std::int64_t pass_epoch = ss.epoch;
    const std::vector<char> retest = compute_retest(nb, ss, kRelocate);
    if (std::none_of(retest.begin(), retest.end(), [](char r) { return r != 0; })) {
        return false;
    }

    for (std::size_t a = 0; a < num_routes; ++a) deletion_rankings(inst, states[a]);

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
            const std::int32_t succ =
                p < mb ? recv.vertices[static_cast<std::size_t>(p)] : 0;
            for (std::size_t a = 0; a < num_routes; ++a) {
                if (a == b) continue;
                const std::int64_t ma =
                    static_cast<std::int64_t>(states[a].vertices.size());
                for (std::int64_t i = 0; i < ma; ++i) {
                    const std::int32_t c =
                        states[a].vertices[static_cast<std::size_t>(i)];
                    if (!retest[static_cast<std::size_t>(c)]) continue;
                    // Granular seam justification (depot bits are never set
                    // in restricted mode; both checks pass when exhaustive).
                    if (!nb.is_neighbour(c, before) &&
                        !(p < mb && nb.is_neighbour(c, succ))) {
                        continue;
                    }
                    if (recv.load + inst.demands[c] > inst.vehicle_capacity)
                        continue;
                    const double gain =
                        states[a].duration -
                        states[a].del_dur[static_cast<std::size_t>(i)];
                    if (!(gain > kScreenEps)) continue;  // also skips kInfeasible
                    // Insertion of c before position p, on the shared prefix.
                    const Pwlf in_bridge = bridge_leaf(inst, before, c);
                    if (in_bridge.xs.empty()) continue;
                    Pwlf acc = compose(view(in_bridge), view(prefix));
                    if (acc.xs.empty()) continue;
                    if (p < mb) {
                        const Pwlf out_bridge = bridge_leaf(inst, c, succ);
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
                        if (commit_two(inst, ss, a, std::move(new_a), b,
                                       std::move(new_b), stats)) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    stamp_clean_pass(ss, kRelocate, retest, pass_epoch);
    return false;
}

bool intra_pass(const Instance& inst, const NeighbourLists& nb, SearchState& ss,
                LsStats* stats) {
    std::vector<RouteState>& states = ss.states;
    const std::int64_t pass_epoch = ss.epoch;
    const std::vector<char> retest = compute_retest(nb, ss, kIntra);
    for (std::size_t a = 0; a < states.size(); ++a) {
        const std::int64_t m = static_cast<std::int64_t>(states[a].vertices.size());
        if (m < 2) continue;
        for (std::int64_t i = 0; i < m; ++i) {
            const std::int32_t c = states[a].vertices[static_cast<std::size_t>(i)];
            if (!retest[static_cast<std::size_t>(c)]) continue;
            for (std::int64_t p = 0; p <= m; ++p) {
                if (p == i || p == i + 1) continue;
                const RouteEval eval =
                    evaluate_intra_relocate(inst, states[a], i, p);
                if (!eval.feasible) continue;
                if (eval.duration - states[a].duration < -kScreenEps) {
                    std::vector<std::int32_t> next = states[a].vertices;
                    next.erase(next.begin() + static_cast<std::ptrdiff_t>(i));
                    const std::int64_t q = p < i ? p : p - 1;
                    next.insert(next.begin() + static_cast<std::ptrdiff_t>(q), c);
                    if (commit_two(inst, ss, a, std::move(next), a, {}, stats)) {
                        return true;
                    }
                }
            }
        }
    }
    stamp_clean_pass(ss, kIntra, retest, pass_epoch);
    return false;
}

bool swap_pass(const Instance& inst, const NeighbourLists& nb, SearchState& ss,
               LsStats* stats) {
    std::vector<RouteState>& states = ss.states;
    const std::int64_t pass_epoch = ss.epoch;
    const std::vector<char> retest = compute_retest(nb, ss, kSwap);
    for (std::size_t a = 0; a + 1 < states.size(); ++a) {
        for (std::size_t b = a + 1; b < states.size(); ++b) {
            const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
            const std::int64_t mb = static_cast<std::int64_t>(states[b].vertices.size());
            for (std::int64_t i = 0; i < ma; ++i) {
                const std::int32_t c1 = states[a].vertices[static_cast<std::size_t>(i)];
                if (!retest[static_cast<std::size_t>(c1)]) continue;
                for (std::int64_t j = 0; j < mb; ++j) {
                    const std::int32_t c2 =
                        states[b].vertices[static_cast<std::size_t>(j)];
                    if (!nb.is_neighbour(c1, c2)) continue;
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
                        if (commit_two(inst, ss, a, std::move(new_a), b,
                                       std::move(new_b), stats)) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    stamp_clean_pass(ss, kSwap, retest, pass_epoch);
    return false;
}

bool two_opt_star_pass(const Instance& inst, const NeighbourLists& nb,
                       SearchState& ss, LsStats* stats) {
    std::vector<RouteState>& states = ss.states;
    const std::int64_t pass_epoch = ss.epoch;
    const std::vector<char> retest = compute_retest(nb, ss, kTwoOptStar);
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
                const std::int32_t ai = states[a].vertices[static_cast<std::size_t>(i)];
                const std::int32_t ai_next =
                    i + 1 < ma ? states[a].vertices[static_cast<std::size_t>(i + 1)] : 0;
                // The cut is keyed on both tail-boundary clients of route a:
                // reconnections can be justified through either created arc.
                if (!retest[static_cast<std::size_t>(ai)] &&
                    !(ai_next != 0 && retest[static_cast<std::size_t>(ai_next)])) {
                    continue;
                }
                for (std::int64_t j = 0; j < mb; ++j) {
                    if (i == ma - 1 && j == mb - 1) continue;  // no-op
                    const std::int32_t bj =
                        states[b].vertices[static_cast<std::size_t>(j)];
                    const std::int32_t bj_next =
                        j + 1 < mb ? states[b].vertices[static_cast<std::size_t>(j + 1)]
                                   : 0;
                    // Granular justification: one created client-client arc
                    // joins granular neighbours (depot reconnections do not
                    // justify on their own; both checks pass when exhaustive).
                    if (!(bj_next != 0 && nb.is_neighbour(ai, bj_next)) &&
                        !(ai_next != 0 && nb.is_neighbour(bj, ai_next))) {
                        continue;
                    }
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
                        if (commit_two(inst, ss, a, std::move(new_a), b,
                                       std::move(new_b), stats)) {
                            return true;
                        }
                    }
                }
            }
        }
    }
    stamp_clean_pass(ss, kTwoOptStar, retest, pass_epoch);
    return false;
}

}  // namespace

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

bool init_search_state(const Instance& inst,
                       const std::vector<std::vector<std::int32_t>>& routes,
                       SearchState& ss) {
    ss.states.assign(routes.size(), RouteState{});
    for (std::size_t k = 0; k < routes.size(); ++k) {
        if (!build_route_state(inst, routes[k], ss.states[k])) return false;
    }
    const std::size_t n1 = static_cast<std::size_t>(inst.num_vertices());
    ss.touched.assign(n1, 1);
    ss.last_tested.assign(4, std::vector<std::int64_t>(n1, 0));
    ss.epoch = 1;
    return true;
}

double ls_descend(const Instance& inst, const NeighbourLists& nb,
                  SearchState& ss, LsStats* stats) {
    // First-improvement VND: any committed move restarts the operator cycle.
    // Terminates: every commit strictly decreases the (checker-fold) total.
    bool improved = true;
    while (improved) {
        improved = relocate_pass(inst, nb, ss, stats) ||
                   intra_pass(inst, nb, ss, stats) ||
                   swap_pass(inst, nb, ss, stats) ||
                   two_opt_star_pass(inst, nb, ss, stats);
    }
    std::vector<std::vector<std::int32_t>> routes;
    routes.reserve(ss.states.size());
    for (const RouteState& s : ss.states) routes.push_back(s.vertices);
    return solution_duration(inst, routes);
}

double local_search(const Instance& inst, const NeighbourLists& nb,
                    std::vector<std::vector<std::int32_t>>& routes,
                    LsStats* stats) {
    if (routes.empty()) return kInfeasible;
    SearchState ss;
    if (!init_search_state(inst, routes, ss)) return kInfeasible;
    const double value = ls_descend(inst, nb, ss, stats);
    routes.clear();
    routes.reserve(ss.states.size());
    for (RouteState& s : ss.states) routes.push_back(std::move(s.vertices));
    return value;
}

double local_search(const Instance& inst,
                    std::vector<std::vector<std::int32_t>>& routes,
                    LsStats* stats) {
    const NeighbourLists exhaustive;  // k == 0 sentinel
    return local_search(inst, exhaustive, routes, stats);
}

}  // namespace kayros
