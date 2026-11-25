#include <algorithm>
#include <bit>
#include <cstdint>
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
// the repriced sum of the touched routes strictly improves on their old sum,
// FleetCostDuration-priced: each side carries fixed_route_cost per non-empty
// route, so a move that empties a route is credited F (exact no-op at F = 0).
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
    const int old_routes = b == a ? 1 : 2;  // stored routes are never empty
    int new_routes = 0;
    RouteState cand_a, cand_b;
    double new_sum = 0.0;
    if (!new_a.empty()) {
        if (!build_route_state(inst, std::move(new_a), cand_a)) {
            if (stats) ++stats->reverted;
            return false;
        }
        new_sum += cand_a.duration;
        ++new_routes;
    }
    if (b != a) {
        if (!new_b.empty()) {
            if (!build_route_state(inst, std::move(new_b), cand_b)) {
                if (stats) ++stats->reverted;
                return false;
            }
            new_sum += cand_b.duration;
            ++new_routes;
        }
    }
    const double f = inst.fixed_route_cost;
    // the fold is the accountant, strictly
    if (!(new_sum + f * new_routes < old_sum + f * old_routes)) {
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
                if (stats) ++stats->evaluated;
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

// Route index and position of every routed client (-1 when unrouted).
struct Positions {
    std::vector<std::int32_t> route_of;
    std::vector<std::int32_t> pos_of;
};

void fill_positions(const SearchState& ss, Positions& at) {
    const std::size_t n1 = ss.touched.size();
    at.route_of.assign(n1, -1);
    at.pos_of.assign(n1, -1);
    for (std::size_t a = 0; a < ss.states.size(); ++a) {
        const std::vector<std::int32_t>& v = ss.states[a].vertices;
        for (std::size_t i = 0; i < v.size(); ++i) {
            at.route_of[static_cast<std::size_t>(v[i])] = static_cast<std::int32_t>(a);
            at.pos_of[static_cast<std::size_t>(v[i])] = static_cast<std::int32_t>(i);
        }
    }
}

// A key orders candidates by receiver route, donor position and receiver position.
constexpr int kKeyBits = 21;
constexpr std::uint64_t kKeyMask = (std::uint64_t{1} << kKeyBits) - 1;

// Larger inputs use the ordered scan.
bool keys_fit(const SearchState& ss) {
    return ss.touched.size() <= kKeyMask + 1 && ss.states.size() <= kKeyMask + 1;
}

inline std::uint64_t pack_key(std::uint64_t r, std::uint64_t i, std::uint64_t j) {
    return (r << (2 * kKeyBits)) | (i << kKeyBits) | j;
}

inline std::int64_t list_degree(const NeighbourLists& nb, std::int32_t c) {
    return nb.offsets[c + 1] - nb.offsets[c];
}

// Flagged donors receive slots in route and position order. Scanning the union of the two seam-neighbour bitmaps in slot order preserves first improvement.
bool relocate_pass(const Instance& inst, const NeighbourLists& nb, SearchState& ss,
                   LsStats* stats) {
    std::vector<RouteState>& states = ss.states;
    const std::size_t num_routes = states.size();
    if (num_routes < 2) return false;

    const std::int64_t pass_epoch = ss.epoch;
    const std::vector<char> retest = compute_retest(nb, ss, kRelocate);
    if (std::none_of(retest.begin(), retest.end(), [](char r) { return r != 0; })) {
        return false;
    }

    for (std::size_t a = 0; a < num_routes; ++a) deletion_rankings(inst, states[a]);

    // Reuse the slot buffers across passes on each search thread.
    thread_local std::vector<std::int32_t> slot_client_tl, slot_route_tl, slot_pos_tl, slot_of_tl,
        slot_begin_tl;
    thread_local std::vector<std::uint64_t> mark_tl;
    std::vector<std::int32_t>& slot_client = slot_client_tl;
    std::vector<std::int32_t>& slot_route = slot_route_tl;
    std::vector<std::int32_t>& slot_pos = slot_pos_tl;
    std::vector<std::int32_t>& slot_of = slot_of_tl;
    std::vector<std::int32_t>& slot_begin = slot_begin_tl;
    std::vector<std::uint64_t>& mark = mark_tl;
    slot_client.clear();
    slot_route.clear();
    slot_pos.clear();
    slot_of.assign(retest.size(), -1);
    slot_begin.assign(num_routes + 1, 0);
    for (std::size_t a = 0; a < num_routes; ++a) {
        slot_begin[a] = static_cast<std::int32_t>(slot_client.size());
        const std::vector<std::int32_t>& v = states[a].vertices;
        for (std::size_t i = 0; i < v.size(); ++i) {
            if (!retest[static_cast<std::size_t>(v[i])]) continue;
            slot_of[static_cast<std::size_t>(v[i])] = static_cast<std::int32_t>(slot_client.size());
            slot_client.push_back(v[i]);
            slot_route.push_back(static_cast<std::int32_t>(a));
            slot_pos.push_back(static_cast<std::int32_t>(i));
        }
    }
    slot_begin[num_routes] = static_cast<std::int32_t>(slot_client.size());
    const std::int32_t num_slots = slot_begin[num_routes];
    mark.assign((static_cast<std::size_t>(num_slots) + 63) / 64, 0);

    const Pwlf dep = departure_identity(inst);
    Pwlf prefix_next;
    for (std::size_t b = 0; b < num_routes; ++b) {
        RouteState& recv = states[b];
        const std::int64_t mb = static_cast<std::int64_t>(recv.vertices.size());
        const std::int32_t b_lo = slot_begin[b];
        const std::int32_t b_hi = slot_begin[b + 1];
        Pwlf prefix = dep;
        for (std::int64_t p = 0; p <= mb; ++p) {
            if (p > 0) {
                compose_into(view(recv.tree.leaf(p - 1)), view(prefix), prefix_next);
                std::swap(prefix, prefix_next);
                if (prefix.xs.empty()) break;
            }
            const std::int32_t before =
                p > 0 ? recv.vertices[static_cast<std::size_t>(p - 1)] : 0;
            const std::int32_t succ =
                p < mb ? recv.vertices[static_cast<std::size_t>(p)] : 0;
            // Both enumerations inline the same hot candidate body.
            const auto try_relocate = [&](std::size_t a, std::int64_t i,
                                          std::int32_t c) __attribute__((always_inline)) -> bool {
                const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
                // Moving a singleton also removes its fixed route cost.
                const double empty_credit = ma == 1 ? inst.fixed_route_cost : 0.0;
                if (recv.load + inst.demands[c] > inst.vehicle_capacity) return false;
                const double gain =
                    states[a].duration - states[a].del_dur[static_cast<std::size_t>(i)];
                if (!(gain + empty_credit > kScreenEps)) return false;
                if (stats) ++stats->evaluated;
                MinShift s;
                if (!price_insertion(inst, recv, view(prefix), before, c, p, mb, s)) {
                    return false;
                }
                const double delta = (s.value - recv.duration) - gain - empty_credit;
                if (delta < -kScreenEps) {
                    std::vector<std::int32_t> new_a = states[a].vertices;
                    new_a.erase(new_a.begin() + static_cast<std::ptrdiff_t>(i));
                    std::vector<std::int32_t> new_b = recv.vertices;
                    new_b.insert(new_b.begin() + static_cast<std::ptrdiff_t>(p), c);
                    if (commit_two(inst, ss, a, std::move(new_a), b, std::move(new_b), stats)) {
                        return true;
                    }
                }
                return false;
            };
            // Scanning flagged donors is cheaper when the seam lists cover most slots.
            const bool scan =
                !nb.restricted() ||
                list_degree(nb, before) +
                        (p < mb ? list_degree(nb, succ) : 0) >=
                    num_slots;
            if (scan) {
                // The exhaustive sentinel also uses this scan.
                for (std::size_t a = 0; a < num_routes; ++a) {
                    if (a == b) continue;
                    const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
                    for (std::int64_t i = 0; i < ma; ++i) {
                        const std::int32_t c = states[a].vertices[static_cast<std::size_t>(i)];
                        if (!retest[static_cast<std::size_t>(c)]) continue;
                        if (!nb.is_neighbour(c, before) && !(p < mb && nb.is_neighbour(c, succ))) {
                            continue;
                        }
                        if (try_relocate(a, i, c)) return true;
                    }
                }
                continue;
            }
            const auto gather = [&](std::int32_t v) {
                for (const std::int32_t* it = nb.neighbours_begin(v); it != nb.neighbours_end(v);
                     ++it) {
                    const std::int32_t k = slot_of[static_cast<std::size_t>(*it)];
                    if (k < 0 || (k >= b_lo && k < b_hi)) continue;
                    mark[static_cast<std::size_t>(k) >> 6] |= std::uint64_t{1} << (k & 63);
                }
            };
            gather(before);
            if (p < mb) gather(succ);
            for (std::size_t w = 0; w < mark.size(); ++w) {
                while (mark[w] != 0) {
                    const int bit = std::countr_zero(mark[w]);
                    mark[w] &= mark[w] - 1;
                    const std::size_t k = w * 64 + static_cast<std::size_t>(bit);
                    if (try_relocate(static_cast<std::size_t>(slot_route[k]), slot_pos[k], slot_client[k])) {
                        return true;
                    }
                }
            }
        }
    }
    stamp_clean_pass(ss, kRelocate, retest, pass_epoch);
    return false;
}

// A stable counting sort groups by route. Within each group the donor positions already increase, so insertion sort finishes the receiver-position ordering.
void order_keys(std::vector<std::uint64_t>& keys, std::vector<std::uint64_t>& tmp,
                std::vector<std::size_t>& cnt, std::size_t num_routes) {
    cnt.assign(num_routes + 1, 0);
    for (const std::uint64_t k : keys) ++cnt[static_cast<std::size_t>(k >> (2 * kKeyBits)) + 1];
    for (std::size_t r = 1; r <= num_routes; ++r) cnt[r] += cnt[r - 1];
    tmp.resize(keys.size());
    for (const std::uint64_t k : keys) {
        tmp[static_cast<std::size_t>(cnt[static_cast<std::size_t>(k >> (2 * kKeyBits))]++)] = k;
    }
    for (std::size_t k = 1; k < tmp.size(); ++k) {
        const std::uint64_t x = tmp[k];
        std::size_t t = k;
        while (t > 0 && tmp[t - 1] > x) {
            tmp[t] = tmp[t - 1];
            --t;
        }
        tmp[t] = x;
    }
    keys.swap(tmp);
}

// The list path of the swap and 2-opt* passes is taken when the neighbour lists it reads are this many times shorter than the scan it replaces. Both paths visit the same candidates in the same order.
constexpr std::int64_t kListFactor = 4;

// The list path gathers each flagged donor's neighbours, then visits swaps in route and position order.
bool swap_pass(const Instance& inst, const NeighbourLists& nb, SearchState& ss,
               LsStats* stats) {
    std::vector<RouteState>& states = ss.states;
    const std::size_t num_routes = states.size();
    const std::int64_t pass_epoch = ss.epoch;
    const std::vector<char> retest = compute_retest(nb, ss, kSwap);

    const auto try_swap = [&](std::size_t a, std::size_t b, std::int64_t i,
                              std::int64_t j) __attribute__((always_inline)) -> bool {
        const std::int32_t c1 = states[a].vertices[static_cast<std::size_t>(i)];
        const std::int32_t c2 = states[b].vertices[static_cast<std::size_t>(j)];
        if (states[a].load - inst.demands[c1] + inst.demands[c2] > inst.vehicle_capacity ||
            states[b].load - inst.demands[c2] + inst.demands[c1] > inst.vehicle_capacity) {
            return false;
        }
        if (stats) ++stats->evaluated;
        const RouteEval ea = evaluate_splice(inst, states[a], i, i, states[b], j, j);
        if (!ea.feasible) return false;
        const RouteEval eb = evaluate_splice(inst, states[b], j, j, states[a], i, i);
        if (!eb.feasible) return false;
        const double delta =
            (ea.duration - states[a].duration) + (eb.duration - states[b].duration);
        if (delta < -kScreenEps) {
            std::vector<std::int32_t> new_a = states[a].vertices;
            std::vector<std::int32_t> new_b = states[b].vertices;
            new_a[static_cast<std::size_t>(i)] = c2;
            new_b[static_cast<std::size_t>(j)] = c1;
            if (commit_two(inst, ss, a, std::move(new_a), b, std::move(new_b), stats)) {
                return true;
            }
        }
        return false;
    };

    const bool use_lists = [&] {
        if (!nb.restricted() || !keys_fit(ss)) return false;
        std::int64_t list_cost = 0, scan_cost = 0, suffix = 0;
        for (const RouteState& s : states) suffix += static_cast<std::int64_t>(s.vertices.size());
        for (std::size_t a = 0; a < num_routes; ++a) {
            suffix -= static_cast<std::int64_t>(states[a].vertices.size());
            for (const std::int32_t c1 : states[a].vertices) {
                if (!retest[static_cast<std::size_t>(c1)]) continue;
                list_cost += list_degree(nb, c1);
                scan_cost += suffix;
            }
        }
        return list_cost * kListFactor < scan_cost;
    }();

    if (!use_lists) {
        for (std::size_t a = 0; a + 1 < num_routes; ++a) {
            for (std::size_t b = a + 1; b < num_routes; ++b) {
                const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
                const std::int64_t mb = static_cast<std::int64_t>(states[b].vertices.size());
                for (std::int64_t i = 0; i < ma; ++i) {
                    const std::int32_t c1 = states[a].vertices[static_cast<std::size_t>(i)];
                    if (!retest[static_cast<std::size_t>(c1)]) continue;
                    for (std::int64_t j = 0; j < mb; ++j) {
                        const std::int32_t c2 = states[b].vertices[static_cast<std::size_t>(j)];
                        if (!nb.is_neighbour(c1, c2)) continue;
                        if (try_swap(a, b, i, j)) return true;
                    }
                }
            }
        }
        stamp_clean_pass(ss, kSwap, retest, pass_epoch);
        return false;
    }

    thread_local Positions at_tl;
    thread_local std::vector<std::uint64_t> cand_tl, tmp_tl;
    thread_local std::vector<std::size_t> cnt_tl;
    Positions& at = at_tl;
    std::vector<std::uint64_t>& cand = cand_tl;
    fill_positions(ss, at);
    for (std::size_t a = 0; a + 1 < num_routes; ++a) {
        const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
        cand.clear();
        for (std::int64_t i = 0; i < ma; ++i) {
            const std::int32_t c1 = states[a].vertices[static_cast<std::size_t>(i)];
            if (!retest[static_cast<std::size_t>(c1)]) continue;
            for (const std::int32_t* it = nb.neighbours_begin(c1); it != nb.neighbours_end(c1);
                 ++it) {
                const std::int32_t rb = at.route_of[static_cast<std::size_t>(*it)];
                if (rb <= static_cast<std::int32_t>(a)) continue;
                cand.push_back(pack_key(static_cast<std::uint64_t>(rb), static_cast<std::uint64_t>(i),
                                        static_cast<std::uint64_t>(at.pos_of[static_cast<std::size_t>(*it)])));
            }
        }
        order_keys(cand, tmp_tl, cnt_tl, num_routes);
        for (const std::uint64_t key : cand) {
            if (try_swap(a, static_cast<std::size_t>(key >> (2 * kKeyBits)),
                         static_cast<std::int64_t>((key >> kKeyBits) & kKeyMask),
                         static_cast<std::int64_t>(key & kKeyMask))) {
                return true;
            }
        }
    }
    stamp_clean_pass(ss, kSwap, retest, pass_epoch);
    return false;
}

// A pair of cuts is eligible when either new client-client arc joins neighbours. The two seam lists can nominate the same cut, so deduplicate before pricing.
bool two_opt_star_pass(const Instance& inst, const NeighbourLists& nb, SearchState& ss,
                       LsStats* stats) {
    std::vector<RouteState>& states = ss.states;
    const std::size_t num_routes = states.size();
    const std::int64_t pass_epoch = ss.epoch;
    const std::vector<char> retest = compute_retest(nb, ss, kTwoOptStar);

    // head_load[head_begin[r] + i] = load of route r through position i.
    thread_local std::vector<std::int64_t> head_load_tl, head_begin_tl;
    std::vector<std::int64_t>& head_load = head_load_tl;
    std::vector<std::int64_t>& head_begin = head_begin_tl;
    head_begin.assign(num_routes + 1, 0);
    for (std::size_t r = 0; r < num_routes; ++r) {
        head_begin[r + 1] = head_begin[r] + static_cast<std::int64_t>(states[r].vertices.size());
    }
    head_load.resize(static_cast<std::size_t>(head_begin.back()));
    for (std::size_t r = 0; r < num_routes; ++r) {
        std::int64_t acc_load = 0;
        for (std::size_t i = 0; i < states[r].vertices.size(); ++i) {
            acc_load += inst.demands[states[r].vertices[i]];
            head_load[static_cast<std::size_t>(head_begin[r]) + i] = acc_load;
        }
    }

    const auto cut_flagged = [&](std::size_t a, std::int64_t i, std::int32_t& ai,
                                 std::int32_t& ai_next) __attribute__((always_inline)) {
        const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
        ai = states[a].vertices[static_cast<std::size_t>(i)];
        ai_next = i + 1 < ma ? states[a].vertices[static_cast<std::size_t>(i + 1)] : 0;
        return retest[static_cast<std::size_t>(ai)] ||
               (ai_next != 0 && retest[static_cast<std::size_t>(ai_next)]);
    };

    const auto try_cut = [&](std::size_t a, std::size_t b, std::int64_t i,
                             std::int64_t j) __attribute__((always_inline)) -> bool {
        const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
        const std::int64_t mb = static_cast<std::int64_t>(states[b].vertices.size());
        if (i == ma - 1 && j == mb - 1) return false;  // no-op
        const std::int64_t head_a = head_load[static_cast<std::size_t>(head_begin[a] + i)];
        const std::int64_t head_b = head_load[static_cast<std::size_t>(head_begin[b] + j)];
        if (head_a + (states[b].load - head_b) > inst.vehicle_capacity ||
            head_b + (states[a].load - head_a) > inst.vehicle_capacity) {
            return false;
        }
        if (stats) ++stats->evaluated;
        const RouteEval ea =
            evaluate_splice(inst, states[a], i + 1, ma - 1, states[b], j + 1, mb - 1);
        if (!ea.feasible) return false;
        const RouteEval eb =
            evaluate_splice(inst, states[b], j + 1, mb - 1, states[a], i + 1, ma - 1);
        if (!eb.feasible) return false;
        const double delta =
            (ea.duration - states[a].duration) + (eb.duration - states[b].duration);
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
            if (commit_two(inst, ss, a, std::move(new_a), b, std::move(new_b), stats)) {
                return true;
            }
        }
        return false;
    };

    const bool use_lists = [&] {
        if (!nb.restricted() || !keys_fit(ss)) return false;
        std::int64_t list_cost = 0, scan_cost = 0, suffix = head_begin.back();
        for (std::size_t a = 0; a < num_routes; ++a) {
            const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
            suffix -= ma;
            for (std::int64_t i = 0; i < ma; ++i) {
                std::int32_t ai, ai_next;
                if (!cut_flagged(a, i, ai, ai_next)) continue;
                list_cost += list_degree(nb, ai) + (ai_next != 0 ? list_degree(nb, ai_next) : 0);
                scan_cost += suffix;
            }
        }
        return list_cost * kListFactor < scan_cost;
    }();

    if (!use_lists) {
        for (std::size_t a = 0; a + 1 < num_routes; ++a) {
            for (std::size_t b = a + 1; b < num_routes; ++b) {
                const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
                const std::int64_t mb = static_cast<std::int64_t>(states[b].vertices.size());
                for (std::int64_t i = 0; i < ma; ++i) {
                    std::int32_t ai, ai_next;
                    if (!cut_flagged(a, i, ai, ai_next)) continue;
                    for (std::int64_t j = 0; j < mb; ++j) {
                        const std::int32_t bj = states[b].vertices[static_cast<std::size_t>(j)];
                        const std::int32_t bj_next =
                            j + 1 < mb ? states[b].vertices[static_cast<std::size_t>(j + 1)] : 0;
                        if (!(bj_next != 0 && nb.is_neighbour(ai, bj_next)) &&
                            !(ai_next != 0 && nb.is_neighbour(bj, ai_next))) {
                            continue;
                        }
                        if (try_cut(a, b, i, j)) return true;
                    }
                }
            }
        }
        stamp_clean_pass(ss, kTwoOptStar, retest, pass_epoch);
        return false;
    }

    thread_local Positions at_tl;
    thread_local std::vector<std::uint64_t> cand_tl, tmp_tl;
    thread_local std::vector<std::size_t> cnt_tl;
    Positions& at = at_tl;
    std::vector<std::uint64_t>& cand = cand_tl;
    fill_positions(ss, at);
    for (std::size_t a = 0; a + 1 < num_routes; ++a) {
        const std::int64_t ma = static_cast<std::int64_t>(states[a].vertices.size());
        cand.clear();
        for (std::int64_t i = 0; i < ma; ++i) {
            std::int32_t ai, ai_next;
            if (!cut_flagged(a, i, ai, ai_next)) continue;
            for (const std::int32_t* it = nb.neighbours_begin(ai); it != nb.neighbours_end(ai);
                 ++it) {
                const std::int32_t rb = at.route_of[static_cast<std::size_t>(*it)];
                const std::int32_t pos = at.pos_of[static_cast<std::size_t>(*it)];
                if (rb <= static_cast<std::int32_t>(a) || pos < 1) continue;
                cand.push_back(pack_key(static_cast<std::uint64_t>(rb), static_cast<std::uint64_t>(i),
                                        static_cast<std::uint64_t>(pos - 1)));
            }
            if (ai_next != 0) {
                for (const std::int32_t* it = nb.neighbours_begin(ai_next);
                     it != nb.neighbours_end(ai_next); ++it) {
                    const std::int32_t rb = at.route_of[static_cast<std::size_t>(*it)];
                    if (rb <= static_cast<std::int32_t>(a)) continue;
                    cand.push_back(pack_key(static_cast<std::uint64_t>(rb), static_cast<std::uint64_t>(i),
                                            static_cast<std::uint64_t>(at.pos_of[static_cast<std::size_t>(*it)])));
                }
            }
        }
        order_keys(cand, tmp_tl, cnt_tl, num_routes);
        cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
        for (const std::uint64_t key : cand) {
            if (try_cut(a, static_cast<std::size_t>(key >> (2 * kKeyBits)),
                        static_cast<std::int64_t>((key >> kKeyBits) & kKeyMask),
                        static_cast<std::int64_t>(key & kKeyMask))) {
                return true;
            }
        }
    }
    stamp_clean_pass(ss, kTwoOptStar, retest, pass_epoch);
    return false;
}

}  // namespace

void mark_all_touched(SearchState& ss) {
    const std::int64_t epoch = ++ss.epoch;
    for (std::size_t c = 1; c < ss.touched.size(); ++c) ss.touched[c] = epoch;
}

Pwlf departure_identity(const Instance& inst) {
    double points[2];
    return copy_pwlf(inst.departure_window(points));
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
                  SearchState& ss, LsStats* stats,
                  const std::chrono::steady_clock::time_point* deadline) {
    const auto past = [deadline]() {
        return deadline != nullptr &&
               std::chrono::steady_clock::now() >= *deadline;
    };
    // First-improvement VND: any committed move restarts the operator cycle.
    // Terminates: every commit strictly decreases the (checker-fold) total.
    bool improved = true;
    while (improved && !past()) {
        improved = relocate_pass(inst, nb, ss, stats) ||
                   (!past() && intra_pass(inst, nb, ss, stats)) ||
                   (!past() && swap_pass(inst, nb, ss, stats)) ||
                   (!past() && two_opt_star_pass(inst, nb, ss, stats));
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
