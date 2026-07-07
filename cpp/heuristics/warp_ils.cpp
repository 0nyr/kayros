#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "core/queries.h"
#include "heuristics/warp_ils.h"
#include "ls/perturb.h"

namespace kayros {

namespace {

constexpr double kScreenEps = 1e-9;  // local_search.cpp's screening epsilon

// Whole-search state: warp route states + client->route bookkeeping +
// route-level staleness (coarser than M7.0's per-operator epochs: a client is
// retested when its own route or a granular neighbour's route changed).
struct WarpSearch {
    std::vector<WarpRouteState> states;
    std::vector<std::int32_t> route_of;   // vertex id -> state index (-1 = none)
    std::vector<std::int64_t> pos_of;     // vertex id -> position in its route
    std::vector<std::int64_t> route_epoch;
    std::vector<std::int64_t> client_stamp;
    std::int64_t epoch = 1;
};

void index_route(WarpSearch& ws, std::int32_t r) {
    const std::vector<std::int32_t>& vs = ws.states[static_cast<std::size_t>(r)].vertices;
    for (std::int64_t p = 0; p < static_cast<std::int64_t>(vs.size()); ++p) {
        ws.route_of[static_cast<std::size_t>(vs[static_cast<std::size_t>(p)])] = r;
        ws.pos_of[static_cast<std::size_t>(vs[static_cast<std::size_t>(p)])] = p;
    }
}

bool init_warp_search(const Instance& inst,
                      const std::vector<std::vector<std::int32_t>>& routes,
                      double penalty, double t_end, WarpSearch& ws) {
    ws.states.clear();
    const std::size_t nv = static_cast<std::size_t>(inst.num_vertices());
    ws.route_of.assign(nv, -1);
    ws.pos_of.assign(nv, -1);
    ws.client_stamp.assign(nv, 0);
    ws.epoch = 1;
    for (const std::vector<std::int32_t>& r : routes) {
        if (r.empty()) continue;
        WarpRouteState st;
        if (!build_warp_route_state(inst, r, penalty, t_end, st)) return false;
        ws.states.push_back(std::move(st));
    }
    ws.route_epoch.assign(ws.states.size(), 1);
    for (std::int32_t r = 0; r < static_cast<std::int32_t>(ws.states.size()); ++r) {
        index_route(ws, r);
    }
    return true;
}

// Replace route r's vertices (fold rebuild). Empty vertices erase the route
// (swap-erase; the swapped route is reindexed). Returns false on a hard wall.
bool set_route(const Instance& inst, WarpSearch& ws, std::int32_t r,
               std::vector<std::int32_t> vertices, double penalty, double t_end) {
    if (vertices.empty()) {
        const std::int32_t last =
            static_cast<std::int32_t>(ws.states.size()) - 1;
        if (r != last) {
            ws.states[static_cast<std::size_t>(r)] =
                std::move(ws.states[static_cast<std::size_t>(last)]);
            ws.route_epoch[static_cast<std::size_t>(r)] =
                ws.route_epoch[static_cast<std::size_t>(last)];
            index_route(ws, r);
        }
        ws.states.pop_back();
        ws.route_epoch.pop_back();
        return true;
    }
    WarpRouteState st;
    if (!build_warp_route_state(inst, std::move(vertices), penalty, t_end, st)) {
        return false;
    }
    ws.states[static_cast<std::size_t>(r)] = std::move(st);
    index_route(ws, r);
    return true;
}

double search_cost(const WarpSearch& ws, double penalty) {
    double total = 0.0;
    for (const WarpRouteState& s : ws.states) total += warp_state_cost(s, penalty);
    return total;
}

bool search_feasible(const WarpSearch& ws) {
    for (const WarpRouteState& s : ws.states) {
        if (s.min_warp != 0.0) return false;
    }
    return true;
}

std::vector<std::vector<std::int32_t>> extract_routes(const WarpSearch& ws) {
    std::vector<std::vector<std::int32_t>> routes;
    routes.reserve(ws.states.size());
    for (const WarpRouteState& s : ws.states) routes.push_back(s.vertices);
    return routes;
}

std::vector<std::int32_t> without_at(const std::vector<std::int32_t>& vs,
                                     std::int64_t i) {
    std::vector<std::int32_t> out;
    out.reserve(vs.size() - 1);
    for (std::int64_t k = 0; k < static_cast<std::int64_t>(vs.size()); ++k) {
        if (k != i) out.push_back(vs[static_cast<std::size_t>(k)]);
    }
    return out;
}

std::vector<std::int32_t> with_inserted(const std::vector<std::int32_t>& vs,
                                        std::int64_t p, std::int32_t c) {
    std::vector<std::int32_t> out;
    out.reserve(vs.size() + 1);
    out.insert(out.end(), vs.begin(), vs.begin() + static_cast<std::ptrdiff_t>(p));
    out.push_back(c);
    out.insert(out.end(), vs.begin() + static_cast<std::ptrdiff_t>(p), vs.end());
    return out;
}

// Penalised removal ranking: route r without position i (0 for a singleton —
// the route vanishes).
double removal_cost(const Instance& inst, const WarpRouteState& r,
                    std::int64_t i, double penalty, double t_end, bool* ok) {
    if (r.vertices.size() == 1) {
        *ok = true;
        return 0.0;
    }
    const WarpRouteEval ev =
        evaluate_splice_warp(inst, r, i, i, r, 1, 0, penalty, t_end);
    *ok = ev.total;
    return ev.penalised;
}

// One first-improvement descent pass set over inter-route relocate + swap,
// granular justification, route-level staleness. Returns true if any move
// committed.
bool descend(const Instance& inst, const NeighbourLists& nb, WarpSearch& ws,
             double penalty, double t_end,
             const std::chrono::steady_clock::time_point* deadline) {
    const std::int32_t n = inst.num_customers;
    bool any_commit = false;
    bool improved = true;
    while (improved) {
        improved = false;
        for (std::int32_t c = 1; c <= n; ++c) {
            if ((c & 63) == 0 && deadline != nullptr &&
                std::chrono::steady_clock::now() >= *deadline) {
                return any_commit;
            }
            const std::int32_t r1 = ws.route_of[static_cast<std::size_t>(c)];
            if (r1 < 0) continue;
            // Route-level staleness: retest iff c's route or a granular
            // neighbour's route changed since c's stamp.
            std::int64_t relevant = ws.route_epoch[static_cast<std::size_t>(r1)];
            for (const std::int32_t* it = nb.neighbours_begin(c);
                 it != nb.neighbours_end(c); ++it) {
                const std::int32_t rv = ws.route_of[static_cast<std::size_t>(*it)];
                if (rv >= 0) {
                    relevant = std::max(
                        relevant, ws.route_epoch[static_cast<std::size_t>(rv)]);
                }
            }
            if (ws.client_stamp[static_cast<std::size_t>(c)] >= relevant) continue;

            const std::int64_t i1 = ws.pos_of[static_cast<std::size_t>(c)];
            WarpRouteState& s1 = ws.states[static_cast<std::size_t>(r1)];
            const double cost1 = warp_state_cost(s1, penalty);
            bool rem_ok = false;
            const double rem_cost =
                removal_cost(inst, s1, i1, penalty, t_end, &rem_ok);

            bool committed = false;
            for (const std::int32_t* it = nb.neighbours_begin(c);
                 it != nb.neighbours_end(c); ++it) {
                const std::int32_t v = *it;
                const std::int32_t r2 = ws.route_of[static_cast<std::size_t>(v)];
                if (r2 < 0 || r2 == r1) continue;
                WarpRouteState& s2 = ws.states[static_cast<std::size_t>(r2)];
                const double cost2 = warp_state_cost(s2, penalty);
                const std::int64_t iv = ws.pos_of[static_cast<std::size_t>(v)];

                // --- relocate c before/after v (capacity hard) ---
                if (rem_ok &&
                    s2.load + inst.demands[c] <= inst.vehicle_capacity) {
                    for (const std::int64_t p : {iv, iv + 1}) {
                        const WarpRouteEval ins = evaluate_splice_warp(
                            inst, s2, p, p - 1, s1, i1, i1, penalty, t_end);
                        if (!ins.total) continue;
                        const double delta =
                            (rem_cost + ins.penalised) - (cost1 + cost2);
                        if (!(delta < -kScreenEps)) continue;
                        // Repricing rule: rebuild + fold-account, revert
                        // unless strictly better.
                        std::vector<std::int32_t> n1 = without_at(s1.vertices, i1);
                        std::vector<std::int32_t> n2 =
                            with_inserted(s2.vertices, p, c);
                        WarpRouteState t2;
                        if (!build_warp_route_state(inst, std::move(n2), penalty,
                                                    t_end, t2)) {
                            continue;
                        }
                        double new_total = warp_state_cost(t2, penalty);
                        WarpRouteState t1;
                        const bool has1 = !n1.empty();
                        if (has1) {
                            if (!build_warp_route_state(inst, std::move(n1),
                                                        penalty, t_end, t1)) {
                                continue;
                            }
                            new_total += warp_state_cost(t1, penalty);
                        }
                        if (!(new_total < cost1 + cost2 - kScreenEps)) continue;
                        ws.states[static_cast<std::size_t>(r2)] = std::move(t2);
                        index_route(ws, r2);
                        ++ws.epoch;
                        ws.route_epoch[static_cast<std::size_t>(r2)] = ws.epoch;
                        if (has1) {
                            ws.states[static_cast<std::size_t>(r1)] = std::move(t1);
                            index_route(ws, r1);
                            ws.route_epoch[static_cast<std::size_t>(r1)] = ws.epoch;
                        } else {
                            set_route(inst, ws, r1, {}, penalty, t_end);
                        }
                        committed = true;
                        break;
                    }
                }
                if (committed) break;

                // --- swap c <-> v (capacity hard) ---
                if (s1.load - inst.demands[c] + inst.demands[v] <=
                        inst.vehicle_capacity &&
                    s2.load - inst.demands[v] + inst.demands[c] <=
                        inst.vehicle_capacity) {
                    const WarpRouteEval e1 = evaluate_splice_warp(
                        inst, s1, i1, i1, s2, iv, iv, penalty, t_end);
                    if (e1.total) {
                        const WarpRouteEval e2 = evaluate_splice_warp(
                            inst, s2, iv, iv, s1, i1, i1, penalty, t_end);
                        if (e2.total) {
                            const double delta = (e1.penalised + e2.penalised) -
                                                 (cost1 + cost2);
                            if (delta < -kScreenEps) {
                                std::vector<std::int32_t> n1 = s1.vertices;
                                n1[static_cast<std::size_t>(i1)] = v;
                                std::vector<std::int32_t> n2 = s2.vertices;
                                n2[static_cast<std::size_t>(iv)] = c;
                                WarpRouteState t1, t2;
                                if (build_warp_route_state(inst, std::move(n1),
                                                           penalty, t_end, t1) &&
                                    build_warp_route_state(inst, std::move(n2),
                                                           penalty, t_end, t2)) {
                                    const double new_total =
                                        warp_state_cost(t1, penalty) +
                                        warp_state_cost(t2, penalty);
                                    if (new_total < cost1 + cost2 - kScreenEps) {
                                        ws.states[static_cast<std::size_t>(r1)] =
                                            std::move(t1);
                                        ws.states[static_cast<std::size_t>(r2)] =
                                            std::move(t2);
                                        index_route(ws, r1);
                                        index_route(ws, r2);
                                        ++ws.epoch;
                                        ws.route_epoch[static_cast<std::size_t>(
                                            r1)] = ws.epoch;
                                        ws.route_epoch[static_cast<std::size_t>(
                                            r2)] = ws.epoch;
                                        committed = true;
                                    }
                                }
                            }
                        }
                    }
                }
                if (committed) break;
            }

            if (committed) {
                improved = true;
                any_commit = true;
            } else {
                ws.client_stamp[static_cast<std::size_t>(c)] = ws.epoch;
            }
        }
    }
    return any_commit;
}

// Draw the removal set: K ~ U[min, max] clients, seeds + granular
// neighbours, seeded random order (Fisher-Yates, modulo draws).
std::vector<std::int32_t> draw_removal_set(const Instance& inst,
                                           const NeighbourLists& nb,
                                           std::mt19937_64& rng,
                                           std::int32_t min_removals,
                                           std::int32_t max_removals) {
    const std::int32_t n = inst.num_customers;
    std::int32_t span = max_removals - min_removals + 1;
    if (span < 1) span = 1;
    std::int32_t target = min_removals + static_cast<std::int32_t>(
                                             rng() % static_cast<std::uint64_t>(span));
    if (target > n - 1) target = n - 1;

    std::vector<std::int32_t> order(static_cast<std::size_t>(n));
    for (std::int32_t c = 1; c <= n; ++c) order[static_cast<std::size_t>(c - 1)] = c;
    for (std::size_t k = order.size(); k > 1; --k) {
        const std::size_t j = static_cast<std::size_t>(rng() % k);
        std::swap(order[k - 1], order[j]);
    }

    std::vector<bool> removed(static_cast<std::size_t>(n) + 1, false);
    std::vector<std::int32_t> removed_list;
    for (const std::int32_t seed_c : order) {
        if (static_cast<std::int32_t>(removed_list.size()) >= target) break;
        if (removed[static_cast<std::size_t>(seed_c)]) continue;
        removed[static_cast<std::size_t>(seed_c)] = true;
        removed_list.push_back(seed_c);
        for (const std::int32_t* it = nb.neighbours_begin(seed_c);
             it != nb.neighbours_end(seed_c); ++it) {
            if (static_cast<std::int32_t>(removed_list.size()) >= target) break;
            if (!removed[static_cast<std::size_t>(*it)]) {
                removed[static_cast<std::size_t>(*it)] = true;
                removed_list.push_back(*it);
            }
        }
    }
    return removed_list;
}

// Ruin + penalised recreate of a given removal set on a live WarpSearch
// (which may be a sub-solution: fleet_slack = how many new singleton routes
// may still be opened globally). Undoes everything on failure.
bool ruin_recreate_warp(const Instance& inst, const NeighbourLists& nb,
                        WarpSearch& ws, std::mt19937_64& rng,
                        const std::vector<std::int32_t>& removal,
                        double penalty, double t_end, std::int64_t fleet_slack) {
    const std::int32_t n = inst.num_customers;
    if (n < 2 || ws.states.empty()) return false;
    const std::vector<std::vector<std::int32_t>> backup = extract_routes(ws);

    std::vector<bool> removed(static_cast<std::size_t>(n) + 1, false);
    std::vector<std::int32_t> removed_list;
    for (const std::int32_t c : removal) {
        if (ws.route_of[static_cast<std::size_t>(c)] < 0) continue;  // outside
        removed[static_cast<std::size_t>(c)] = true;
        removed_list.push_back(c);
    }
    if (removed_list.empty()) return false;

    // Ruin: rebuild every affected route without its removed clients.
    for (std::int32_t r = static_cast<std::int32_t>(ws.states.size()) - 1; r >= 0;
         --r) {
        const std::vector<std::int32_t>& vs =
            ws.states[static_cast<std::size_t>(r)].vertices;
        std::vector<std::int32_t> kept;
        kept.reserve(vs.size());
        for (const std::int32_t v : vs) {
            if (!removed[static_cast<std::size_t>(v)]) kept.push_back(v);
        }
        if (kept.size() == vs.size()) continue;
        for (const std::int32_t v : vs) {
            if (removed[static_cast<std::size_t>(v)]) {
                ws.route_of[static_cast<std::size_t>(v)] = -1;
            }
        }
        if (!set_route(inst, ws, r, std::move(kept), penalty, t_end)) {
            init_warp_search(inst, backup, penalty, t_end, ws);
            return false;
        }
        ++ws.epoch;
        if (static_cast<std::size_t>(r) < ws.route_epoch.size()) {
            ws.route_epoch[static_cast<std::size_t>(r)] = ws.epoch;
        }
    }

    // Recreate in random order at the best penalised position.
    for (std::size_t k = removed_list.size(); k > 1; --k) {
        const std::size_t j = static_cast<std::size_t>(rng() % k);
        std::swap(removed_list[k - 1], removed_list[j]);
    }
    std::int64_t opened = 0;
    for (const std::int32_t c : removed_list) {
        double best_cost = std::numeric_limits<double>::infinity();
        std::int32_t best_r = -1;
        std::int64_t best_p = -1;
        // Tree-ranked insertion (repricing rule: the winning position is
        // rebuilt through the fold at commit). Candidate positions: adjacent
        // to granular neighbours of c, plus both route ends.
        WarpRouteState single;
        if (!build_warp_route_state(inst, {c}, penalty, t_end, single)) {
            init_warp_search(inst, backup, penalty, t_end, ws);
            return false;
        }
        std::vector<char> tried;
        for (std::int32_t r = 0; r < static_cast<std::int32_t>(ws.states.size());
             ++r) {
            const WarpRouteState& s = ws.states[static_cast<std::size_t>(r)];
            if (s.load + inst.demands[c] > inst.vehicle_capacity) continue;
            const double base = warp_state_cost(s, penalty);
            const std::int64_t m = static_cast<std::int64_t>(s.vertices.size());
            tried.assign(static_cast<std::size_t>(m) + 1, 0);
            auto try_position = [&](std::int64_t p) {
                if (p < 0 || p > m || tried[static_cast<std::size_t>(p)]) return;
                tried[static_cast<std::size_t>(p)] = 1;
                const WarpRouteEval ins = evaluate_splice_warp(
                    inst, s, p, p - 1, single, 0, 0, penalty, t_end);
                if (!ins.total) return;
                const double cost = ins.penalised - base;
                if (cost < best_cost) {
                    best_cost = cost;
                    best_r = r;
                    best_p = p;
                }
            };
            try_position(0);
            try_position(m);
            for (const std::int32_t* it = nb.neighbours_begin(c);
                 it != nb.neighbours_end(c); ++it) {
                if (ws.route_of[static_cast<std::size_t>(*it)] != r) continue;
                const std::int64_t pv = ws.pos_of[static_cast<std::size_t>(*it)];
                try_position(pv);
                try_position(pv + 1);
            }
        }
        const bool fleet_open = fleet_slack < 0 || opened < fleet_slack;
        if (best_r < 0 && fleet_open) {
            ws.states.push_back(std::move(single));
            ws.route_epoch.push_back(++ws.epoch);
            index_route(ws, static_cast<std::int32_t>(ws.states.size()) - 1);
            ++opened;
            continue;
        }
        if (best_r < 0) {
            init_warp_search(inst, backup, penalty, t_end, ws);
            return false;
        }
        WarpRouteState& s = ws.states[static_cast<std::size_t>(best_r)];
        WarpRouteState rebuilt;
        if (!build_warp_route_state(inst, with_inserted(s.vertices, best_p, c),
                                    penalty, t_end, rebuilt)) {
            init_warp_search(inst, backup, penalty, t_end, ws);
            return false;
        }
        s = std::move(rebuilt);
        index_route(ws, best_r);
        ws.route_epoch[static_cast<std::size_t>(best_r)] = ++ws.epoch;
    }
    return true;
}

// Full-solution penalised perturbation (solve_warp_ils's kick).
bool perturb_warp(const Instance& inst, const NeighbourLists& nb, WarpSearch& ws,
                  std::mt19937_64& rng, std::int32_t min_removals,
                  std::int32_t max_removals, double penalty, double t_end) {
    const std::vector<std::int32_t> removal =
        draw_removal_set(inst, nb, rng, min_removals, max_removals);
    const std::int64_t fleet_slack =
        inst.num_vehicles < 0
            ? -1
            : inst.num_vehicles - static_cast<std::int64_t>(ws.states.size());
    return ruin_recreate_warp(inst, nb, ws, rng, removal, penalty, t_end,
                              fleet_slack);
}

}  // namespace

SolveResult solve_warp_ils(const Instance& inst, const WarpIlsParams& params,
                           std::uint64_t seed, double time_limit_seconds,
                           std::vector<std::vector<std::int32_t>> initial_routes) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto elapsed = [&start]() {
        return std::chrono::duration<double>(Clock::now() - start).count();
    };
    Clock::time_point deadline_point{};
    const Clock::time_point* deadline = nullptr;
    if (time_limit_seconds > 0.0) {
        deadline_point =
            start + std::chrono::duration_cast<Clock::duration>(
                        std::chrono::duration<double>(time_limit_seconds));
        deadline = &deadline_point;
    }
    const auto past = [&]() {
        return deadline != nullptr && Clock::now() >= *deadline;
    };

    SolveResult result;
    const double t_end = warp_horizon(inst);
    double penalty = params.penalty_init;

    std::vector<std::vector<std::int32_t>> seed_routes = std::move(initial_routes);
    if (seed_routes.empty() &&
        (!greedy_makespan(inst, seed_routes) ||
         solution_duration(inst, seed_routes) == kInfeasible)) {
        result.status = SolveStatus::Infeasible;
        return result;
    }
    const NeighbourLists nb =
        build_neighbour_lists(inst, params.num_neighbours, params.weight_wait);

    WarpSearch ws;
    if (!init_warp_search(inst, seed_routes, penalty, t_end, ws)) {
        result.status = SolveStatus::Infeasible;
        return result;
    }

    // Bank the feasible seed BEFORE the first penalised descent: the descent
    // may ride into warp territory and take a long time to return to
    // exactly-zero warp (observed on Lera2026 n=400) — the anytime contract
    // requires a feasible incumbent from t=0 like solve_ils's.
    double best = std::numeric_limits<double>::infinity();
    {
        const double seed_duration = solution_duration(inst, seed_routes);
        if (seed_duration != kInfeasible) {
            best = seed_duration;
            result.routes = seed_routes;
            result.value = best;
            result.incumbents.push_back({best, elapsed(), 0, 3});
        }
    }

    descend(inst, nb, ws, penalty, t_end, deadline);
    if (search_feasible(ws)) {
        const double duration = solution_duration(inst, extract_routes(ws));
        if (duration != kInfeasible && duration < best) {
            best = duration;
            result.routes = extract_routes(ws);
            result.value = best;
            result.incumbents.push_back({best, elapsed(), 0, 3});
        }
    }

    std::mt19937_64 rng(seed);
    const std::size_t history_len =
        params.history_length > 0 ? static_cast<std::size_t>(params.history_length)
                                  : 1;
    std::vector<double> history(history_len,
                                std::numeric_limits<double>::quiet_NaN());
    std::uint64_t history_idx = 0;
    double curr = search_cost(ws, penalty);
    const double init_cost = curr;

    std::int32_t window_count = 0;
    std::int32_t window_feasible = 0;
    SolveStatus status = SolveStatus::Finished;
    std::uint64_t iteration = 0;
    std::int64_t no_improvement = 0;
    std::vector<std::vector<std::int32_t>> snapshot;

    for (; iteration < params.max_iterations; ++iteration) {
        if (past()) {
            status = SolveStatus::TimeLimit;
            break;
        }
        if (no_improvement == params.restart_no_improvement &&
            !result.routes.empty()) {
            if (!init_warp_search(inst, result.routes, penalty, t_end, ws)) break;
            std::fill(history.begin(), history.end(),
                      std::numeric_limits<double>::quiet_NaN());
            history_idx = 0;
            no_improvement = 0;
        }

        snapshot = extract_routes(ws);
        const double snap_cost = search_cost(ws, penalty);
        perturb_warp(inst, nb, ws, rng, params.min_perturbations,
                     params.max_perturbations, penalty, t_end);
        descend(inst, nb, ws, penalty, t_end, deadline);
        const double cand = search_cost(ws, penalty);
        const bool cand_feasible = search_feasible(ws);

        ++no_improvement;
        if (cand_feasible) {
            const double duration = solution_duration(inst, extract_routes(ws));
            if (duration != kInfeasible && duration < best) {
                no_improvement = 0;
                best = duration;
                result.routes = extract_routes(ws);
                result.value = best;
                result.incumbents.push_back({best, elapsed(), iteration + 1, 3});
            }
        }

        // PyVRP penalty management on candidate feasibility.
        ++window_count;
        if (cand_feasible) ++window_feasible;
        if (window_count >= params.penalty_window) {
            const double frac =
                static_cast<double>(window_feasible) / window_count;
            penalty = frac >= params.target_feasible
                          ? penalty * params.penalty_decrease
                          : penalty * params.penalty_increase;
            penalty = std::min(std::max(penalty, params.penalty_min),
                               params.penalty_max);
            window_count = 0;
            window_feasible = 0;
        }

        // LAHC on the penalised cost (history values under older penalties
        // are tolerated, PyVRP-style).
        const std::size_t slot = static_cast<std::size_t>(
            history_idx % static_cast<std::uint64_t>(history_len));
        const bool slot_empty = std::isnan(history[slot]);
        const double late = slot_empty ? init_cost : history[slot];
        if (cand < late || cand < snap_cost) {
            curr = cand;
        } else {
            if (!init_warp_search(inst, snapshot, penalty, t_end, ws)) break;
            curr = snap_cost;
        }
        if (curr < late || slot_empty) history[slot] = curr;
        ++history_idx;
    }

    result.iterations_run = iteration;
    result.status = result.routes.empty() ? SolveStatus::Infeasible : status;
    return result;
}

namespace {

// solve_ils's rejection-restore snapshot, duplicated here (file-local there).
struct WkSnapshot {
    std::vector<std::vector<std::int32_t>> vertices;
    std::vector<std::int64_t> touched;
    std::vector<std::vector<std::int64_t>> last_tested;
    std::int64_t epoch = 0;
};

void wk_take_snapshot(const SearchState& ss, WkSnapshot& snap) {
    snap.vertices.clear();
    snap.vertices.reserve(ss.states.size());
    for (const RouteState& s : ss.states) snap.vertices.push_back(s.vertices);
    snap.touched = ss.touched;
    snap.last_tested = ss.last_tested;
    snap.epoch = ss.epoch;
}

void wk_restore_snapshot(const Instance& inst, SearchState& ss,
                         const WkSnapshot& snap) {
    ss.states.resize(snap.vertices.size());
    for (std::size_t k = 0; k < snap.vertices.size(); ++k) {
        if (ss.states[k].vertices != snap.vertices[k]) {
            const bool ok = build_route_state(inst, snap.vertices[k], ss.states[k]);
            (void)ok;
        }
    }
    ss.touched = snap.touched;
    ss.last_tested = snap.last_tested;
    ss.epoch = snap.epoch;
}

std::vector<std::vector<std::int32_t>> wk_extract(const SearchState& ss) {
    std::vector<std::vector<std::int32_t>> routes;
    routes.reserve(ss.states.size());
    for (const RouteState& s : ss.states) routes.push_back(s.vertices);
    return routes;
}

// Adopt the kicked routes into a live SearchState, preserving the RouteStates
// (trees, deletion caches) of unchanged routes and the staleness arrays —
// only the rebuilt routes' clients are marked touched (M7.1-style targeted
// marking, so the following granular descent rescans around the kick only).
bool wk_adopt_routes(const Instance& inst, SearchState& ss,
                     const std::vector<std::vector<std::int32_t>>& kicked) {
    std::vector<RouteState> old = std::move(ss.states);
    std::vector<char> used(old.size(), 0);
    ss.states.clear();
    ss.states.reserve(kicked.size());
    ++ss.epoch;
    for (const std::vector<std::int32_t>& r : kicked) {
        std::int64_t match = -1;
        for (std::size_t k = 0; k < old.size(); ++k) {
            if (!used[k] && old[k].vertices == r) {
                match = static_cast<std::int64_t>(k);
                break;
            }
        }
        if (match >= 0) {
            used[static_cast<std::size_t>(match)] = 1;
            ss.states.push_back(std::move(old[static_cast<std::size_t>(match)]));
        } else {
            RouteState st;
            if (!build_route_state(inst, r, st)) return false;
            for (const std::int32_t v : r) {
                ss.touched[static_cast<std::size_t>(v)] = ss.epoch;
            }
            ss.states.push_back(std::move(st));
        }
    }
    return true;
}

// The warp kick: draw the removal set, restrict the warp machinery to the
// affected SUB-solution (routes holding removed clients or their granular
// neighbours — the insertion targets), penalised ruin-recreate under
// p_explore, then a repair descent under p_repair until exactly-zero warp.
// Untouched routes never pay the warp-state cost, so the kick's price is
// K-scale, not n-scale. Returns the repaired feasible routes or nothing.
bool warp_kick(const Instance& inst, const NeighbourLists& nb,
               const std::vector<std::vector<std::int32_t>>& routes,
               std::mt19937_64& rng, const WarpKickIlsParams& params,
               double t_end,
               const std::chrono::steady_clock::time_point* deadline,
               std::vector<std::vector<std::int32_t>>* out) {
    const std::vector<std::int32_t> removal =
        draw_removal_set(inst, nb, rng, params.base.min_perturbations,
                         params.base.max_perturbations);
    if (removal.empty()) return false;

    std::vector<std::int32_t> route_of(
        static_cast<std::size_t>(inst.num_vertices()), -1);
    for (std::size_t r = 0; r < routes.size(); ++r) {
        for (const std::int32_t v : routes[r]) {
            route_of[static_cast<std::size_t>(v)] = static_cast<std::int32_t>(r);
        }
    }
    std::vector<char> in_subset(routes.size(), 0);
    for (const std::int32_t c : removal) {
        const std::int32_t rc = route_of[static_cast<std::size_t>(c)];
        if (rc >= 0) in_subset[static_cast<std::size_t>(rc)] = 1;
        for (const std::int32_t* it = nb.neighbours_begin(c);
             it != nb.neighbours_end(c); ++it) {
            const std::int32_t rv = route_of[static_cast<std::size_t>(*it)];
            if (rv >= 0) in_subset[static_cast<std::size_t>(rv)] = 1;
        }
    }
    std::vector<std::vector<std::int32_t>> sub_routes, other_routes;
    for (std::size_t r = 0; r < routes.size(); ++r) {
        (in_subset[r] ? sub_routes : other_routes).push_back(routes[r]);
    }
    if (sub_routes.empty()) return false;

    WarpSearch ws;
    if (!init_warp_search(inst, sub_routes, params.p_explore, t_end, ws)) {
        return false;
    }
    const std::int64_t fleet_slack =
        inst.num_vehicles < 0
            ? -1
            : inst.num_vehicles - static_cast<std::int64_t>(routes.size());
    if (!ruin_recreate_warp(inst, nb, ws, rng, removal, params.p_explore, t_end,
                            fleet_slack)) {
        return false;
    }
    if (!search_feasible(ws)) {
        // Repair under a dominating penalty, restricted to the warp-positive
        // routes and their granular surroundings.
        std::fill(ws.client_stamp.begin(), ws.client_stamp.end(), ws.epoch);
        ++ws.epoch;
        for (std::size_t r = 0; r < ws.states.size(); ++r) {
            if (ws.states[r].min_warp != 0.0) ws.route_epoch[r] = ws.epoch;
        }
        descend(inst, nb, ws, params.p_repair, t_end, deadline);
        if (!search_feasible(ws)) return false;
    }
    *out = std::move(other_routes);
    for (const WarpRouteState& s : ws.states) out->push_back(s.vertices);
    return true;
}

}  // namespace

SolveResult solve_ils_wk(const Instance& inst, const WarpKickIlsParams& params,
                         std::uint64_t seed, double time_limit_seconds,
                         std::vector<std::vector<std::int32_t>> initial_routes) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto elapsed = [&start]() {
        return std::chrono::duration<double>(Clock::now() - start).count();
    };
    Clock::time_point deadline_point{};
    const Clock::time_point* deadline = nullptr;
    if (time_limit_seconds > 0.0) {
        deadline_point =
            start + std::chrono::duration_cast<Clock::duration>(
                        std::chrono::duration<double>(time_limit_seconds));
        deadline = &deadline_point;
    }
    const auto past = [&]() {
        return deadline != nullptr && Clock::now() >= *deadline;
    };

    SolveResult result;
    const double t_end = warp_horizon(inst);

    std::vector<std::vector<std::int32_t>> seed_routes = std::move(initial_routes);
    if (seed_routes.empty() &&
        (!greedy_makespan(inst, seed_routes) ||
         solution_duration(inst, seed_routes) == kInfeasible)) {
        result.status = SolveStatus::Infeasible;
        return result;
    }
    const NeighbourLists nb = build_neighbour_lists(
        inst, params.base.num_neighbours, params.base.weight_wait);
    const NeighbourLists exhaustive;

    SearchState ss;
    if (!init_search_state(inst, seed_routes, ss)) {
        result.status = SolveStatus::Infeasible;
        return result;
    }
    double curr = ls_descend(inst, nb, ss, nullptr, deadline);

    double best = curr;
    result.routes = wk_extract(ss);
    result.value = best;
    result.incumbents.push_back({best, elapsed(), 0, 4});

    if (params.base.exhaustive_on_best && !past()) {
        mark_all_touched(ss);
        curr = ls_descend(inst, exhaustive, ss, nullptr, deadline);
        if (curr < best) {
            best = curr;
            result.routes = wk_extract(ss);
            result.value = best;
            result.incumbents.push_back({best, elapsed(), 0, 4});
        }
    }

    std::mt19937_64 rng(seed);
    PerturbParams fallback_params;
    fallback_params.min_removals = params.base.min_perturbations;
    fallback_params.max_removals = params.base.max_perturbations;

    const std::size_t history_len =
        params.base.history_length > 0
            ? static_cast<std::size_t>(params.base.history_length)
            : 1;
    std::vector<double> history(history_len,
                                std::numeric_limits<double>::quiet_NaN());
    std::uint64_t history_idx = 0;
    const double init_cost = curr;

    WkSnapshot snapshot;
    SolveStatus status = SolveStatus::Finished;
    std::uint64_t iteration = 0;
    std::int64_t no_improvement = 0;

    for (; iteration < params.base.max_iterations; ++iteration) {
        if (past()) {
            status = SolveStatus::TimeLimit;
            break;
        }
        if (no_improvement == params.base.restart_no_improvement) {
            if (!init_search_state(inst, result.routes, ss)) break;
            curr = best;
            std::fill(history.begin(), history.end(),
                      std::numeric_limits<double>::quiet_NaN());
            history_idx = 0;
            no_improvement = 0;
        }

        wk_take_snapshot(ss, snapshot);

        // Warp kick first; feasible-only M7.1 kick as the fallback.
        std::vector<std::vector<std::int32_t>> kicked;
        if (warp_kick(inst, nb, wk_extract(ss), rng, params, t_end, deadline,
                      &kicked)) {
            if (!wk_adopt_routes(inst, ss, kicked)) {
                wk_restore_snapshot(inst, ss, snapshot);
                perturb(inst, nb, ss, rng, fallback_params);
            }
        } else {
            perturb(inst, nb, ss, rng, fallback_params);
        }
        double cand = ls_descend(inst, nb, ss, nullptr, deadline);

        ++no_improvement;
        if (cand < best) {
            no_improvement = 0;
            if (params.base.exhaustive_on_best && !past()) {
                mark_all_touched(ss);
                cand = ls_descend(inst, exhaustive, ss, nullptr, deadline);
            }
            best = cand;
            result.routes = wk_extract(ss);
            result.value = best;
            result.incumbents.push_back({best, elapsed(), iteration + 1, 4});
        }

        const std::size_t slot = static_cast<std::size_t>(
            history_idx % static_cast<std::uint64_t>(history_len));
        const bool slot_empty = std::isnan(history[slot]);
        const double late = slot_empty ? init_cost : history[slot];
        if (cand < late || cand < curr) {
            curr = cand;
        } else {
            wk_restore_snapshot(inst, ss, snapshot);
        }
        if (curr < late || slot_empty) history[slot] = curr;
        ++history_idx;
    }

    result.iterations_run = iteration;
    result.status = result.routes.empty() ? SolveStatus::Infeasible : status;
    return result;
}

}  // namespace kayros
