#include "ls/squeeze.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <utility>

#include "ls/rng.h"

namespace kayros {

namespace {

constexpr double kScreenEps = 1e-9;  // local_search.cpp's screening epsilon
// Dominating repair-leg weight: one ms of warp outweighs any single-move
// duration delta by orders of magnitude, so repair descents only ever trade
// warp down (the P8.3 lesson made structural). Constant by design, not a
// knob: the repair leg exists to reach zero warp, not to be calibrated.
constexpr double kRepairPenalty = 1e8;

// Whole-search state: warp route states + client->route bookkeeping +
// route-level staleness. Transplanted verbatim from the td-time-warp
// branch's warp_ils.cpp (P8.3), with work-budget charging and zero-warp
// checkpoint banking added; see the plan-15 M1 memo.
struct WarpSearch {
    std::vector<WarpRouteState> states;
    std::vector<std::int32_t> route_of;  // vertex id -> state index (-1 = none)
    std::vector<std::int64_t> pos_of;    // vertex id -> position in its route
    std::vector<std::int64_t> route_epoch;
    std::vector<std::int64_t> client_stamp;
    std::int64_t epoch = 1;
};

void index_route(WarpSearch& ws, std::int32_t r) {
    const std::vector<std::int32_t>& vs =
        ws.states[static_cast<std::size_t>(r)].vertices;
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
    for (std::int32_t r = 0; r < static_cast<std::int32_t>(ws.states.size());
         ++r) {
        index_route(ws, r);
    }
    return true;
}

// Replace route r's vertices (fold rebuild). Empty vertices erase the route
// (swap-erase; the swapped route is reindexed). Returns false on a hard wall.
bool set_route(const Instance& inst, WarpSearch& ws, std::int32_t r,
               std::vector<std::int32_t> vertices, double penalty,
               double t_end) {
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

bool search_feasible(const WarpSearch& ws) {
    for (const WarpRouteState& s : ws.states) {
        if (s.min_warp != 0.0) return false;
    }
    return true;
}

// Exact zero-warp objective (duration sum plus the fleet term: a penalised
// relocate can empty a route mid-phase, so banks at different K must be
// compared FleetCostDuration-priced). Only meaningful when search_feasible.
double feasible_objective(const Instance& inst, const WarpSearch& ws) {
    double total =
        inst.fixed_route_cost * static_cast<double>(ws.states.size());
    for (const WarpRouteState& s : ws.states) total += s.duration;
    return total;
}

std::vector<std::vector<std::int32_t>> extract_ws_routes(const WarpSearch& ws) {
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

// The banking context threaded through the descent: every committed move
// that lands on an exactly-zero-warp state strictly below the best banked
// duration snapshots the routes. The phase result is the best bank, never
// the final descent state.
struct Bank {
    double best_objective;
    std::vector<std::vector<std::int32_t>> routes;  // empty = entry state
    std::int64_t checkpoints = 0;
};

void maybe_bank(const Instance& inst, const WarpSearch& ws, Bank& bank) {
    if (!search_feasible(ws)) return;
    const double d = feasible_objective(inst, ws);
    if (d < bank.best_objective - kScreenEps) {
        bank.best_objective = d;
        bank.routes = extract_ws_routes(ws);
        ++bank.checkpoints;
    }
}

// Penalised removal ranking: route r without position i (0 for a singleton,
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

// One first-improvement descent over inter-route relocate + swap, granular
// justification, route-level staleness, at weight `penalty`. Charges every
// penalised pricing against *budget and stops when it falls to `floor` or
// below. stop_when_feasible: exit as soon as every route has exactly-zero
// warp (the repair mode). Banks every zero-warp improvement it commits.
void descend(const Instance& inst, const NeighbourLists& nb, WarpSearch& ws,
             double penalty, double t_end,
             const std::chrono::steady_clock::time_point* deadline,
             std::int64_t* budget, std::int64_t floor, SqueezeStats* stats,
             Bank& bank, bool stop_when_feasible) {
    const std::int32_t n = inst.num_customers;
    const auto charge = [&](std::int64_t units) {
        if (stats) stats->evaluated += units;
        *budget -= units;
    };
    // The exhaustive sentinel (k == 0) carries no materialized lists
    // (neighbours_begin is out of bounds there): fall back to all clients.
    std::vector<std::int32_t> all;
    if (!nb.restricted()) {
        all.reserve(static_cast<std::size_t>(n));
        for (std::int32_t v = 1; v <= n; ++v) all.push_back(v);
    }
    const auto cand_begin = [&](std::int32_t c) {
        return nb.restricted() ? nb.neighbours_begin(c) : all.data();
    };
    const auto cand_end = [&](std::int32_t c) {
        return nb.restricted() ? nb.neighbours_end(c) : all.data() + all.size();
    };
    if (stop_when_feasible && search_feasible(ws)) return;
    bool improved = true;
    while (improved) {
        improved = false;
        for (std::int32_t c = 1; c <= n; ++c) {
            if (*budget <= floor) return;
            if ((c & 63) == 0 && deadline != nullptr &&
                std::chrono::steady_clock::now() >= *deadline) {
                return;
            }
            const std::int32_t r1 = ws.route_of[static_cast<std::size_t>(c)];
            if (r1 < 0) continue;
            // Route-level staleness: retest iff c's route or a granular
            // neighbour's route changed since c's stamp.
            std::int64_t relevant = ws.route_epoch[static_cast<std::size_t>(r1)];
            for (const std::int32_t* it = cand_begin(c);
                 it != cand_end(c); ++it) {
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
            charge(1);
            const double rem_cost =
                removal_cost(inst, s1, i1, penalty, t_end, &rem_ok);

            bool committed = false;
            for (const std::int32_t* it = cand_begin(c);
                 it != cand_end(c); ++it) {
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
                        charge(1);
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
                        charge(1);
                        if (!build_warp_route_state(inst, std::move(n2), penalty,
                                                    t_end, t2)) {
                            continue;
                        }
                        double new_total = warp_state_cost(t2, penalty);
                        WarpRouteState t1;
                        const bool has1 = !n1.empty();
                        if (has1) {
                            charge(1);
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
                    charge(1);
                    const WarpRouteEval e1 = evaluate_splice_warp(
                        inst, s1, i1, i1, s2, iv, iv, penalty, t_end);
                    if (e1.total) {
                        charge(1);
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
                                charge(2);
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
                maybe_bank(inst, ws, bank);
                if (stop_when_feasible && search_feasible(ws)) return;
            } else {
                ws.client_stamp[static_cast<std::size_t>(c)] = ws.epoch;
            }
        }
    }
}

}  // namespace

bool squeeze_phase(const Instance& inst, const NeighbourLists& nb,
                   std::vector<std::vector<std::int32_t>>& routes,
                   const SqueezeParams& params,
                   const std::chrono::steady_clock::time_point* deadline,
                   SqueezeStats* stats) {
    if (params.work_cap <= 0) return false;
    const double t_end = warp_horizon(inst);
    WarpSearch ws;
    if (!init_warp_search(inst, routes, params.penalty, t_end, ws)) {
        return false;  // entry state hits a hard wall: not S2's business
    }
    if (stats) ++stats->phases;

    Bank bank;
    bank.best_objective = feasible_objective(inst, ws);  // entry is feasible

    std::int64_t budget = params.work_cap;
    // Explore leg at P, reserving a repair tail (~20 % of the budget) so a
    // warp-positive final state can still be driven back to zero warp.
    const std::int64_t reserve = params.work_cap / 5;
    descend(inst, nb, ws, params.penalty, t_end, deadline, &budget, reserve,
            stats, bank, /*stop_when_feasible=*/false);
    // Repair leg at the dominating weight, exiting at the first exactly-zero
    // warp state (P8.3), banking it when improving.
    if (!search_feasible(ws)) {
        descend(inst, nb, ws, kRepairPenalty, t_end, deadline, &budget, 0,
                stats, bank, /*stop_when_feasible=*/true);
        maybe_bank(inst, ws, bank);
    }

    if (stats) stats->checkpoints += bank.checkpoints;
    if (bank.routes.empty()) return false;  // nothing beat the entry state
    routes = std::move(bank.routes);
    if (stats) ++stats->improved;
    return true;
}

bool squeeze_insert(const Instance& inst, const NeighbourLists& nb,
                    std::vector<std::vector<std::int32_t>>& routes,
                    std::int32_t c, double penalty, std::int64_t* work_budget,
                    std::mt19937_64& rng, SqueezeStats* stats) {
    const double t_end = warp_horizon(inst);
    WarpSearch ws;
    if (!init_warp_search(inst, routes, penalty, t_end, ws)) return false;
    const auto charge = [&](std::int64_t units) {
        if (stats) stats->evaluated += units;
        *work_budget -= units;
    };
    charge(static_cast<std::int64_t>(ws.states.size()));  // the state builds

    WarpRouteState c_single;
    charge(1);
    if (!build_warp_route_state(inst, {c}, penalty, t_end, c_single)) {
        return false;
    }

    // Min-penalised force-insert: candidate positions are route ends plus
    // positions adjacent to granular neighbours of c (the branch's recreate
    // primitive); every position of every route when exhaustive.
    struct Ins {
        bool found = false;
        std::int32_t route = 0;
        std::int64_t pos = 0;
        double delta = 0.0;
    } best;
    const auto consider = [&](std::int32_t r, std::int64_t p) {
        const WarpRouteState& s = ws.states[static_cast<std::size_t>(r)];
        if (s.load + inst.demands[static_cast<std::size_t>(c)] >
            inst.vehicle_capacity) {
            return;
        }
        charge(1);
        const WarpRouteEval ev =
            evaluate_splice_warp(inst, s, p, p - 1, c_single, 0, 0, penalty, t_end);
        if (!ev.total) return;
        const double delta = ev.penalised - warp_state_cost(s, penalty);
        if (!best.found || delta < best.delta ||
            (delta == best.delta &&
             (r < best.route || (r == best.route && p < best.pos)))) {
            best = {true, r, p, delta};
        }
    };
    if (!nb.restricted()) {
        for (std::int32_t r = 0; r < static_cast<std::int32_t>(ws.states.size());
             ++r) {
            const std::int64_t m = static_cast<std::int64_t>(
                ws.states[static_cast<std::size_t>(r)].vertices.size());
            for (std::int64_t p = 0; p <= m; ++p) consider(r, p);
        }
    } else {
        // Route ends first, then neighbour-adjacent positions, deduplicated
        // via a per-route visited-position set kept as a sorted probe.
        std::vector<std::pair<std::int32_t, std::int64_t>> seen;
        const auto push = [&](std::int32_t r, std::int64_t p) {
            const auto key = std::make_pair(r, p);
            for (const auto& s : seen) {
                if (s == key) return;
            }
            seen.push_back(key);
            consider(r, p);
        };
        for (std::int32_t r = 0; r < static_cast<std::int32_t>(ws.states.size());
             ++r) {
            const std::int64_t m = static_cast<std::int64_t>(
                ws.states[static_cast<std::size_t>(r)].vertices.size());
            push(r, 0);
            push(r, m);
        }
        for (const std::int32_t* it = nb.neighbours_begin(c);
             it != nb.neighbours_end(c); ++it) {
            const std::int32_t r = ws.route_of[static_cast<std::size_t>(*it)];
            if (r < 0) continue;
            const std::int64_t pv = ws.pos_of[static_cast<std::size_t>(*it)];
            push(r, pv);
            push(r, pv + 1);
        }
    }
    if (!best.found) return false;
    {
        std::vector<std::int32_t> nv = with_inserted(
            ws.states[static_cast<std::size_t>(best.route)].vertices, best.pos, c);
        charge(1);
        if (!set_route(inst, ws, best.route, std::move(nv), penalty, t_end)) {
            return false;
        }
        ++ws.epoch;
        ws.route_epoch[static_cast<std::size_t>(best.route)] = ws.epoch;
    }

    // Confined reference repair: one warp-positive route at a time.
    while (!search_feasible(ws)) {
        if (*work_budget <= 0) return false;
        std::vector<std::int32_t> bad;
        for (std::int32_t r = 0; r < static_cast<std::int32_t>(ws.states.size());
             ++r) {
            if (ws.states[static_cast<std::size_t>(r)].min_warp != 0.0) {
                bad.push_back(r);
            }
        }
        const std::int32_t rsel = bad[static_cast<std::size_t>(
            draw(rng, static_cast<std::uint64_t>(bad.size())))];
        const WarpRouteState& sr = ws.states[static_cast<std::size_t>(rsel)];
        const double cost_r = warp_state_cost(sr, penalty);

        struct Move {
            bool found = false;
            bool is_swap = false;
            std::int32_t u = 0, v = 0;
            std::int32_t r2 = 0;
            std::int64_t p = 0;  // relocate target position in r2
            double delta = 0.0;
        } mv;
        const std::vector<std::int32_t> members = sr.vertices;  // copy: sr moves
        // Exhaustive-sentinel fallback, as in descend: all clients when the
        // lists are not materialized.
        std::vector<std::int32_t> all;
        if (!nb.restricted()) {
            all.reserve(static_cast<std::size_t>(inst.num_customers));
            for (std::int32_t v = 1; v <= inst.num_customers; ++v) {
                all.push_back(v);
            }
        }
        for (const std::int32_t u : members) {
            if (*work_budget <= 0) break;
            const std::int64_t iu = ws.pos_of[static_cast<std::size_t>(u)];
            bool rem_ok = false;
            charge(1);
            const double rem_cost =
                removal_cost(inst, sr, iu, penalty, t_end, &rem_ok);
            const std::int32_t* vb =
                nb.restricted() ? nb.neighbours_begin(u) : all.data();
            const std::int32_t* ve =
                nb.restricted() ? nb.neighbours_end(u) : all.data() + all.size();
            for (const std::int32_t* it = vb; it != ve; ++it) {
                const std::int32_t v = *it;
                const std::int32_t r2 = ws.route_of[static_cast<std::size_t>(v)];
                if (r2 < 0 || r2 == rsel) continue;
                const WarpRouteState& s2 = ws.states[static_cast<std::size_t>(r2)];
                const double cost2 = warp_state_cost(s2, penalty);
                const std::int64_t iv = ws.pos_of[static_cast<std::size_t>(v)];
                // relocate u out of rsel, before/after v
                if (rem_ok && s2.load + inst.demands[static_cast<std::size_t>(u)] <=
                                  inst.vehicle_capacity) {
                    for (const std::int64_t p : {iv, iv + 1}) {
                        charge(1);
                        const WarpRouteEval ins = evaluate_splice_warp(
                            inst, s2, p, p - 1, sr, iu, iu, penalty, t_end);
                        if (!ins.total) continue;
                        const double delta =
                            (rem_cost + ins.penalised) - (cost_r + cost2);
                        if (delta < -kScreenEps &&
                            (!mv.found || delta < mv.delta)) {
                            // NOTE: the reference's early-commit cutoff
                            // (stop scanning when a move wipes the selected
                            // route's whole penalty, M0.7 trick 3) is
                            // deliberately not in v0: it needs the removal
                            // eval's residual warp threaded through here,
                            // and it is an optimization, not semantics.
                            mv = {true, false, u, v, r2, p, delta};
                        }
                    }
                }
                // swap u <-> v
                if (ws.states[static_cast<std::size_t>(rsel)].load -
                            inst.demands[static_cast<std::size_t>(u)] +
                            inst.demands[static_cast<std::size_t>(v)] <=
                        inst.vehicle_capacity &&
                    s2.load - inst.demands[static_cast<std::size_t>(v)] +
                            inst.demands[static_cast<std::size_t>(u)] <=
                        inst.vehicle_capacity) {
                    charge(2);
                    const WarpRouteEval e1 = evaluate_splice_warp(
                        inst, sr, iu, iu, s2, iv, iv, penalty, t_end);
                    if (!e1.total) continue;
                    const WarpRouteEval e2 = evaluate_splice_warp(
                        inst, s2, iv, iv, sr, iu, iu, penalty, t_end);
                    if (!e2.total) continue;
                    const double delta =
                        (e1.penalised + e2.penalised) - (cost_r + cost2);
                    if (delta < -kScreenEps && (!mv.found || delta < mv.delta)) {
                        mv = {true, true, u, v, r2, 0, delta};
                    }
                }
            }
        }
        if (!mv.found) return false;  // stuck: the whole squeeze fails

        // Commit through the fold, revert-on-non-improvement (repricing rule).
        const std::int64_t iu = ws.pos_of[static_cast<std::size_t>(mv.u)];
        const double before =
            warp_state_cost(ws.states[static_cast<std::size_t>(rsel)], penalty) +
            warp_state_cost(ws.states[static_cast<std::size_t>(mv.r2)], penalty);
        std::vector<std::int32_t> n1, n2;
        if (mv.is_swap) {
            const std::int64_t iv = ws.pos_of[static_cast<std::size_t>(mv.v)];
            n1 = ws.states[static_cast<std::size_t>(rsel)].vertices;
            n1[static_cast<std::size_t>(iu)] = mv.v;
            n2 = ws.states[static_cast<std::size_t>(mv.r2)].vertices;
            n2[static_cast<std::size_t>(iv)] = mv.u;
        } else {
            n1 = without_at(ws.states[static_cast<std::size_t>(rsel)].vertices, iu);
            n2 = with_inserted(ws.states[static_cast<std::size_t>(mv.r2)].vertices,
                               mv.p, mv.u);
        }
        WarpRouteState t1, t2;
        const bool has1 = !n1.empty();
        charge(has1 ? 2 : 1);
        if (!build_warp_route_state(inst, std::move(n2), penalty, t_end, t2)) {
            return false;
        }
        double after = warp_state_cost(t2, penalty);
        if (has1) {
            if (!build_warp_route_state(inst, std::move(n1), penalty, t_end, t1)) {
                return false;
            }
            after += warp_state_cost(t1, penalty);
        }
        if (!(after < before - kScreenEps)) return false;
        ws.states[static_cast<std::size_t>(mv.r2)] = std::move(t2);
        index_route(ws, mv.r2);
        ++ws.epoch;
        ws.route_epoch[static_cast<std::size_t>(mv.r2)] = ws.epoch;
        if (has1) {
            ws.states[static_cast<std::size_t>(rsel)] = std::move(t1);
            index_route(ws, rsel);
            ws.route_epoch[static_cast<std::size_t>(rsel)] = ws.epoch;
        } else {
            set_route(inst, ws, rsel, {}, penalty, t_end);
        }
    }

    routes = extract_ws_routes(ws);
    return true;
}

}  // namespace kayros
