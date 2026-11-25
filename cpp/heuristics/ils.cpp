#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "core/queries.h"
#include "heuristics/heuristics.h"
#include "ls/fleet_descent.h"
#include "ls/ls.h"
#include "ls/squeeze.h"
#include "ls/perturb.h"

namespace kayros {

namespace {

// Rejection restore: vertex vectors + the full staleness bookkeeping. The
// staleness arrays MUST travel with the solution — stamps taken while the
// rejected candidate was live certify move coverage for THAT solution, not
// for the restored one.
struct IlsSnapshot {
    std::vector<std::vector<std::int32_t>> vertices;
    std::vector<std::int64_t> touched;
    std::vector<std::vector<std::int64_t>> last_tested;
    std::int64_t epoch = 0;
};

void take_snapshot(const SearchState& ss, IlsSnapshot& snap) {
    snap.vertices.clear();
    snap.vertices.reserve(ss.states.size());
    for (const RouteState& s : ss.states) snap.vertices.push_back(s.vertices);
    snap.touched = ss.touched;
    snap.last_tested = ss.last_tested;
    snap.epoch = ss.epoch;
}

void restore_snapshot(const Instance& inst, SearchState& ss,
                      const IlsSnapshot& snap) {
    ss.states.resize(snap.vertices.size());
    for (std::size_t k = 0; k < snap.vertices.size(); ++k) {
        if (ss.states[k].vertices != snap.vertices[k]) {
            const bool ok =
                build_route_state(inst, snap.vertices[k], ss.states[k]);
            (void)ok;  // snapshot solutions were feasible by invariant
        }
    }
    ss.touched = snap.touched;
    ss.last_tested = snap.last_tested;
    ss.epoch = snap.epoch;
}

std::vector<std::vector<std::int32_t>> extract_routes(const SearchState& ss) {
    std::vector<std::vector<std::int32_t>> routes;
    routes.reserve(ss.states.size());
    for (const RouteState& s : ss.states) routes.push_back(s.vertices);
    return routes;
}

}  // namespace

SolveResult solve_ils(const Instance& inst, const IlsParams& params,
                      std::uint64_t seed, double time_limit,
                      const IncumbentCallback& on_incumbent,
                      std::vector<std::vector<std::int32_t>> initial_routes,
                      const IncumbentGate& accept_incumbent) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto elapsed = [&start]() {
        return std::chrono::duration<double>(Clock::now() - start).count();
    };
    Clock::time_point deadline_point{};
    const Clock::time_point* deadline = nullptr;
    if (time_limit > 0.0) {
        deadline_point = start + std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(
                                         time_limit));
        deadline = &deadline_point;
    }
    const auto past = [&]() {
        return deadline != nullptr && Clock::now() >= *deadline;
    };

    SolveResult result;

    // Count candidate pricings across descents and record work at the first in-cap publication.
    LsStats ls_stats;

    // Constrain the fleet only when the objective prices routes.
    const bool cap_active = inst.fixed_route_cost > 0.0 && params.k_cap > 0;

    // Single publication point for the anytime stream: records the incumbent,
    // fires the hook, and keeps the stream strictly decreasing.
    double published = std::numeric_limits<double>::infinity();
    const auto publish =
        [&](double value, std::uint64_t iteration, std::int32_t origin,
            const std::vector<std::vector<std::int32_t>>& routes, bool from_squeeze) {
            const std::int64_t k = static_cast<std::int64_t>(routes.size());
            // Above-cap solutions can guide the search but cannot enter the incumbent stream.
            if (cap_active && k > params.k_cap) return;
            if (!(value < published)) return;
            published = value;
            if (result.incumbents.empty()) {
                result.k_stats.k_best_min = k;
                result.k_stats.k_best_max = k;
            } else {
                if (k < result.k_stats.k_best_min) result.k_stats.k_best_min = k;
                if (k > result.k_stats.k_best_max) result.k_stats.k_best_max = k;
            }
            result.incumbents.push_back({value, elapsed(), iteration, origin, from_squeeze});
            if (cap_active && result.k_cap_stats.reached_cap == 0) {
                result.k_cap_stats.reached_cap = 1;
                result.k_cap_stats.first_capped_work = ls_stats.evaluated;
            }
            if (on_incumbent) on_incumbent(result.incumbents.back(), routes);
        };

    double best = std::numeric_limits<double>::infinity();
    const auto consider = [&](double value, std::uint64_t iteration, std::int32_t origin,
                              const std::vector<std::vector<std::int32_t>>& routes,
                              bool from_squeeze = false) {
        if (!(value < best)) return false;
        if (accept_incumbent) {
            if (cap_active && static_cast<std::int64_t>(routes.size()) > params.k_cap)
                return false;
            if (!accept_incumbent(routes, from_squeeze)) return false;
        }
        best = value;
        result.value = value;
        result.routes = routes;
        publish(value, iteration, origin, routes, from_squeeze);
        return true;
    };
    const auto seed_incumbent = [&](double value, const auto& routes) {
        if (accept_incumbent) consider(value, 0, 0, routes);
        else publish(value, 0, 0, routes, false);
    };

    // Seed (warm-start routes when provided, greedy otherwise) + descent.
    const bool warm_start = !initial_routes.empty();
    std::vector<std::vector<std::int32_t>> seed_routes =
        std::move(initial_routes);
    if (seed_routes.empty()) {
        if (!greedy_construct(inst, seed_routes)) {
            result.status = SolveStatus::Infeasible;
            return result;
        }
        const double seed_value = solution_cost(inst, seed_routes);
        if (seed_value == kInfeasible) {
            result.status = SolveStatus::Infeasible;
            return result;
        }
        // Submit the raw greedy seed before the first descent. The warm-start path submits its validated seed separately.
        seed_incumbent(seed_value, seed_routes);

        // Under Duration, split the constructed routes before descent to vary the initial fleet size. A positive fixed charge disables splitting.
        if (inst.fixed_route_cost == 0.0 && params.seed_k_factor > 1.0) {
            const std::int32_t target = static_cast<std::int32_t>(
                params.seed_k_factor * static_cast<double>(seed_routes.size()));
            std::vector<std::vector<std::int32_t>> split_routes = seed_routes;
            if (split_to_k(inst, split_routes, target)) {
                const double split_value = solution_cost(inst, split_routes);
                if (split_value != kInfeasible) {
                    seed_routes = std::move(split_routes);
                    // Publish only if it is actually better: splitting usually
                    // costs duration up front and pays off through the search,
                    // and the stream must stay strictly decreasing.
                    seed_incumbent(split_value, seed_routes);
                }
            }
        }
    }
    const NeighbourLists nb =
        build_neighbour_lists(inst, params.num_neighbours, params.weight_wait);
    const NeighbourLists exhaustive;  // k == 0 sentinel

    SearchState ss;
    if (!init_search_state(inst, seed_routes, ss)) {
        result.status = SolveStatus::Infeasible;
        return result;
    }
    if (warm_start && accept_incumbent)
        consider(solution_cost(inst, seed_routes), 0, 3, seed_routes);
    result.k_stats.k_seed = static_cast<std::int64_t>(seed_routes.size());
    double curr = ls_descend(inst, nb, ss, &ls_stats, deadline);

    consider(curr, 0, warm_start && accept_incumbent ? 2 : 0, extract_routes(ss));

    // Exhaustive polish of the initial best (PyVRP's exhaustive_on_best).
    if (params.exhaustive_on_best && !past()) {
        mark_all_touched(ss);
        curr = ls_descend(inst, exhaustive, ss, &ls_stats, deadline);
        if (curr < best) {
            consider(curr, 0, 2, extract_routes(ss));
        }
    }

    std::mt19937_64 rng(seed);
    PerturbParams perturb_params;
    perturb_params.min_perturbations = params.min_perturbations;
    perturb_params.max_perturbations = params.max_perturbations;
    perturb_params.dissolve_pct = params.dissolve_pct;

    // LAHC slots (Burke & Bykov 2017 with PyVRP's RingBuffer semantics: the
    // index advances every iteration; a slot is rewritten only when the
    // current solution improves on it). Empty slots fall back to the initial
    // cost.
    const std::size_t history_len =
        params.lahc_history > 0
            ? static_cast<std::size_t>(params.lahc_history)
            : 1;
    std::vector<double> history(history_len,
                                std::numeric_limits<double>::quiet_NaN());
    std::uint64_t history_idx = 0;
    const double init_cost = curr;

    IlsSnapshot snapshot;
    SolveStatus status = SolveStatus::Finished;
    std::uint64_t iteration = 0;
    std::int64_t no_improvement = 0;

    // Fleet descent consumes random draws only when the objective prices routes.
    FleetDescentParams fd_params;
    fd_params.fd_k_max = params.fd_k_max;
    fd_params.fd_ep_budget = params.fd_ep_budget;
    fd_params.sq_ladder = params.sq_ladder;
    fd_params.sq_penalty = params.sq_penalty;
    std::vector<std::int32_t> fd_pcount(
        static_cast<std::size_t>(inst.num_customers) + 1, 0);
    const bool fd_armed = inst.fixed_route_cost > 0.0 && params.fd_attempts > 0;
    // Work-based trigger marks: window baselines in ls_stats.evaluated units,
    // reset at loop entry, on every FD trigger (fd) and improvement/restart
    // (improve). See IlsParams::fd_period_work / restart_no_improvement_work.
    std::int64_t fd_work_mark = 0;
    std::int64_t improve_work_mark = 0;
    // Up to fd_attempts dissolve attempts on the live state; on success the
    // K-1 solution gets the full basin treatment (granular descent +
    // exhaustive polish) and the repriced value is returned (NaN when no
    // attempt stuck; the state is then bitwise unperturbed).
    const auto run_fleet_descent = [&]() -> double {
        ++result.fd_stats.triggers;
        // Each candidate pricing consumes one unit of the per-trigger drain budget. The global deadline still ends the run.
        std::int64_t fd_budget = params.fd_work_cap;
        std::int64_t* budget_ptr = params.fd_work_cap > 0 ? &fd_budget : nullptr;
        const std::int64_t basin_mark = ls_stats.evaluated;
        bool dropped = false;
        bool from_squeeze = false;
        for (std::int32_t a = 0; a < params.fd_attempts; ++a) {
            if (budget_ptr != nullptr && fd_budget <= 0) break;
            if (deadline != nullptr && Clock::now() >= *deadline) break;
            const auto rescues = result.fd_stats.ladder_rescues;
            if (fleet_descent(inst, nb, ss, rng, fd_params, fd_pcount,
                              deadline, budget_ptr, &result.fd_stats)) {
                dropped = true;
                from_squeeze = result.fd_stats.ladder_rescues > rescues;
                break;
            }
        }
        // Squeeze the post-drain state when enabled. Count its work separately and adopt only a zero-warp improvement that rebuilds as feasible.
        const auto try_squeeze = [&]() -> bool {
            SqueezeParams sqp;
            sqp.sq_penalty = params.sq_penalty;
            sqp.sq_work_cap = params.sq_work_cap;
            SqueezeStats sst;
            std::vector<std::vector<std::int32_t>> sq_routes = extract_routes(ss);
            const bool adopted =
                squeeze_phase(inst, nb, sq_routes, sqp, deadline, &sst);
            result.fd_stats.squeeze_phases += sst.phases;
            result.fd_stats.squeeze_evaluated += sst.evaluated;
            result.fd_stats.squeeze_checkpoints += sst.checkpoints;
            if (!adopted) return false;
            std::vector<std::vector<std::int32_t>> pre = extract_routes(ss);
            if (!init_search_state(inst, sq_routes, ss)) {
                // Restore the pre-squeeze state if its unwarped binary64 rebuild fails.
                const bool ok = init_search_state(inst, pre, ss);
                (void)ok;
                mark_all_touched(ss);
                return false;
            }
            ++result.fd_stats.squeeze_improved;
            return true;
        };

        if (!dropped) {
            double nodrop_cand = std::numeric_limits<double>::quiet_NaN();
            if (fd_armed && params.sq_work_cap > 0 && params.sq_on_nodrop &&
                !past() && try_squeeze()) {
                mark_all_touched(ss);
                nodrop_cand = ls_descend(inst, nb, ss, &ls_stats, deadline);
                if (params.exhaustive_on_best && !past()) {
                    mark_all_touched(ss);
                    nodrop_cand =
                        ls_descend(inst, exhaustive, ss, &ls_stats, deadline);
                }
                if (nodrop_cand < best) {
                    consider(nodrop_cand, iteration, 2, extract_routes(ss), true);
                }
            }
            result.fd_stats.basin_evaluated += ls_stats.evaluated - basin_mark;
            fd_work_mark = ls_stats.evaluated;
            return nodrop_cand;
        }
        mark_all_touched(ss);
        double cand = ls_descend(inst, nb, ss, &ls_stats, deadline);
        if (params.exhaustive_on_best && !past()) {
            mark_all_touched(ss);
            cand = ls_descend(inst, exhaustive, ss, &ls_stats, deadline);
        }
        if (params.sq_work_cap > 0 && !past() && try_squeeze()) {
            from_squeeze = true;
            mark_all_touched(ss);
            cand = ls_descend(inst, nb, ss, &ls_stats, deadline);
            if (params.exhaustive_on_best && !past()) {
                mark_all_touched(ss);
                cand = ls_descend(inst, exhaustive, ss, &ls_stats, deadline);
            }
        }
        if (cand < best) {
            consider(cand, iteration, 2, extract_routes(ss), from_squeeze);
        }
        // Track the basin descents separately from ejection pricing so total fleet-descent work can be recovered.
        result.fd_stats.basin_evaluated += ls_stats.evaluated - basin_mark;
        // Reset AFTER the basin descents so the next fd_period_work window
        // measures ILS work between triggers.
        fd_work_mark = ls_stats.evaluated;
        return cand;
    };

    fd_work_mark = ls_stats.evaluated;
    improve_work_mark = ls_stats.evaluated;
    // Drain an above-cap seed before the first iteration. A sweep without progress returns control to ILS with the cap still enforced.
    if (fd_armed && cap_active) {
        while (static_cast<std::int32_t>(ss.states.size()) > params.k_cap &&
               !past()) {
            const std::size_t k_pre = ss.states.size();
            ++result.k_cap_stats.seed_drain_attempts;
            const double fd_cand = run_fleet_descent();
            if (!std::isnan(fd_cand)) curr = fd_cand;
            if (ss.states.size() >= k_pre) break;
        }
        improve_work_mark = ls_stats.evaluated;
    }
    for (; iteration < params.ils_max_iterations; ++iteration) {
        if (past()) {
            status = SolveStatus::TimeLimit;
            break;
        }
        const bool work_restart =
            params.restart_no_improvement_work > 0 &&
            ls_stats.evaluated - improve_work_mark >=
                params.restart_no_improvement_work;
        if (!result.routes.empty() && (no_improvement == params.restart_no_improvement || work_restart)) {
            // Restart-to-best: fresh state, cleared history.
            if (!init_search_state(inst, result.routes, ss)) break;
            ++result.restarts;
            curr = best;
            // Fleet-descent on the incumbent before the fresh window opens;
            // curr keeps pricing whatever state ss holds.
            if (fd_armed && !past()) {
                const double fd_cand = run_fleet_descent();
                if (!std::isnan(fd_cand)) curr = fd_cand;
            }
            std::fill(history.begin(), history.end(),
                      std::numeric_limits<double>::quiet_NaN());
            history_idx = 0;
            no_improvement = 0;
            improve_work_mark = ls_stats.evaluated;
        }
        if (fd_armed && params.fd_period_work > 0 &&
            ls_stats.evaluated - fd_work_mark >= params.fd_period_work &&
            !past()) {
            const double fd_cand = run_fleet_descent();
            if (!std::isnan(fd_cand)) curr = fd_cand;
        }

        take_snapshot(ss, snapshot);
        // Record kick outcomes without consuming RNG draws or changing search prices.
        FleetKickStats& fks = result.fleet_stats;
        const std::size_t k_before = ss.states.size();
        const PerturbOutcome outcome = perturb(inst, nb, ss, rng, perturb_params);
        ++fks.kicks_total;
        fks.redraws_sum += outcome.redraws;
        if (outcome.applied) {
            ++fks.kicks_applied;
            if (outcome.dissolved) {
                ++fks.dissolved_armed;
                if (outcome.new_routes > 0) ++fks.dissolve_undone_in_kick;
                const std::size_t k_kick = ss.states.size();
                if (k_kick < k_before) ++fks.k_after_kick_lt;
                else if (k_kick == k_before) ++fks.k_after_kick_eq;
                else ++fks.k_after_kick_gt;
            } else {
                ++fks.normal_kicks;
            }
        }
        // Ungated K movement across the kick (see KStats: the counters above
        // are all inside the F-gated dissolve branch, so under Duration they
        // say nothing).
        KStats& ks = result.k_stats;
        if (outcome.applied) {
            ks.singleton_opens += outcome.new_routes;
            if (outcome.new_routes > 0) ++ks.kicks_opening;
            const std::size_t k_kick = ss.states.size();
            if (k_kick > k_before) ++ks.k_up_after_kick;
            else if (k_kick < k_before) ++ks.k_down_after_kick;
        }
        double cand = ls_descend(inst, nb, ss, &ls_stats, deadline);
        if (outcome.applied && outcome.dissolved) {
            const std::size_t k_desc = ss.states.size();
            if (k_desc < k_before) ++fks.k_after_descent_lt;
            else if (k_desc == k_before) ++fks.k_after_descent_eq;
            else ++fks.k_after_descent_gt;
        }
        const std::size_t k_after = ss.states.size();
        if (outcome.applied) {
            if (k_after > k_before) ++ks.k_up_after_descent;
            else if (k_after < k_before) ++ks.k_down_after_descent;
        }

        // Above the cap, allow the fleet to shrink or hold. After reaching the cap, reject moves back above it through the usual restoration path.
        const bool cap_reject =
            cap_active &&
            k_after > std::max<std::size_t>(
                          static_cast<std::size_t>(params.k_cap), k_before);
        if (cap_reject) ++result.k_cap_stats.rejected_above_cap;

        ++no_improvement;
        if (!cap_reject && cand < best) {
            const auto candidate_work = ls_stats.evaluated;
            if (params.exhaustive_on_best && !past()) {
                mark_all_touched(ss);
                cand = ls_descend(inst, exhaustive, ss, &ls_stats, deadline);
            }
            const auto previous_k = result.routes.size();
            if (consider(cand, iteration + 1, 2, extract_routes(ss))) {
                no_improvement = 0;
                improve_work_mark = candidate_work;
                if (outcome.applied) {
                    if (outcome.dissolved) {
                        ++fks.dissolved_new_best;
                        if (ss.states.size() < previous_k)
                            ++fks.dissolved_new_best_k_lt;
                    } else {
                        ++fks.normal_new_best;
                    }
                }
                if (k_after > k_before) ++ks.new_best_k_up;
                else if (k_after < k_before) ++ks.new_best_k_down;
            }
        }

        // LAHC acceptance (enhancement 1: accept on improving the current).
        const std::size_t slot = static_cast<std::size_t>(
            history_idx % static_cast<std::uint64_t>(history_len));
        const bool slot_empty = std::isnan(history[slot]);
        const double late = slot_empty ? init_cost : history[slot];
        if (!cap_reject && (cand < late || cand < curr)) {
            curr = cand;
            if (outcome.applied) {
                if (outcome.dissolved) ++fks.dissolved_accepted_lahc;
                else ++fks.normal_accepted_lahc;
                if (k_after > k_before) ++ks.accepted_k_up;
                else if (k_after < k_before) ++ks.accepted_k_down;
            }
        } else {
            restore_snapshot(inst, ss, snapshot);
        }
        // History update (enhancement 2: rewrite only on improvement).
        if (curr < late || slot_empty) history[slot] = curr;
        ++history_idx;
    }

    result.iterations_run = iteration;
    result.work_units = ls_stats.evaluated;
    // Return Infeasible with no published routes if the run never reaches the cap.
    if (cap_active &&
        static_cast<std::int32_t>(result.routes.size()) > params.k_cap) {
        result.routes.clear();
        result.value = kInfeasible;
    }
    result.k_stats.k_final = static_cast<std::int64_t>(result.routes.size());
    result.status = result.routes.empty() ? SolveStatus::Infeasible : status;
    return result;
}

}  // namespace kayros
