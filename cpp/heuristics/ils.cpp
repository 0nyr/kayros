#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include "core/queries.h"
#include "heuristics/heuristics.h"
#include "ls/fleet_descent.h"
#include "ls/ls.h"
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
                      std::uint64_t seed, double time_limit_seconds,
                      const IncumbentCallback& on_incumbent,
                      std::vector<std::vector<std::int32_t>> initial_routes) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto elapsed = [&start]() {
        return std::chrono::duration<double>(Clock::now() - start).count();
    };
    Clock::time_point deadline_point{};
    const Clock::time_point* deadline = nullptr;
    if (time_limit_seconds > 0.0) {
        deadline_point = start + std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(
                                         time_limit_seconds));
        deadline = &deadline_point;
    }
    const auto past = [&]() {
        return deadline != nullptr && Clock::now() >= *deadline;
    };

    SolveResult result;

    // Seed (warm-start routes when provided, greedy otherwise) + descent.
    std::vector<std::vector<std::int32_t>> seed_routes =
        std::move(initial_routes);
    if (seed_routes.empty() &&
        (!greedy_makespan(inst, seed_routes) ||
         solution_duration(inst, seed_routes) == kInfeasible)) {
        result.status = SolveStatus::Infeasible;
        return result;
    }
    const NeighbourLists nb =
        build_neighbour_lists(inst, params.num_neighbours, params.weight_wait);
    const NeighbourLists exhaustive;  // k == 0 sentinel

    SearchState ss;
    if (!init_search_state(inst, seed_routes, ss)) {
        result.status = SolveStatus::Infeasible;
        return result;
    }
    // Work-unit accounting (session 44): candidate pricings across every
    // descent this solve runs. Integer increments only, so streams and priced
    // values are byte-identical with or without the counter.
    LsStats ls_stats;
    double curr = ls_descend(inst, nb, ss, &ls_stats, deadline);

    double best = curr;
    result.routes = extract_routes(ss);
    result.value = best;
    result.incumbents.push_back({best, elapsed(), 0, 0});  // origin 0 = seed
    if (on_incumbent) on_incumbent(result.incumbents.back(), result.routes);

    // Exhaustive polish of the initial best (PyVRP's exhaustive_on_best).
    if (params.exhaustive_on_best && !past()) {
        mark_all_touched(ss);
        curr = ls_descend(inst, exhaustive, ss, &ls_stats, deadline);
        if (curr < best) {
            best = curr;
            result.routes = extract_routes(ss);
            result.value = best;
            result.incumbents.push_back({best, elapsed(), 0, 2});
            if (on_incumbent) on_incumbent(result.incumbents.back(), result.routes);
        }
    }

    std::mt19937_64 rng(seed);
    PerturbParams perturb_params;
    perturb_params.min_removals = params.min_perturbations;
    perturb_params.max_removals = params.max_perturbations;
    // M5 n-scaling (opt-in): a fraction of the customer count as the removal
    // ceiling, never below the flat knob. Only reached when the caller sets a
    // non-zero pct, so the default draw span (and rng stream) is unchanged.
    if (params.max_perturbation_pct > 0.0) {
        perturb_params.max_removals = std::max(
            params.max_perturbations,
            static_cast<std::int32_t>(std::ceil(
                params.max_perturbation_pct *
                static_cast<double>(inst.num_customers))));
    }
    perturb_params.dissolve_pct = params.dissolve_pct;

    // LAHC slots (Burke & Bykov 2017 with PyVRP's RingBuffer semantics: the
    // index advances every iteration; a slot is rewritten only when the
    // current solution improves on it). Empty slots fall back to the initial
    // cost.
    const std::size_t history_len =
        params.history_length > 0
            ? static_cast<std::size_t>(params.history_length)
            : 1;
    std::vector<double> history(history_len,
                                std::numeric_limits<double>::quiet_NaN());
    std::uint64_t history_idx = 0;
    const double init_cost = curr;

    IlsSnapshot snapshot;
    SolveStatus status = SolveStatus::Finished;
    std::uint64_t iteration = 0;
    std::int64_t no_improvement = 0;

    // Plan-12 M4 fleet-descent phase. Armed only when routes are priced:
    // under Duration the branch is dead code and no draw is consumed, so
    // 1.1.x rng streams stay bitwise.
    FleetDescentParams fd_params;
    fd_params.k_max = params.fd_k_max;
    fd_params.ep_budget = params.fd_ep_budget;
    fd_params.route_choice = params.fd_route_choice;
    fd_params.pop_order = params.fd_pop_order;
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
        Clock::time_point cap_point =
            Clock::now() + std::chrono::duration_cast<Clock::duration>(
                               std::chrono::duration<double>(
                                   params.fd_time_cap_seconds));
        const Clock::time_point* fd_deadline = &cap_point;
        if (deadline != nullptr && *deadline < cap_point) fd_deadline = deadline;
        bool dropped = false;
        for (std::int32_t a = 0; a < params.fd_attempts; ++a) {
            if (Clock::now() >= *fd_deadline) break;
            if (fleet_descent(inst, nb, ss, rng, fd_params, fd_pcount,
                              fd_deadline, &result.fd_stats)) {
                dropped = true;
                break;
            }
        }
        if (!dropped) {
            fd_work_mark = ls_stats.evaluated;
            return std::numeric_limits<double>::quiet_NaN();
        }
        mark_all_touched(ss);
        double cand = ls_descend(inst, nb, ss, &ls_stats, deadline);
        if (params.exhaustive_on_best && !past()) {
            mark_all_touched(ss);
            cand = ls_descend(inst, exhaustive, ss, &ls_stats, deadline);
        }
        if (cand < best) {
            best = cand;
            result.routes = extract_routes(ss);
            result.value = best;
            result.incumbents.push_back({best, elapsed(), iteration, 2});
            if (on_incumbent) on_incumbent(result.incumbents.back(), result.routes);
        }
        // Reset AFTER the basin descents so the next fd_period_work window
        // measures pure ILS work between triggers (mirrors fd_period, which
        // counts iterations only).
        fd_work_mark = ls_stats.evaluated;
        return cand;
    };

    fd_work_mark = ls_stats.evaluated;
    improve_work_mark = ls_stats.evaluated;
    for (; iteration < params.max_iterations; ++iteration) {
        if (past()) {
            status = SolveStatus::TimeLimit;
            break;
        }
        const bool work_restart =
            params.restart_no_improvement_work > 0 &&
            ls_stats.evaluated - improve_work_mark >=
                params.restart_no_improvement_work;
        if (no_improvement == params.restart_no_improvement || work_restart) {
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
        if (fd_armed && params.fd_period > 0 && iteration > 0 &&
            iteration % static_cast<std::uint64_t>(params.fd_period) == 0 &&
            !past()) {
            const double fd_cand = run_fleet_descent();
            if (!std::isnan(fd_cand)) curr = fd_cand;
        }
        if (fd_armed && params.fd_period_work > 0 &&
            ls_stats.evaluated - fd_work_mark >= params.fd_period_work &&
            !past()) {
            const double fd_cand = run_fleet_descent();
            if (!std::isnan(fd_cand)) curr = fd_cand;
        }

        take_snapshot(ss, snapshot);
        // Kick lifecycle counters (Plan 12 M1): integer increments and
        // states.size() reads only, so the rng stream and every priced value
        // are byte-identical with or without them.
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
        double cand = ls_descend(inst, nb, ss, &ls_stats, deadline);
        if (outcome.applied && outcome.dissolved) {
            const std::size_t k_desc = ss.states.size();
            if (k_desc < k_before) ++fks.k_after_descent_lt;
            else if (k_desc == k_before) ++fks.k_after_descent_eq;
            else ++fks.k_after_descent_gt;
        }

        ++no_improvement;
        if (cand < best) {
            no_improvement = 0;
            improve_work_mark = ls_stats.evaluated;
            if (params.exhaustive_on_best && !past()) {
                mark_all_touched(ss);
                cand = ls_descend(inst, exhaustive, ss, &ls_stats, deadline);
            }
            if (outcome.applied) {
                if (outcome.dissolved) {
                    ++fks.dissolved_new_best;
                    if (ss.states.size() < result.routes.size())
                        ++fks.dissolved_new_best_k_lt;
                } else {
                    ++fks.normal_new_best;
                }
            }
            best = cand;
            result.routes = extract_routes(ss);
            result.value = best;
            result.incumbents.push_back({best, elapsed(), iteration + 1, 2});
            if (on_incumbent) on_incumbent(result.incumbents.back(), result.routes);
        }

        // LAHC acceptance (enhancement 1: accept on improving the current).
        const std::size_t slot = static_cast<std::size_t>(
            history_idx % static_cast<std::uint64_t>(history_len));
        const bool slot_empty = std::isnan(history[slot]);
        const double late = slot_empty ? init_cost : history[slot];
        if (cand < late || cand < curr) {
            curr = cand;
            if (outcome.applied) {
                if (outcome.dissolved) ++fks.dissolved_accepted_lahc;
                else ++fks.normal_accepted_lahc;
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
    result.status = result.routes.empty() ? SolveStatus::Infeasible : status;
    return result;
}

}  // namespace kayros
