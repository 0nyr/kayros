#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "core/instance.h"
#include "ls/fleet_descent.h"

namespace kayros {

// Native ILS budgets and neighbourhood, fleet-descent and squeeze settings.
struct IlsParams {
    std::uint64_t ils_max_iterations = std::numeric_limits<std::uint64_t>::max();
    std::int32_t num_neighbours = 50;
    double weight_wait = 0.2;
    std::int32_t min_perturbations = 1;
    std::int32_t max_perturbations = 25;
    std::int32_t dissolve_pct = 50;
    std::int32_t lahc_history = 300;
    std::int64_t restart_no_improvement = 20000;
    bool exhaustive_on_best = true;
    std::int32_t fd_attempts = 3;
    std::int32_t fd_k_max = 2;
    std::int64_t fd_ep_budget = 2000;
    std::int64_t fd_work_cap = 6'500'000;
    std::int64_t sq_work_cap = 0;
    double sq_penalty = 10.0;
    bool sq_on_nodrop = true;
    bool sq_ladder = false;
    std::int32_t k_cap = 0;
    std::int64_t fd_period_work = 100'000'000;
    double seed_k_factor = 2.0;
    std::int64_t restart_no_improvement_work = 1'000'000'000;
};

struct Incumbent {
    double value = 0.0;            // solution_cost (approximate search objective, canonical route order)
    double seconds = 0.0;          // wall time since solve start
    std::uint64_t iteration = 0;   // 0 = seed or initial descent
    std::int32_t origin = 0;       // 0 = greedy, 2 = ils, 3 = supplied warm start
    bool from_squeeze = false;
};

enum class SolveStatus : std::int32_t {
    Finished = 0,    // iteration budget exhausted
    TimeLimit = 2,
    Infeasible = 3,  // no feasible solution constructed
};

// Dissolve-kick counters read route counts without consuming RNG draws. Duration runs do not arm the dissolve operator.
struct FleetKickStats {
    std::int64_t kicks_total = 0;      // perturb calls (one per ILS iteration)
    std::int64_t kicks_applied = 0;    // outcomes with applied == true
    std::int64_t redraws_sum = 0;      // failed ruin attempts, all kicks (arming re-rolls per attempt)
    std::int64_t dissolved_armed = 0;  // applied kicks that seeded a whole route
    std::int64_t dissolve_undone_in_kick = 0;  // dissolved kicks whose repair reopened singleton route(s)
    std::int64_t normal_kicks = 0;     // applied kicks without a dissolve seed
    // Route count right after a dissolved kick vs right before it.
    std::int64_t k_after_kick_lt = 0;
    std::int64_t k_after_kick_eq = 0;
    std::int64_t k_after_kick_gt = 0;
    // Route count after the granular descent vs before the dissolved kick.
    // No LS move opens a route, so per kick K only falls during the descent:
    // lt covers every kick-drop that reached the LAHC judgment, and gt means
    // the kick's own singleton-fallback repair net-opened routes the descent
    // kept (the only K-restoring mechanism; the descent cannot re-split).
    std::int64_t k_after_descent_lt = 0;
    std::int64_t k_after_descent_eq = 0;
    std::int64_t k_after_descent_gt = 0;
    std::int64_t dissolved_accepted_lahc = 0;
    std::int64_t normal_accepted_lahc = 0;
    std::int64_t dissolved_new_best = 0;
    std::int64_t normal_new_best = 0;
    std::int64_t dissolved_new_best_k_lt = 0;  // dissolved new best with fewer routes than the incumbent
};

// Route-count changes, recorded for every objective. The counters read the search state without drawing random numbers or changing prices.
struct KStats {
    std::int64_t k_seed = 0;    // routes in the solution the search started from
    std::int64_t k_final = 0;   // routes in the returned best
    std::int64_t k_best_min = 0;  // extremes over the incumbents published
    std::int64_t k_best_max = 0;
    // Route openings by the repair's last-resort singleton fallback, the only
    // in-loop mechanism that can raise K (see ls/perturb.cpp).
    std::int64_t singleton_opens = 0;   // routes opened, summed over kicks
    std::int64_t kicks_opening = 0;     // kicks whose repair opened at least one
    // K across the kick, and across kick + granular descent, versus before it.
    std::int64_t k_up_after_kick = 0;
    std::int64_t k_down_after_kick = 0;
    std::int64_t k_up_after_descent = 0;
    std::int64_t k_down_after_descent = 0;
    // Outcomes that actually survived: LAHC-accepted, and new global bests.
    std::int64_t accepted_k_up = 0;
    std::int64_t accepted_k_down = 0;
    std::int64_t new_best_k_up = 0;
    std::int64_t new_best_k_down = 0;
};

// Fleet-cap counts and the number of candidate evaluations at the first in-cap incumbent. Disabled caps leave these fields zero.
struct KCapStats {
    std::int64_t seed_drain_attempts = 0;  // back-to-back drain calls before the loop
    std::int64_t reached_cap = 0;          // 1 once any K <= k_cap incumbent published
    std::int64_t rejected_above_cap = 0;   // candidates rejected by the cap guard
    std::int64_t first_capped_work = -1;   // work-unit stamp of the first capped incumbent
};

struct SolveResult {
    std::vector<std::vector<std::int32_t>> routes;  // best solution (customer ids, no depot)
    double value = 0.0;                             // its solution_cost
    std::vector<Incumbent> incumbents;
    SolveStatus status = SolveStatus::Infeasible;
    std::uint64_t iterations_run = 0;
    FleetKickStats fleet_stats;
    FdStats fd_stats;  // Fleet-descent diagnostics.
    KStats k_stats;    // route-count movement, recorded under every objective
    KCapStats k_cap_stats;  // Fleet-cap diagnostics.
    // Work-trigger diagnostics (ILS only): total LS work units spent
    // (calibration source for the *_work thresholds: work_units divided by
    // wall seconds is the machine's work rate) and restart-to-best count.
    std::int64_t work_units = 0;
    std::int64_t restarts = 0;
};

// Deterministic nearest-ready-time construction adapted from GMH1. Start at the earliest depot departure, choose the unserved customer with the earliest direct ready time, and break ties by customer ID. Return false if a remaining customer cannot start a fresh route.
bool greedy_construct(const Instance& inst,
                     std::vector<std::vector<std::int32_t>>& routes_out);

// Multi-hop lookahead reference for the direct greedy construction. Dijkstra selection can break direct-ready-time ties differently and raises construction complexity from quadratic to cubic.
bool greedy_construct_lookahead(
    const Instance& inst, std::vector<std::vector<std::int32_t>>& routes_out);

// Split a feasible Duration seed towards target_k. At each step choose the feasible split with the smallest change in the two affected binary64 route durations. Stop if no feasible split remains. The caller enables this only when routes carry no fixed charge.
bool split_to_k(const Instance& inst,
                std::vector<std::vector<std::int32_t>>& routes,
                std::int32_t target_k);

// Objective value of a full solution: canonical route order (sorted by first
// customer), approximate per-route pricing, plus the FleetCostDuration term
// fixed_route_cost * K (K = non-empty routes; exact no-op at the default 0 =
// Duration); +inf when any route is time-infeasible or the fleet bound is
// exceeded.
double solution_cost(const Instance& inst,
                         const std::vector<std::vector<std::int32_t>>& routes);

// Anytime hook: called synchronously on every new incumbent (greedy seed
// included) with the incumbent record and the full routes. The value is the
// canonical-order search objective (solution_cost).
using IncumbentCallback = std::function<void(
    const Incumbent&, const std::vector<std::vector<std::int32_t>>&)>;

// A supplied gate approves a publishable candidate before it becomes the restart solution.
using IncumbentGate = std::function<bool(
    const std::vector<std::vector<std::int32_t>>&, bool from_squeeze)>;

// Iterated local search starts from a feasible warm seed or the greedy construction. Perturbation, descent and late acceptance use the binary64 search objective. A new best passes the supplied gate before publication or restart. Time checks run between iterations and local-search passes.
SolveResult solve_ils(const Instance& inst, const IlsParams& params,
                      std::uint64_t seed, double time_limit,
                      const IncumbentCallback& on_incumbent = {},
                      std::vector<std::vector<std::int32_t>> initial_routes = {},
                      const IncumbentGate& accept_incumbent = {});

}  // namespace kayros
