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
    std::uint64_t iteration = 0;   // 0 = greedy seed
    std::int32_t origin = 0;       // 0 = greedy, 2 = ils, 3 = supplied warm start
    bool from_squeeze = false;
};

enum class SolveStatus : std::int32_t {
    Finished = 0,    // iteration budget exhausted
    TimeLimit = 2,
    Infeasible = 3,  // no feasible solution constructed
};

// Dissolve-kick lifecycle counters. Captured in solve_ils with integer increments and route-count reads
// only (no rng draw, no double fed back into the search), so Duration runs
// stay bitwise pre-M1. Under Duration the dissolve is never armed, so every
// dissolved_* counter is structurally zero there.
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

// Route-count movement over a solve, recorded under EVERY objective.
//
// FleetKickStats above answers the same question for FleetCostDuration only:
// every one of its K counters lives inside the dissolve branch, which is
// F-gated, so under Duration they are structurally zero. That made the
// solver's K behavior unobservable exactly where it turned out to matter — a
// Duration run's route count is set by the seed and then drifts down, and
// nothing in the record said so. These counters are never gated.
//
// Same discipline as FleetKickStats: integer increments and route-count reads
// only, no rng draw and no priced value, so streams stay bitwise with or
// without them.
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

// Plan-15 S3 K-cap diagnostics (ILS only; all zero when k_cap is off).
// Integer increments, route-count reads and one work-unit stamp: no rng draw
// and no priced value, so streams stay bitwise with or without them.
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
    FdStats fd_stats;  // Plan-12 M4 fleet-descent phase diagnostics (ILS only)
    KStats k_stats;    // route-count movement, recorded under every objective
    KCapStats k_cap_stats;  // Plan-15 S3 K-cap diagnostics (ILS only)
    // Work-trigger diagnostics (ILS only): total LS work units spent
    // (calibration source for the *_work thresholds: work_units divided by
    // wall seconds is the machine's work rate) and restart-to-best count.
    std::int64_t work_units = 0;
    std::int64_t restarts = 0;
};

// Deterministic greedy nearest-ready-time construction (GMH1 port): routes
// depart at the earliest feasible depot time; selection by earliest direct
// ready time over the remaining free customers, smallest id breaking ties.
// Returns false when construction gets stuck (a remaining customer cannot
// start a fresh route).
//
// Through 1.3.0 the selection ran a full TD Dijkstra over the free customers
// at every placement and picked the earliest MULTI-HOP ready time. That
// lookahead is almost entirely redundant: Dijkstra's first settled node is by
// construction the one with the smallest direct ready time, so the multi-hop
// minimum equals the direct minimum and is attained at the same customer. The
// two rules can only part on ties — a customer reachable in the same total
// time through a detour, which needs a time window binding hard enough for
// the wait to absorb the detour — and there the lookahead is a different tie
// break, not a better choice. Dropping it (1.4.0) turns an O(n^3) construction
// into an O(n^2) one: 0.03 s instead of 36 s at n = 1000, which is what makes
// the anytime stream open in well under a second at every scale. The removed
// rule is kept below as the test oracle.
bool greedy_construct(const Instance& inst,
                     std::vector<std::vector<std::int32_t>>& routes_out);

// The pre-1.4.0 multi-hop-lookahead construction, kept as the reference oracle
// for greedy_construct: not used by any solver path, exercised by the
// equality tests and available to experiments that want to re-measure the
// lookahead's effect. Same loop, O(n^3) selection.
bool greedy_construct_lookahead(
    const Instance& inst, std::vector<std::vector<std::int32_t>>& routes_out);

// Split routes of a feasible solution until it holds target_k of them, each
// step taking the cheapest feasible split under the canonical fold. The
// objective is a sum of per-route durations, so a candidate costs two route
// evaluations rather than a whole-solution refold. Stops early when no route
// admits a feasible split. Returns whether anything was split.
//
// This is the K-diverse seeding of plan 13 I2 (1.5.0). Under Duration the
// search sheds routes freely and effectively never adds one, so the seed's
// route count decides the final one, and on fleet-starved instances that
// costs a great deal; starting above the greedy count and letting the search
// descend is worth up to -27 percent there and is inside noise elsewhere.
// It is NEVER right under FleetCostDuration, where every extra route is
// priced: callers must gate on the objective.
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

// TD-ILS driver (M7.2): greedy seed -> granular descent (+ exhaustive polish)
// -> {perturb, granular descent, LAHC accept, restart-to-best} until the
// budget ends. Feasible-only; every value is the canonical checker Duration;
// the incumbent stream is monotone (LAHC worse-accepts stay internal). The
// time limit is checked per iteration and threaded into the descent's pass
// boundaries, so the overshoot is bounded by one operator pass.
// A non-empty initial_routes warm-starts the loop from that solution instead
// of the greedy seed. The supplied routes must be feasible.
SolveResult solve_ils(const Instance& inst, const IlsParams& params,
                      std::uint64_t seed, double time_limit,
                      const IncumbentCallback& on_incumbent = {},
                      std::vector<std::vector<std::int32_t>> initial_routes = {},
                      const IncumbentGate& accept_incumbent = {});

}  // namespace kayros
