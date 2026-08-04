#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "core/instance.h"
#include "ls/fleet_descent.h"

namespace kayros {

// MMAS-style TD-ACO parameters. Defaults are the tuned values from the
// original TDVRPTW-solver experiments (bp_heur.json); a re-tuning sweep on
// MAMUT-format instances is scheduled (milestone M3.5).
struct AcoParams {
    std::uint64_t max_iterations = 3000;
    std::uint64_t max_no_improvement = 20;  // cumulative quiet iterations (see solve_aco)
    std::uint32_t nb_ants = 8;
    std::uint32_t alpha = 15;  // pheromone importance (integer exponent)
    std::uint32_t beta = 10;   // heuristic importance (integer exponent)
    double rho = 0.02;         // evaporation rate
    double tau_min = 1e-6;
    double tau_0 = 2.0;
    double tau_max = 10.0;
    double delta_pheromone_threshold = 1e-4;
    // M3.7 TD-LS: descend on the greedy seed and on each iteration's best ant
    // (LCA-BST ranked moves, checker-fold repriced commits).
    bool use_local_search = true;
    bool ls_all_ants = false;  // apply TD-LS to every feasible ant instead of the iteration-best only
    // M7.0 granular candidate lists (TD-Vidal proximity; see
    // tdvrptw-workspace reports/design/td-ils-design.md). Default-on for every
    // strategy since 0.4.0 (deliberate behavior change vs 0.3.0's exhaustive
    // scans); 0 restores the exhaustive enumeration.
    std::int32_t num_neighbours = 50;
    double weight_wait = 0.2;  // inevitable-wait weight in the proximity
};

// M7.2 TD-ILS parameters (Stream 7, tdvrptw-workspace
// reports/design/td-ils-design.md; loop shape and defaults per PyVRP v0.14's
// IteratedLocalSearch + LAHC, adapted to feasible-only checker-exact search).
struct IlsParams {
    std::uint64_t max_iterations = std::numeric_limits<std::uint64_t>::max();
    // Granular LS (M7.0): shared with the perturbation's neighbourhoods.
    std::int32_t num_neighbours = 50;
    double weight_wait = 0.2;
    // Perturbation magnitude (M7.1).
    std::int32_t min_perturbations = 1;
    std::int32_t max_perturbations = 25;
    // Route-dissolve kick share (M7 FleetCostDuration; see PerturbParams).
    // Inert under Duration: only armed when the instance prices routes.
    std::int32_t dissolve_pct = 50;
    // Late-acceptance hill climbing (Burke & Bykov 2017, both section-4.2
    // enhancements as in PyVRP).
    std::int32_t history_length = 300;
    // Restart-to-best after this many iterations without a global-best
    // improvement. PyVRP's 150k assumes microsecond iterations; kayros ILS
    // iterations are ms-class, so the default is scaled down (M7.4 tunable).
    std::int64_t restart_no_improvement = 20000;
    // Exhaustive-VND polish on every new global best (PyVRP-consistent).
    bool exhaustive_on_best = true;
    // Plan-12 M4 fleet-descent phase (NBRMH-lite ejection ladder), run on the
    // incumbent at every restart-to-best trigger (and every fd_period
    // iterations when fd_period > 0). Armed only when the instance prices
    // routes (fixed_route_cost > 0): under Duration no draw is consumed and
    // the branch is dead code, so 1.1.x streams stay bitwise.
    std::int32_t fd_attempts = 3;        // attempts per trigger; 0 disables
    std::int32_t fd_k_max = 2;           // max ejection-window size
    std::int64_t fd_ep_budget = 2000;    // max pool pops per attempt
    double fd_time_cap_seconds = 10.0;   // wall cap per trigger
    std::int64_t fd_period = 0;          // 0 = stagnation-trigger only
    std::int32_t fd_route_choice = 0;    // 0 random, 1 smallest
    std::int32_t fd_pop_order = 0;       // 0 LIFO, 1 difficult-first
    // Work-based triggers (session 44): thresholds in LS work units
    // (LsStats::evaluated, candidate pricings). Deterministic wall-time
    // proxies that self-scale with instance size, unlike the flat iteration
    // counts above (iteration velocity spans n=10 ~1100/s to n=2000 ~0.2/s
    // on Blauth2024, so any flat count fits one size class only). 0 = off;
    // both may be combined with their iteration-count counterparts
    // (whichever fires first wins). fd_period_work is F-gated like fd_period:
    // under Duration the trigger is dead code at any value, so the 1.3.0
    // default (the campaign-winning ~150 s cadence on one modern core) only
    // changes FleetCostDuration runs.
    std::int64_t fd_period_work = 100'000'000;
    // BREAKING default in 1.3.0 (session 45): the flat restart threshold
    // above effectively never fired at n >= 500 (dead code on realistic
    // budgets, stalled searches never restarted), so the work-based restart
    // is ON by default with a deliberately long window: ~25 minutes of
    // stall at the calibrated ~650k units/s single-core rate, at every
    // instance size. Invisible on sub-half-hour budgets (the 250M variant
    // measurably hurt those), active on the hours-scale runs where the dead
    // flat default was the real problem. Long Duration runs diverge from
    // 1.1.x AT DEFAULTS; set 0 to recover bitwise 1.1.x streams.
    std::int64_t restart_no_improvement_work = 1'000'000'000;
};

struct Incumbent {
    double value = 0.0;            // solution_duration (canonical-order, checker-exact)
    double seconds = 0.0;          // wall time since solve start
    std::uint64_t iteration = 0;   // 0 = greedy seed
    std::int32_t origin = 0;       // 0 = greedy, 1 = aco, 2 = ils
};

enum class SolveStatus : std::int32_t {
    Finished = 0,    // iteration budget exhausted
    Converged = 1,   // pheromone mass stagnated
    TimeLimit = 2,
    Infeasible = 3,  // no feasible solution constructed
};

// Dissolve-kick lifecycle counters (Plan 12 M1; ILS only, ACO leaves them
// zero). Captured in solve_ils with integer increments and route-count reads
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

struct SolveResult {
    std::vector<std::vector<std::int32_t>> routes;  // best solution (customer ids, no depot)
    double value = 0.0;                             // its solution_duration
    std::vector<Incumbent> incumbents;
    SolveStatus status = SolveStatus::Infeasible;
    std::uint64_t iterations_run = 0;
    FleetKickStats fleet_stats;
    FdStats fd_stats;  // Plan-12 M4 fleet-descent phase diagnostics (ILS only)
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
bool greedy_makespan(const Instance& inst,
                     std::vector<std::vector<std::int32_t>>& routes_out);

// The pre-1.4.0 multi-hop-lookahead construction, kept as the reference oracle
// for greedy_makespan: not used by any solver path, exercised by the
// equality tests and available to experiments that want to re-measure the
// lookahead's effect. Same loop, O(n^3) selection.
bool greedy_makespan_lookahead(
    const Instance& inst, std::vector<std::vector<std::int32_t>>& routes_out);

// Objective value of a full solution: canonical route order (sorted by first
// customer), checker-exact per-route pricing, plus the FleetCostDuration term
// fixed_route_cost * K (K = non-empty routes; exact no-op at the default 0 =
// Duration); +inf when any route is time-infeasible or the fleet bound is
// exceeded. Every heuristic value in kayros flows through this fold, so ACO
// and ILS inherit the objective unchanged.
double solution_duration(const Instance& inst,
                         const std::vector<std::vector<std::int32_t>>& routes);

// Anytime hook: called synchronously on every new incumbent (greedy seed
// included) with the incumbent record and the full routes. The value is the
// canonical-order checker-exact Duration (solution_duration).
using IncumbentCallback = std::function<void(
    const Incumbent&, const std::vector<std::vector<std::int32_t>>&)>;

// TD-ACO driver (faithful rewrite of the TDVRPTW-solver heuristic: greedy
// seed, Ant-System deposits with MMAS bounds, pheromone-mass convergence).
// time_limit_seconds <= 0 disables the wall-clock limit.
SolveResult solve_aco(const Instance& inst, const AcoParams& params,
                      std::uint64_t seed, double time_limit_seconds,
                      const IncumbentCallback& on_incumbent = {});

// TD-ILS driver (M7.2): greedy seed -> granular descent (+ exhaustive polish)
// -> {perturb, granular descent, LAHC accept, restart-to-best} until the
// budget ends. Feasible-only; every value is the canonical checker Duration;
// the incumbent stream is monotone (LAHC worse-accepts stay internal). The
// time limit is checked per iteration and threaded into the descent's pass
// boundaries, so the overshoot is bounded by one operator pass.
// A non-empty initial_routes warm-starts the loop from that solution instead
// of the greedy seed (M7.3 "aco+ils" split; must be feasible).
SolveResult solve_ils(const Instance& inst, const IlsParams& params,
                      std::uint64_t seed, double time_limit_seconds,
                      const IncumbentCallback& on_incumbent = {},
                      std::vector<std::vector<std::int32_t>> initial_routes = {});

}  // namespace kayros
