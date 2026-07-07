#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "core/instance.h"

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
    // Late-acceptance hill climbing (Burke & Bykov 2017, both section-4.2
    // enhancements as in PyVRP).
    std::int32_t history_length = 300;
    // Restart-to-best after this many iterations without a global-best
    // improvement. PyVRP's 150k assumes microsecond iterations; kayros ILS
    // iterations are ms-class, so the default is scaled down (M7.4 tunable).
    std::int64_t restart_no_improvement = 20000;
    // Exhaustive-VND polish on every new global best (PyVRP-consistent).
    bool exhaustive_on_best = true;
};

struct Incumbent {
    double value = 0.0;            // Duration (canonical-order, checker-exact)
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

struct SolveResult {
    std::vector<std::vector<std::int32_t>> routes;  // best solution (customer ids, no depot)
    double value = 0.0;                             // its Duration
    std::vector<Incumbent> incumbents;
    SolveStatus status = SolveStatus::Infeasible;
    std::uint64_t iterations_run = 0;
};

// Deterministic greedy nearest-ready-time construction (GMH1 port): routes
// depart at the earliest feasible depot time; selection by earliest multi-hop
// ready time over the remaining customers. Returns false when construction
// gets stuck (a remaining customer cannot start a fresh route).
bool greedy_makespan(const Instance& inst,
                     std::vector<std::vector<std::int32_t>>& routes_out);

// Duration of a full solution: canonical route order (sorted by first
// customer), checker-exact per-route pricing; +inf when any route is
// time-infeasible or the fleet bound is exceeded.
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
SolveResult solve_ils(const Instance& inst, const IlsParams& params,
                      std::uint64_t seed, double time_limit_seconds,
                      const IncumbentCallback& on_incumbent = {});

}  // namespace kayros
