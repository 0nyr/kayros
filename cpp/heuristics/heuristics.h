#pragma once

#include <cstdint>
#include <functional>
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
};

struct Incumbent {
    double value = 0.0;            // Duration (canonical-order, checker-exact)
    double seconds = 0.0;          // wall time since solve start
    std::uint64_t iteration = 0;   // 0 = greedy seed
    std::int32_t origin = 0;       // 0 = greedy, 1 = aco
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

}  // namespace kayros
