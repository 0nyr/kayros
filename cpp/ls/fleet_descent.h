#pragma once

#include <chrono>
#include <cstdint>
#include <random>
#include <vector>

#include "ls/ls.h"

namespace kayros {

// The Nagata-Bräysy-inspired ejection ladder dissolves one route into a pool. For each customer it tries feasible insertion, optional squeeze, then insertion with contiguous ejection. The ejection selects the feasible segment with the lowest sum of customer difficulty counts.
// A successful drain preserves coverage and removes one route. A dead end, exhausted work or pop budget, or deadline restores the original routes. The caller marks all customers touched before the next descent. The caller enables the ladder only with a positive route charge and supplies its RNG.
struct FleetDescentParams {
    std::int32_t fd_k_max = 2;         // max ejection-window size (clients)
    std::int64_t fd_ep_budget = 2000;  // max ejection-pool pops per attempt
    // Try a squeeze after feasible insertion fails and before ejection. Increment the difficulty count after these attempts fail.
    bool sq_ladder = false;
    double sq_penalty = 10.0;
};

struct FdStats {
    std::int64_t triggers = 0;   // trigger events (attempt groups, caller-side)
    std::int64_t attempts = 0;
    std::int64_t successes = 0;
    std::int64_t pops = 0;
    std::int64_t step1_inserts = 0;
    std::int64_t ejections = 0;  // clients ejected back into the pool
    std::int64_t rollbacks_deadend = 0;
    std::int64_t rollbacks_budget = 0;
    // Attempts rolled back when the global run deadline expires.
    std::int64_t rollbacks_time = 0;
    // Work-cap rollback (fd_work_cap exhausted mid-drain), and the drain's
    // work counter: candidate pricings (ranked insertion candidates, splice
    // evaluations, fold-commit rebuilds; one route pricing each). The
    // caller-side basin descents after a successful drop are measured
    // separately (FdStats::basin_evaluated, in LsStats::evaluated units).
    std::int64_t rollbacks_work = 0;
    std::int64_t evaluated = 0;
    std::int64_t basin_evaluated = 0;  // caller-side (ils.cpp), not this file
    // Post-drain squeeze phases, candidate evaluations, zero-warp checkpoints and adoptions.
    std::int64_t squeeze_phases = 0;
    std::int64_t squeeze_evaluated = 0;
    std::int64_t squeeze_checkpoints = 0;
    std::int64_t squeeze_improved = 0;
    // In-ladder squeeze attempts and placements after direct insertion failed.
    std::int64_t ladder_squeezes = 0;
    std::int64_t ladder_rescues = 0;
};

// Returns true when the drain removes one route. The caller retains one difficulty count per customer across attempts. A non-null work budget charges each candidate pricing. Exhaustion or the global deadline restores the original solution.
bool fleet_descent(const Instance& inst, const NeighbourLists& nb,
                   SearchState& ss, std::mt19937_64& rng,
                   const FleetDescentParams& params,
                   std::vector<std::int32_t>& pcount,
                   const std::chrono::steady_clock::time_point* deadline,
                   std::int64_t* work_budget, FdStats* stats);

}  // namespace kayros
