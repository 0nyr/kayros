#pragma once

#include <chrono>
#include <cstdint>
#include <random>
#include <vector>

#include "core/warp_eval.h"
#include "ls/ls.h"
#include "ls/warp_ls.h"

namespace kayros {

// Post-drain squeeze runs first-improvement inter-route relocations and swaps on a copy of the routes. It minimizes duration plus penalty times accumulated warp and retains zero-warp states improving the starting duration. A final repair leg uses a larger penalty and stops on reaching zero warp. The caller rebuilds an improved route set with the binary64 search fold before adoption. The phase uses a deterministic candidate budget and consumes no RNG draws.
struct SqueezeParams {
    double sq_penalty = 10.0;     // explore-leg P (ms of duration per ms of warp)
    std::int64_t sq_work_cap = 0; // pricing budget per phase; 0 disables entirely
};

struct SqueezeStats {
    std::int64_t phases = 0;
    std::int64_t evaluated = 0;    // penalised candidate pricings
    std::int64_t improved = 0;     // phases that returned a better bank
    std::int64_t checkpoints = 0;  // zero-warp improvements banked
};

// Runs one phase on `routes` (must be feasible). Returns true iff a strictly
// better exactly-zero-warp solution was found; `routes` then holds it.
// Charges every penalised pricing against the work budget derived from
// params.sq_work_cap. `deadline` is the GLOBAL time limit only (end-of-run
// truncation, never a decision).
bool squeeze_phase(const Instance& inst, const NeighbourLists& nb,
                   std::vector<std::vector<std::int32_t>>& routes,
                   const SqueezeParams& params,
                   const std::chrono::steady_clock::time_point* deadline,
                   SqueezeStats* stats);

// Drain-assist squeeze follows the insertion step of Nagata and Bräysy. Force-insert the absent customer at a minimum-penalty route end or position next to a granular neighbour. Pick a warp-positive route at random and scan improving relocations and swaps, charging candidate evaluations to work_budget. Repeat until total warp reaches zero or repair fails. Only successful repair replaces the supplied routes.
bool squeeze_insert(const Instance& inst, const NeighbourLists& nb,
                    std::vector<std::vector<std::int32_t>>& routes,
                    std::int32_t c, double penalty, std::int64_t* work_budget,
                    std::mt19937_64& rng, SqueezeStats* stats);

}  // namespace kayros
