#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include "core/warp_eval.h"
#include "ls/ls.h"
#include "ls/warp_ls.h"

namespace kayros {

// Plan-15 S2: the post-drain squeeze. A penalised improvement phase on a
// FEASIBLE solution: a granular first-improvement descent on
// Phi = (rho(t) - t) + P * W(t) that may pass through transient time-window
// violations, banking every exactly-zero-warp state that improves on the
// entry duration, with a dominating-repair tail leg (the P8.3
// stop_when_feasible exit) so the budget is not stranded warp-positive. The
// move set is the td-time-warp branch's proven penalised descent
// (inter-route relocate + swap, granular justification, route-level
// staleness); see the plan-15 M1 memo for the v0 scope decision.
//
// Deterministic and rng-free. Operates on a routes COPY: the caller's search
// state is untouched unless the phase returns true, so a phase that banks
// nothing is a strict no-op. The returned routes are always exactly-zero
// warp; the caller reprices them through the checker fold (repricing rule).
struct SqueezeParams {
    double penalty = 10.0;     // explore-leg P (ms of duration per ms of warp)
    std::int64_t work_cap = 0; // pricing budget per phase; 0 disables entirely
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
// params.work_cap. `deadline` is the GLOBAL time limit only (end-of-run
// truncation, never a decision).
bool squeeze_phase(const Instance& inst, const NeighbourLists& nb,
                   std::vector<std::vector<std::int32_t>>& routes,
                   const SqueezeParams& params,
                   const std::chrono::steady_clock::time_point* deadline,
                   SqueezeStats* stats);

}  // namespace kayros
