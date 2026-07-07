#pragma once

#include <cstdint>
#include <vector>

#include "heuristics/heuristics.h"
#include "ls/warp_ls.h"

namespace kayros {

// P8.3-minimal penalised ILS (Stream 8): the M7.2 loop shape over
// warp-augmented route states, with PyVRP-style dynamic time-warp penalties.
//
// Deliberate scope reductions vs the feasible-only solve_ils (documented for
// the head-to-head's honesty): operators = inter-route relocate + swap only
// (no intra-relocate, no 2-opt*); route-level staleness (coarser than M7.0's
// per-operator epochs); no exhaustive-on-best polish. Capacity stays HARD —
// only the time-window wall is dissolved (the stream's question is the TD
// time-warp). Feasibility threshold = exactly-zero fold-accounted warp, never
// a tolerance. Incumbents are feasible solutions only, priced by the zero-warp
// accounting evaluator (checker-class values).
struct WarpIlsParams {
    std::uint64_t max_iterations = std::numeric_limits<std::uint64_t>::max();
    std::int32_t num_neighbours = 50;
    double weight_wait = 0.2;
    std::int32_t min_perturbations = 1;
    std::int32_t max_perturbations = 25;
    std::int32_t history_length = 300;
    std::int64_t restart_no_improvement = 20000;
    // PyVRP PenaltyManager baseline (P8.0 memo §8).
    double penalty_init = 10.0;
    double penalty_increase = 1.5;
    double penalty_decrease = 0.9;
    double target_feasible = 0.65;
    std::int32_t penalty_window = 100;
    double penalty_min = 0.1;
    double penalty_max = 1e5;
};

// Same result contract as solve_ils; incumbent origin code 3 = warp-ils.
SolveResult solve_warp_ils(
    const Instance& inst, const WarpIlsParams& params, std::uint64_t seed,
    double time_limit_seconds,
    std::vector<std::vector<std::int32_t>> initial_routes = {});

// Warp-in-kick ILS (the P8.3 "budgeted infeasibility" variant): the M7.2
// feasible-only loop VERBATIM (granular VND, LAHC on plain Durations,
// restart-to-best, exhaustive-on-best), except the perturbation explores the
// infeasible region: ruin K clients, reinsert at the best PENALISED position
// under p_explore (time warp allowed, capacity hard), then repair the kick
// back to exactly-zero warp by a penalised descent under p_repair (>>
// p_explore). A kick that cannot be repaired falls back to the feasible-only
// M7.1 perturbation, so every LAHC candidate is feasible — the trajectory
// only *passes through* infeasibility inside the kick, paying the warp
// machinery cost exactly there and nowhere else.
struct WarpKickIlsParams {
    IlsParams base;              // the feasible loop's own knobs
    double p_explore = 1.0;      // penalty during the penalised reinsertion
    double p_repair = 1e5;       // penalty during the repair descent
};

// Incumbent origin code 4 = ils-wk.
SolveResult solve_ils_wk(
    const Instance& inst, const WarpKickIlsParams& params, std::uint64_t seed,
    double time_limit_seconds,
    std::vector<std::vector<std::int32_t>> initial_routes = {});

}  // namespace kayros
