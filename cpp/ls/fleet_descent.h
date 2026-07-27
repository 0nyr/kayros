#pragma once

#include <chrono>
#include <cstdint>
#include <random>
#include <vector>

#include "ls/ls.h"

namespace kayros {

// Plan-12 M4 fleet-descent attempt (NBRMH-lite ejection ladder): dissolve one
// route into an ejection pool and drain it back through (1) feasible
// best-insert, then (2) contiguous-window insertion-with-ejection choosing
// the feasible window minimizing the ejected clients' difficulty counters
// (p-counts, the NBRMH / BonnTour difficult-item idea; a client's count grows
// each time step 1 fails for it). All-or-nothing: any dead end, pop-budget or
// deadline hit restores the pre-attempt solution exactly (deterministic
// rebuild from a vertices snapshot, the perturb undo discipline). No step
// ever opens a route, so success means exactly one route fewer, all routes
// feasible, every client served.
//
// Determinism/inertness contract: draws come from the caller's rng via the
// shared modulo helpers (ls/rng.h) and the caller only invokes this behind
// its fixed_route_cost > 0 gate, so Duration rng streams stay bitwise
// unchanged. The M7.0 staleness stamps are NOT maintained during the attempt;
// on success the caller must mark_all_touched(ss) before descending.
struct FleetDescentParams {
    std::int32_t k_max = 2;         // max ejection-window size (clients)
    std::int64_t ep_budget = 2000;  // max ejection-pool pops per attempt
    std::int32_t route_choice = 0;  // 0 uniform random; 1 smallest (ties uniform)
    std::int32_t pop_order = 0;     // 0 LIFO; 1 difficult-first (max p-count)
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
    std::int64_t rollbacks_time = 0;
};

// One attempt; true iff the route count strictly decreased (by exactly one).
// `pcount` (size num_customers + 1) persists across attempts at the caller.
bool fleet_descent(const Instance& inst, const NeighbourLists& nb,
                   SearchState& ss, std::mt19937_64& rng,
                   const FleetDescentParams& params,
                   std::vector<std::int32_t>& pcount,
                   const std::chrono::steady_clock::time_point* deadline,
                   FdStats* stats);

}  // namespace kayros
