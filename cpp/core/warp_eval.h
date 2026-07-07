#pragma once

#include "core/instance.h"
#include "pwlf/warp.h"

namespace kayros {

// Warp-augmented route evaluation (Stream 8, P8.0 design memo §3).
//
// The augmented fold carries a PAIR of functions of the depot departure time:
//   rho  = clamped route ready-time delta~_r  (NDCPWLF, total in t except for
//          horizon walls, which stay hard in both folds)
//   warp = accumulated lateness W_r(t) = sum_k w_{v_k}(A_k(t)), non-decreasing
// On the exactly-zero-warp set, rho reduces BITWISE to the checker's delta_r
// (its breakpoint grid is the checker grid) — gates G1/G2. Time windows only:
// on a TDVRP instance the fold is the base fold and warp is identically zero.

// Domain end for the warp-augmented vertex builders: an upper bound on every
// pre-clamp arrival. Max of the horizon end, every ATF image value, and every
// clamped service completion l_j + s_j (incl. the depot due date). O(ATF pool);
// compute once per instance and reuse.
double warp_horizon(const Instance& inst);

struct WarpFunctions {
    Pwlf rho;   // both empty when the route hits a hard (horizon) wall
    Pwlf warp;  // same domain as rho
};

// route points to customer ids in 1..n, depot excluded (route_eval convention).
// dedup=false is the checker-twin reference path (grid-prefix gate); dedup=true
// applies the safe flat/vertical-run dedup after every composition step —
// value-bitwise-identical accounting (gate-tested), a fraction of the
// breakpoints. Search and accounting use dedup=true.
WarpFunctions warp_route_functions(const Instance& inst, const std::int32_t* route,
                                   std::int64_t len, double t_end,
                                   bool dedup = false);

struct WarpRouteEval {
    bool total = false;        // rho non-empty; false = hard horizon wall
    bool feasible = false;     // zero-warp set non-empty == checker-feasible (G2)
    double duration = 0.0;     // iff feasible: checker's Delta*_r bitwise (G1)
    double departure = 0.0;    // iff feasible: earliest zero-warp argmin (NEVER
                               // asserted across associations — value channel)
    double min_warp = 0.0;     // iff total: min achievable warp = W(domain start)
    double penalised = 0.0;    // iff total: Phi* = min(rho(t) - t + penalty*W(t));
                               // SEARCH-ONLY (ranking), never accounting
    double penalised_departure = 0.0;  // iff total: Phi* argmin (search-only)
};

WarpRouteEval evaluate_route_warp(const Instance& inst, const std::int32_t* route,
                                  std::int64_t len, double penalty, double t_end,
                                  bool dedup = false);

}  // namespace kayros
