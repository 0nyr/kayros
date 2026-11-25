#pragma once

#include "core/instance.h"
#include "pwlf/warp.h"

namespace kayros {

// The binary64 augmented fold carries clamped ready time rho(t) and accumulated nonnegative lateness W(t). Horizon restrictions remain hard. On its zero-warp prefix the full knot grid agrees with sequential arc-then-window composition. Without time windows W is identically zero.

// Domain end for the warp-augmented vertex builders: an upper bound on every
// pre-clamp arrival. Max of the horizon end, every ATF image value, and every
// clamped service completion l_j + s_j (incl. the depot due date). O(ATF pool);
// compute once per instance and reuse.
double warp_horizon(const Instance& inst);

struct WarpFunctions {
    Pwlf rho;   // both empty when the route hits a hard (horizon) wall
    Pwlf warp;  // same domain as rho
};

// The route contains customer IDs only. The full sequential fold preserves the reference knot grid. Search removes interior flat and vertical knots while keeping the endpoints needed for duration and warp.
WarpFunctions warp_route_functions(const Instance& inst, const std::int32_t* route,
                                   std::int64_t len, double t_end,
                                   bool dedup = false);

struct WarpRouteEval {
    bool total = false;        // rho non-empty; false = hard horizon wall
    bool feasible = false;     // A departure with zero accumulated binary64 warp exists.
    double duration = 0.0;     // Minimum binary64 duration among zero-warp departures, when feasible.
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
