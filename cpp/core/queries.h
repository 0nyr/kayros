#pragma once

#include <cmath>
#include <limits>

#include "core/instance.h"

namespace kayros {

// Point queries used by construction heuristics. They mirror the checker's
// composition semantics point-wise (theta_j ∘ alpha_ij at a single departure),
// with plain max/+ arithmetic: guidance-exact, while final pricing always goes
// through the compose-based evaluate_route (bit-identical to the checker).
inline constexpr double kInfeasible = std::numeric_limits<double>::infinity();

// Earliest feasible depot departure time (the checker's departure_low).
inline double departure_low(const Instance& inst) {
    double lo = inst.horizon_start;
    if (inst.has_time_windows && inst.tw_earliest[0] > lo) lo = inst.tw_earliest[0];
    return lo;
}

// Ready time at j (service completed) when leaving i at ready time t;
// +inf when the move is time-window or horizon infeasible.
inline double ready_next(const Instance& inst, std::int32_t i, std::int32_t j,
                         double t) {
    const PwlfView alpha = inst.arc(i, j);
    if (alpha.n == 0 || t < alpha.xs[0] || t > alpha.xs[alpha.n - 1]) {
        return kInfeasible;
    }
    const double arrival = evaluate(alpha, t);
    if (inst.has_time_windows) {
        if (arrival > inst.tw_latest[j]) return kInfeasible;
        const double start = arrival > inst.tw_earliest[j] ? arrival : inst.tw_earliest[j];
        return start + inst.service_times[j];
    }
    return arrival + inst.service_times[j];
}

// Can the vehicle return to the depot when leaving j at ready time t?
inline bool depot_return_feasible(const Instance& inst, std::int32_t j, double t) {
    const PwlfView alpha = inst.arc(j, 0);
    if (alpha.n == 0 || t < alpha.xs[0] || t > alpha.xs[alpha.n - 1]) return false;
    if (!inst.has_time_windows) return true;
    return evaluate(alpha, t) <= inst.tw_latest[0];
}

}  // namespace kayros
