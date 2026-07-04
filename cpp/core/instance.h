#pragma once

#include <cstdint>
#include <vector>

#include "pwlf/pwlf.h"

namespace kayros {

// TD instance in checker conventions: vertices 0..n with 0 = depot, customers
// 1..n; complete arc set carrying arrival-time functions alpha_ij over the
// horizon (arrival_at_j = alpha_ij(departure_from_i), FIFO by monotonicity).
struct Instance {
    std::int32_t num_customers = 0;   // n
    std::int32_t num_vehicles = -1;   // fleet upper bound; -1 = unbounded
    std::int64_t vehicle_capacity = 0;
    double horizon_start = 0.0;
    double horizon_end = 0.0;
    bool has_time_windows = false;    // TDVRPTW when true, TDVRP otherwise
    std::vector<std::int64_t> demands;    // size n+1, demands[0] = 0
    std::vector<double> service_times;    // size n+1
    std::vector<double> tw_earliest;      // size n+1 iff has_time_windows
    std::vector<double> tw_latest;        // size n+1 iff has_time_windows

    // Arc ATF pool: arc (i,j) owns the slice [atf_offset[a], atf_offset[a+1])
    // of atf_xs/atf_ys, with a = i*(n+1)+j. Self-arcs have empty slices.
    std::vector<std::int64_t> atf_offset;  // size (n+1)*(n+1)+1
    std::vector<double> atf_xs;
    std::vector<double> atf_ys;

    std::int32_t num_vertices() const { return num_customers + 1; }

    PwlfView arc(std::int32_t i, std::int32_t j) const {
        const std::int64_t a =
            static_cast<std::int64_t>(i) * num_vertices() + j;
        const std::int64_t b = atf_offset[a];
        const std::int64_t e = atf_offset[a + 1];
        return {atf_xs.data() + b, atf_ys.data() + b, e - b};
    }
};

struct RouteEval {
    bool feasible = false;
    double duration = 0.0;   // Delta*_r = min_t (delta_r(t) - t)
    double departure = 0.0;  // earliest optimal depot departure t*_r
};

// Route ready-time function delta_r over feasible depot departure times.
// Exact port of the checker's compute_route_ready_time_function: per vertex
// acc <- theta_v ∘ alpha_{prev,v} ∘ acc, with the TDVRPTW depot TW restriction
// at departure and on return. Returns the empty function when time-infeasible.
// route points to customer ids in 1..n, depot excluded.
Pwlf route_ready_time_function(const Instance& inst, const std::int32_t* route,
                               std::int64_t len);

// Optimal duration and earliest optimal departure (checker's compute_route_duration).
RouteEval evaluate_route(const Instance& inst, const std::int32_t* route,
                         std::int64_t len);

}  // namespace kayros
