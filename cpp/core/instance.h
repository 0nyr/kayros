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
    // FleetCostDuration fixed cost F per used (non-empty) route, in the
    // instance's time unit; 0 = plain Duration (every pre-M7 caller). The
    // solution objective is then fold + F * K with K the route count, priced
    // exactly like the checker: canonical duration fold first, one multiply,
    // one add.
    double fixed_route_cost = 0.0;
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

    // Windowed instances store the ready-time function of each arc. The return arc uses only the depot due date.
    // Entry a holds its xs followed by its ys in leaf_data[leaf_offset[a], leaf_offset[a+1]).
    std::vector<std::size_t> leaf_offset;
    std::vector<double> leaf_data;
    void prepare_leaves();
    PwlfView packed_leaf(std::int32_t from, std::int32_t to) const;

    // The view refers to the instance or to scratch. The caller owns scratch and keeps it until the view is consumed.
    PwlfView move_leaf(std::int32_t from, std::int32_t to, Pwlf& scratch) const;
    void apply_service(std::int32_t vertex, PwlfView arrivals, Pwlf& out) const;
    PwlfView departure_window(double (&points)[2]) const;

    std::int32_t num_vertices() const { return num_customers + 1; }

    PwlfView arc(std::int32_t i, std::int32_t j) const {
        const std::int64_t a =
            static_cast<std::int64_t>(i) * num_vertices() + j;
        const std::int64_t b = atf_offset[a];
        const std::int64_t e = atf_offset[a + 1];
        if (b == e) return {};
        return {atf_xs.data() + b, atf_ys.data() + b, e - b};
    }
};

struct RouteEval {
    bool feasible = false;
    double duration = 0.0;   // Delta*_r = min_t (delta_r(t) - t)
    double departure = 0.0;  // earliest optimal depot departure t*_r
};

// Route ready-time function delta_r over feasible depot departure times.
// The binary64 fold includes window and service at each customer and the depot restrictions at departure and return. Returns the empty function when time-infeasible.
// route points to customer ids in 1..n, depot excluded.
Pwlf route_ready_time_function(const Instance& inst, const std::int32_t* route,
                               std::int64_t len);

// Optimal duration and earliest optimal departure (checker's compute_route_duration).
RouteEval evaluate_route(const Instance& inst, const std::int32_t* route,
                         std::int64_t len);

}  // namespace kayros
