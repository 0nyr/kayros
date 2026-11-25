#pragma once

#include <cstdint>
#include <vector>

#include "pwlf/pwlf.h"

namespace kayros {

// MAMUT TD vertices use 0 for the depot and 1..n for customers. Arc alpha_ij maps a departure at i to its arrival at j and is nondecreasing.
struct Instance {
    std::int32_t num_customers = 0;   // n
    std::int32_t num_vehicles = -1;   // fleet upper bound; -1 = unbounded
    std::int64_t vehicle_capacity = 0;
    // Binary64 charge per nonempty route, in the instance time unit. The heuristic sums route durations in canonical order, then adds F times the route count. Duration uses F = 0.
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

    // Windowed heuristic instances store the ready-time function of each arc. The exact-component bridge leaves the pool empty to retain its sequential fold. The return arc uses only the depot due date.
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
    double departure = 0.0;  // earliest binary64 minimizer on this route curve
};

// Route ready-time function delta_r over feasible depot departure times.
// The binary64 fold includes window and service at each customer and the depot restrictions at departure and return. Returns the empty function when time-infeasible.
// route points to customer ids in 1..n, depot excluded.
Pwlf route_ready_time_function(const Instance& inst, const std::int32_t* route,
                               std::int64_t len);

// Minimum duration and earliest minimizing departure on the binary64 route curve.
RouteEval evaluate_route(const Instance& inst, const std::int32_t* route,
                         std::int64_t len);

}  // namespace kayros
