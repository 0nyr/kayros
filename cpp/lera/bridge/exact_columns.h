// kayros-owned bridge to the vendored Lera-Romero BPC (NOT vendored code).
// Exact evaluation of the columns of the master.
#pragma once

#include <algorithm>
#include <cstdint>
#include <vector>

#include <goc/goc.h>

#include "bcp/exact_cost.h"
#include "core/exact_route_pricer.h"
#include "core/instance.h"

namespace kayros::lera {

using solver::ExactCost;

struct ExactColumn {
    bool feasible = false;
    ExactCost cost;          // exact minimum duration of the route
    double departure = 0.0;  // nearest binary64 value of the exact earliest minimizing departure
};

// Lera path (o=0, customers..., d=n+1) -> exact evaluation of the customers by the library header (route_timing). A route is feasible when its arcs exist, its load fits the capacity and its set of feasible depot departures is nonempty.
inline ExactColumn exact_column(kayros::ExactRoutePricer& pricer, const kayros::Instance& ci, const goc::GraphPath& path) {
    std::vector<std::int32_t> customers;
    std::int64_t load = 0;
    for (goc::Vertex v : path)
        if (v != 0 && v != ci.num_customers + 1) { customers.push_back(v); load += ci.demands[v]; }
    if (customers.empty() || load > ci.vehicle_capacity) return {};
    std::int32_t from = 0;
    for (std::int32_t to : customers) {
        if (ci.arc(from, to).n == 0) return {};
        from = to;
    }
    if (ci.arc(from, 0).n == 0) return {};
    const auto timing = pricer.time(customers);
    if (!timing) return {};
    return {true, timing->duration, solver::nearest_binary64(timing->departure)};
}

// Cost M of the artificial columns, a binary64 value at least the cost of any solution: a solution has at most n routes, and a route departs at or after the start of the horizon and arrives at the latest at the largest arrival of an arc.
inline ExactCost artificial_cost(const kayros::Instance& ci) {
    double latest = ci.horizon_end;
    for (double y : ci.atf_ys) latest = std::max(latest, y);
    const ExactCost bound = (ExactCost(latest) - ExactCost(ci.horizon_start)) * ci.num_customers + 1;
    return ExactCost(solver::binary64_at_least(bound));
}

}  // namespace kayros::lera
