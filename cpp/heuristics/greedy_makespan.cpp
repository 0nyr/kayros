#include <algorithm>
#include <utility>

#include "heuristics/construct.h"
#include "heuristics/heuristics.h"

namespace kayros {

bool greedy_makespan(const Instance& inst,
                     std::vector<std::vector<std::int32_t>>& routes_out) {
    const std::int32_t n = inst.num_customers;
    const std::int32_t nv = inst.num_vertices();
    std::vector<std::uint8_t> free_v(static_cast<std::size_t>(nv), 1);
    free_v[0] = 0;
    std::int32_t remaining = n;
    routes_out.clear();
    const double dep_lo = departure_low(inst);
    std::vector<double> eat;

    while (remaining > 0) {
        std::vector<std::int32_t> path;
        std::int32_t current = 0;
        double t = dep_lo;
        std::int64_t load = 0;
        while (true) {
            detail::earliest_ready_times(inst, current, t, free_v, eat);
            // Select the free customer with the earliest multi-hop ready time
            // among those directly reachable (smallest id breaks ties).
            std::int32_t next = -1;
            double next_ready = kInfeasible;
            double best_eat = kInfeasible;
            for (std::int32_t v = 1; v < nv; ++v) {
                if (!free_v[v]) continue;
                const double ready = ready_next(inst, current, v, t);
                if (ready == kInfeasible) continue;
                if (eat[v] < best_eat) {
                    best_eat = eat[v];
                    next = v;
                    next_ready = ready;
                }
            }
            if (next < 0) break;
            if (!depot_return_feasible(inst, next, next_ready) ||
                load + inst.demands[next] > inst.vehicle_capacity) {
                break;  // close the route; the customer stays available
            }
            path.push_back(next);
            t = next_ready;
            load += inst.demands[next];
            free_v[next] = 0;
            --remaining;
            current = next;
        }
        if (path.empty()) return false;  // stuck: a customer cannot start a route
        routes_out.push_back(std::move(path));
    }
    return true;
}

double solution_duration(const Instance& inst,
                         const std::vector<std::vector<std::int32_t>>& routes) {
    if (inst.num_vehicles >= 0 &&
        routes.size() > static_cast<std::size_t>(inst.num_vehicles)) {
        return kInfeasible;
    }
    std::vector<const std::vector<std::int32_t>*> order;
    order.reserve(routes.size());
    for (const auto& route : routes) order.push_back(&route);
    std::sort(order.begin(), order.end(),
              [](const auto* a, const auto* b) { return a->front() < b->front(); });
    double total = 0.0;
    for (const auto* route : order) {
        const RouteEval eval = evaluate_route(
            inst, route->data(), static_cast<std::int64_t>(route->size()));
        if (!eval.feasible) return kInfeasible;
        total += eval.duration;
    }
    return total;
}

}  // namespace kayros
