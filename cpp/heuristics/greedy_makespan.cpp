#include <algorithm>
#include <utility>

#include "heuristics/construct.h"
#include "heuristics/heuristics.h"

namespace kayros {

namespace {

// Shared route-building loop of the shipped construction and its historical
// GMH1 reference. `lookahead` picks the selection rule among the free
// customers directly reachable from the current position: with lookahead, the
// one with the earliest multi-hop ready time (a TD Dijkstra over the free
// customers); without it, the one with the earliest direct ready time.
// Everything else — route opening at the earliest depot departure, the
// capacity and depot-return guards, the smallest-id tie break, the stuck
// detection — is identical, so the two differ exactly in the per-placement
// Dijkstra: O(n) ready_next calls per placement instead of O(n^2), hence an
// O(n^2) construction instead of an O(n^3) one.
bool construct(const Instance& inst,
               std::vector<std::vector<std::int32_t>>& routes_out,
               bool lookahead) {
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
            if (lookahead) {
                detail::earliest_ready_times(inst, current, t, free_v, eat);
            }
            // Select the free customer with the smallest selection key among
            // those directly reachable (smallest id breaks ties).
            std::int32_t next = -1;
            double next_ready = kInfeasible;
            double best_key = kInfeasible;
            for (std::int32_t v = 1; v < nv; ++v) {
                if (!free_v[v]) continue;
                const double ready = ready_next(inst, current, v, t);
                if (ready == kInfeasible) continue;
                const double key = lookahead ? eat[v] : ready;
                if (key < best_key) {
                    best_key = key;
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

}  // namespace

bool greedy_makespan(const Instance& inst,
                     std::vector<std::vector<std::int32_t>>& routes_out) {
    return construct(inst, routes_out, /*lookahead=*/false);
}

bool greedy_makespan_lookahead(
    const Instance& inst,
    std::vector<std::vector<std::int32_t>>& routes_out) {
    return construct(inst, routes_out, /*lookahead=*/true);
}

double solution_duration(const Instance& inst,
                         const std::vector<std::vector<std::int32_t>>& routes) {
    if (inst.num_vehicles >= 0 &&
        routes.size() > static_cast<std::size_t>(inst.num_vehicles)) {
        return kInfeasible;
    }
    std::vector<const std::vector<std::int32_t>*> order;
    order.reserve(routes.size());
    for (const auto& route : routes) {
        if (!route.empty()) order.push_back(&route);
    }
    std::sort(order.begin(), order.end(),
              [](const auto* a, const auto* b) { return a->front() < b->front(); });
    double total = 0.0;
    for (const auto* route : order) {
        const RouteEval eval = evaluate_route(
            inst, route->data(), static_cast<std::int64_t>(route->size()));
        if (!eval.feasible) return kInfeasible;
        total += eval.duration;
    }
    // FleetCostDuration term, checker order: the canonical fold first, then a
    // single F * K multiply-add over the used (non-empty) routes. Exact no-op
    // when fixed_route_cost is 0 (Duration).
    return total +
           inst.fixed_route_cost * static_cast<double>(order.size());
}

}  // namespace kayros
