#pragma once

#include <mamut_routing_lib/td_exact.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/instance.h"

namespace kayros {
namespace td = mamut_routing::td_exact;

// Prepare original binary64 arcs on demand. A route owns its legs through the
// entire call, even when adding its next leg clears the cross-proposal cache.
class ExactRoutePricer {
    using PreparedArc = std::shared_ptr<const td::Arc>;
    const kayros::Instance& instance;
    const std::size_t budget;
    std::unordered_map<std::int64_t, PreparedArc> cache;
    std::size_t knots = 0;
    std::size_t preparations = 0;

    PreparedArc arc(std::int32_t from, std::int32_t to) {
        const auto key = static_cast<std::int64_t>(from) * instance.num_vertices() + to;
        if (auto saved = cache.find(key); saved != cache.end()) return saved->second;
        const auto raw = instance.arc(from, to);
        if (raw.n == 0)
            throw std::invalid_argument("missing materialized TD arc (" + std::to_string(from) + ", " + std::to_string(to) + ")");
        const auto count = static_cast<std::size_t>(raw.n);
        auto prepared = std::make_shared<td::Arc>(raw.xs, raw.ys, count);
        ++preparations;
        if (count <= budget) {
            if (knots > budget - count) {
                cache.clear();
                knots = 0;
            }
            cache.emplace(key, prepared);
            knots += count;
        }
        return prepared;
    }

public:
    static constexpr std::size_t default_budget = 65536;

    ExactRoutePricer(const kayros::Instance& instance, std::size_t budget = default_budget)
        : instance(instance), budget(budget) {}

    std::size_t cached_knots() const { return knots; }
    std::size_t prepared_arcs() const { return preparations; }

    std::optional<td::Timing> time(const std::vector<std::int32_t>& route) {
        if (route.empty()) throw std::invalid_argument("cannot price an empty route");
        std::vector<td::Customer> customers;
        std::vector<PreparedArc> storage;
        std::vector<const td::Arc*> legs;
        customers.reserve(route.size());
        storage.reserve(route.size() + 1);
        legs.reserve(route.size() + 1);
        auto add_leg = [&](std::int32_t from, std::int32_t to) {
            storage.push_back(arc(from, to));
            legs.push_back(storage.back().get());
        };
        std::int32_t from = 0;
        for (auto to : route) {
            if (to < 1 || to > instance.num_customers)
                throw std::invalid_argument("customer outside the instance");
            add_leg(from, to);
            td::WindowInput window;
            if (instance.has_time_windows)
                window = std::pair{instance.tw_earliest[to], instance.tw_latest[to]};
            customers.emplace_back(window, instance.service_times[to]);
            from = to;
        }
        add_leg(from, 0);
        double lo = instance.horizon_start, hi = instance.horizon_end;
        std::optional<double> close;
        if (instance.has_time_windows) {
            lo = std::max(lo, instance.tw_earliest[0]);
            close = instance.tw_latest[0];
            hi = std::min(hi, *close);
        }
        return td::route_timing(legs, customers, lo, hi, close);
    }

    // Callers enforce global coverage, fleet size and capacity.
    std::optional<td::Q> cost(const std::vector<std::vector<std::int32_t>>& routes) {
        td::Q total = td::exact(instance.fixed_route_cost) * routes.size();
        for (const auto& route : routes) {
            const auto timing = time(route);
            if (!timing) return std::nullopt;
            total += timing->duration;
        }
        return total;
    }
};
}  // namespace kayros
