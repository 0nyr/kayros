#pragma once

#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

#include "core/instance.h"
#include "core/queries.h"

namespace kayros::detail {

// Time-dependent Dijkstra: earliest ready time at every free customer when
// leaving `source` at ready time t0, traversing free customers only. Label
// setting is valid because canonical ATFs are non-decreasing (FIFO).
// eat[source] is forced to +inf afterwards (the source is never free).
inline void earliest_ready_times(const Instance& inst, std::int32_t source,
                                 double t0,
                                 const std::vector<std::uint8_t>& free_v,
                                 std::vector<double>& eat) {
    const std::int32_t nv = inst.num_vertices();
    eat.assign(static_cast<std::size_t>(nv), kInfeasible);
    std::vector<std::uint8_t> done(static_cast<std::size_t>(nv), 0);
    using Item = std::pair<double, std::int32_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    queue.push({t0, source});
    while (!queue.empty()) {
        const auto [t, v] = queue.top();
        queue.pop();
        if (done[v]) continue;
        done[v] = 1;
        eat[v] = t;
        for (std::int32_t w = 1; w < nv; ++w) {
            if (done[w] || !free_v[w]) continue;
            const double ready = ready_next(inst, v, w, t);
            if (ready == kInfeasible) continue;
            queue.push({ready, w});
        }
    }
    eat[source] = kInfeasible;
}

}  // namespace kayros::detail
