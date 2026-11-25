#include "ls/insertion.h"

#include <algorithm>

namespace kayros {

std::vector<InsertionCandidate> insertion_candidates(
    const Instance& inst, const std::vector<RouteState>& states,
    std::int32_t c, const Pwlf& dep) {
    std::vector<InsertionCandidate> out;
    Pwlf prefix_next;
    for (std::size_t b = 0; b < states.size(); ++b) {
        const RouteState& recv = states[b];
        if (recv.load + inst.demands[c] > inst.vehicle_capacity) continue;
        const std::int64_t mb = static_cast<std::int64_t>(recv.vertices.size());
        Pwlf prefix = dep;
        for (std::int64_t p = 0; p <= mb; ++p) {
            if (p > 0) {
                compose_into(view(recv.tree.leaf(p - 1)), view(prefix), prefix_next);
                std::swap(prefix, prefix_next);
                if (prefix.xs.empty()) break;
            }
            const std::int32_t before =
                p > 0 ? recv.vertices[static_cast<std::size_t>(p - 1)] : 0;
            MinShift s;
            if (!price_insertion(inst, recv, view(prefix), before, c, p, mb, s)) continue;
            out.push_back({s.value - recv.duration, b, p});
        }
    }
    std::sort(out.begin(), out.end(),
              [](const InsertionCandidate& x, const InsertionCandidate& y) {
                  if (x.delta != y.delta) return x.delta < y.delta;
                  if (x.route != y.route) return x.route < y.route;
                  return x.position < y.position;
              });
    return out;
}

}  // namespace kayros
