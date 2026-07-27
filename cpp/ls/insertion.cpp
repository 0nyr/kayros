#include "ls/insertion.h"

#include <algorithm>

namespace kayros {

std::vector<InsertionCandidate> insertion_candidates(
    const Instance& inst, const std::vector<RouteState>& states,
    std::int32_t c, const Pwlf& dep) {
    std::vector<InsertionCandidate> out;
    for (std::size_t b = 0; b < states.size(); ++b) {
        const RouteState& recv = states[b];
        if (recv.load + inst.demands[c] > inst.vehicle_capacity) continue;
        const std::int64_t mb = static_cast<std::int64_t>(recv.vertices.size());
        Pwlf prefix = dep;
        for (std::int64_t p = 0; p <= mb; ++p) {
            if (p > 0) {
                prefix = compose(view(recv.tree.leaf(p - 1)), view(prefix));
                if (prefix.xs.empty()) break;
            }
            const std::int32_t before =
                p > 0 ? recv.vertices[static_cast<std::size_t>(p - 1)] : 0;
            const Pwlf in_bridge = bridge_leaf(inst, before, c);
            if (in_bridge.xs.empty()) continue;
            Pwlf acc = compose(view(in_bridge), view(prefix));
            if (acc.xs.empty()) continue;
            if (p < mb) {
                const Pwlf out_bridge = bridge_leaf(
                    inst, c, recv.vertices[static_cast<std::size_t>(p)]);
                if (out_bridge.xs.empty()) continue;
                acc = compose(view(out_bridge), view(acc));
                if (acc.xs.empty()) continue;
                const Pwlf rest = recv.tree.query(p + 1, mb);
                if (rest.xs.empty()) continue;
                acc = compose(view(rest), view(acc));
            } else {
                const Pwlf ret = return_leaf(inst, c);
                if (ret.xs.empty()) continue;
                acc = compose(view(ret), view(acc));
            }
            if (acc.xs.empty()) continue;
            const MinShift s = min_shifted_image(view(acc));
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
