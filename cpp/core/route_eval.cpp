#include <algorithm>

#include "core/instance.h"

namespace kayros {

Pwlf route_ready_time_function(const Instance& inst, const std::int32_t* route,
                               std::int64_t len) {
    double dep_lo = inst.horizon_start;
    double dep_hi = inst.horizon_end;
    if (inst.has_time_windows) {
        dep_lo = std::max(dep_lo, inst.tw_earliest[0]);
        dep_hi = std::min(dep_hi, inst.tw_latest[0]);
    }
    if (dep_lo > dep_hi) return {};

    Pwlf acc = identity(dep_lo, dep_hi);
    std::int32_t prev = 0;
    for (std::int64_t k = 0; k < len; ++k) {
        const std::int32_t v = route[k];
        acc = compose(inst.arc(prev, v), view(acc));
        if (acc.xs.empty()) return acc;
        const double service_time = inst.service_times[v];
        Pwlf theta;
        if (inst.has_time_windows) {
            theta = make_theta(inst.tw_earliest[v], inst.tw_latest[v], service_time);
        } else {
            // TDVRP: pure service shift over the reachable arrival range.
            const double upper = acc.ys.back();
            theta = Pwlf{{0.0, upper}, {service_time, upper + service_time}};
        }
        acc = compose(view(theta), view(acc));
        if (acc.xs.empty()) return acc;
        prev = v;
    }

    acc = compose(inst.arc(prev, 0), view(acc));
    if (acc.xs.empty()) return acc;
    if (inst.has_time_windows) {
        // Restrict the return arrival to the depot due date, without any
        // waiting clamp: the route ends upon arrival.
        const Pwlf clamp = identity(0.0, inst.tw_latest[0]);
        acc = compose(view(clamp), view(acc));
    }
    return acc;
}

RouteEval evaluate_route(const Instance& inst, const std::int32_t* route,
                         std::int64_t len) {
    const Pwlf delta = route_ready_time_function(inst, route, len);
    if (delta.xs.empty()) return {false, 0.0, 0.0};
    const MinShift m = min_shifted_image(view(delta));
    return {true, m.value, m.argmin_x};
}

}  // namespace kayros
