#include <algorithm>

#include "core/instance.h"

namespace kayros {

namespace {

// F0: the fold as it was before the fold study (2 compositions per stop).
Pwlf route_ready_time_function_f0(const Instance& inst, const std::int32_t* route,
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

bool departure_window(const Instance& inst, double& dep_lo, double& dep_hi) {
    dep_lo = inst.horizon_start;
    dep_hi = inst.horizon_end;
    if (inst.has_time_windows) {
        dep_lo = std::max(dep_lo, inst.tw_earliest[0]);
        dep_hi = std::min(dep_hi, inst.tw_latest[0]);
    }
    return dep_lo <= dep_hi;
}

// Vertex transform of the in-place shape (td-fold/2): max(t, e) + s with the
// due-date cut under TW, a pure service shift under TDVRP.
Pwlf apply_vertex(const Instance& inst, std::int32_t v, PwlfView acc) {
    const double service_time = inst.service_times[v];
    if (inst.has_time_windows) {
        return apply_ready_time(acc, true, inst.tw_earliest[v], true,
                                inst.tw_latest[v], service_time);
    }
    return apply_ready_time(acc, false, 0.0, false, 0.0, service_time);
}

// F1: td-fold/2 shape. The first arc is restricted to the departure window,
// every later arc is composed with the accumulator, and the vertex transform
// is applied on the breakpoints (1 composition per stop, no theta).
Pwlf route_ready_time_function_in_place(const Instance& inst,
                                        const std::int32_t* route,
                                        std::int64_t len) {
    double dep_lo, dep_hi;
    if (!departure_window(inst, dep_lo, dep_hi)) return {};
    Pwlf acc;
    std::int32_t prev = 0;
    for (std::int64_t k = 0; k < len; ++k) {
        const std::int32_t v = route[k];
        acc = k == 0 ? restrict_domain(inst.arc(0, v), dep_lo, dep_hi)
                     : compose(inst.arc(prev, v), view(acc));
        if (acc.xs.empty()) return acc;
        acc = apply_vertex(inst, v, view(acc));
        if (acc.xs.empty()) return acc;
        prev = v;
    }
    acc = compose(inst.arc(prev, 0), view(acc));
    if (acc.xs.empty()) return acc;
    if (inst.has_time_windows) {
        acc = apply_ready_time(view(acc), false, 0.0, true, inst.tw_latest[0], 0.0);
    }
    return acc;
}

// F2, F4, F5: 1 composition per stop with the cached leaves.
Pwlf route_ready_time_function_cached(const Instance& inst,
                                      const std::int32_t* route,
                                      std::int64_t len) {
    double dep_lo, dep_hi;
    if (!departure_window(inst, dep_lo, dep_hi)) return {};
    Pwlf acc = identity(dep_lo, dep_hi);
    std::int32_t prev = 0;
    for (std::int64_t k = 0; k < len; ++k) {
        const std::int32_t v = route[k];
        const Pwlf& leaf = cached_leaf(inst, prev, v);
        if (leaf.xs.empty()) return {};
        acc = compose(view(leaf), view(acc));
        if (acc.xs.empty()) return acc;
        prev = v;
    }
    const Pwlf& ret = cached_return_leaf(inst, prev);
    if (ret.xs.empty()) return {};
    return compose(view(ret), view(acc));
}

}  // namespace

Pwlf leaf_uncached(const Instance& inst, std::int32_t from, std::int32_t to) {
    const PwlfView alpha = inst.arc(from, to);
    if (alpha.n == 0) return {};
    if (inst.arm_in_place()) return apply_vertex(inst, to, alpha);
    // F0 arithmetic (the bridge_leaf of the LS): theta composed with alpha,
    // the TDVRP shift covering the whole image of alpha.
    const double service_time = inst.service_times[to];
    Pwlf theta;
    if (inst.has_time_windows) {
        theta = make_theta(inst.tw_earliest[to], inst.tw_latest[to], service_time);
    } else {
        const double upper = alpha.ys[alpha.n - 1];
        theta = Pwlf{{0.0, upper}, {service_time, upper + service_time}};
    }
    return compose(view(theta), alpha);
}

Pwlf return_leaf_uncached(const Instance& inst, std::int32_t from) {
    const PwlfView alpha = inst.arc(from, 0);
    Pwlf tail{std::vector<double>(alpha.xs, alpha.xs + alpha.n),
              std::vector<double>(alpha.ys, alpha.ys + alpha.n)};
    if (inst.has_time_windows && !tail.xs.empty()) {
        if (inst.arm_in_place()) {
            return apply_ready_time(view(tail), false, 0.0, true, inst.tw_latest[0], 0.0);
        }
        const Pwlf clamp = identity(0.0, inst.tw_latest[0]);
        tail = compose(view(clamp), view(tail));
    }
    return tail;
}

const Pwlf& cached_leaf(const Instance& inst, std::int32_t from, std::int32_t to) {
    LeafCache& cache = *inst.leaf_cache;
    const std::size_t a = static_cast<std::size_t>(from) *
                              static_cast<std::size_t>(inst.num_vertices()) +
                          static_cast<std::size_t>(to);
    if (!cache.ready[a]) {
        cache.leaf[a] = leaf_uncached(inst, from, to);
        cache.ready[a] = 1;
    }
    return cache.leaf[a];
}

const Pwlf& cached_return_leaf(const Instance& inst, std::int32_t from) {
    LeafCache& cache = *inst.leaf_cache;
    const std::size_t a = static_cast<std::size_t>(from);
    if (!cache.ret_ready[a]) {
        cache.ret[a] = return_leaf_uncached(inst, from);
        cache.ret_ready[a] = 1;
    }
    return cache.ret[a];
}

Pwlf route_ready_time_function(const Instance& inst, const std::int32_t* route,
                               std::int64_t len) {
    if (inst.arm_leaf_cache()) return route_ready_time_function_cached(inst, route, len);
    if (inst.arm_in_place()) return route_ready_time_function_in_place(inst, route, len);
    return route_ready_time_function_f0(inst, route, len);
}

RouteEval evaluate_route(const Instance& inst, const std::int32_t* route,
                         std::int64_t len) {
    const Pwlf delta = route_ready_time_function(inst, route, len);
    if (delta.xs.empty()) return {false, 0.0, 0.0};
    const MinShift m = min_shifted_image(view(delta));
    return {true, m.value, m.argmin_x};
}

}  // namespace kayros
