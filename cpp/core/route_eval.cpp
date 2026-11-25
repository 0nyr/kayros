#include <algorithm>

#include "core/instance.h"

namespace kayros {

PwlfView Instance::departure_window(double (&points)[2]) const {
    points[0] = has_time_windows ? std::max(horizon_start, tw_earliest[0]) : horizon_start;
    points[1] = has_time_windows ? std::min(horizon_end, tw_latest[0]) : horizon_end;
    if (points[0] > points[1]) return {};
    return {points, points, points[0] == points[1] ? 1 : 2};
}

void Instance::prepare_leaves() {
    if (!has_time_windows) return;
    const std::int32_t nv = num_vertices();
    std::vector<Pwlf> transforms;
    transforms.reserve(nv);
    transforms.push_back(identity(0.0, tw_latest[0]));
    for (std::int32_t j = 1; j < nv; ++j) {
        transforms.push_back(make_theta(tw_earliest[j], tw_latest[j], service_times[j]));
    }
    leaf_data.clear();
    leaf_offset.resize(atf_offset.size());
    Pwlf leaf;
    std::size_t entry = 0;
    for (std::int32_t i = 0; i < nv; ++i) {
        for (std::int32_t j = 0; j < nv; ++j) {
            leaf_offset[entry++] = leaf_data.size();
            compose_into(view(transforms[j]), arc(i, j), leaf);
            leaf_data.insert(leaf_data.end(), leaf.xs.begin(), leaf.xs.end());
            leaf_data.insert(leaf_data.end(), leaf.ys.begin(), leaf.ys.end());
        }
    }
    leaf_offset[entry] = leaf_data.size();
    leaf_data.shrink_to_fit();
}

void Instance::apply_service(std::int32_t vertex, PwlfView arrivals, Pwlf& out) const {
    if (has_time_windows) {
        const Pwlf theta = make_theta(tw_earliest[vertex], tw_latest[vertex], service_times[vertex]);
        compose_into(view(theta), arrivals, out);
    } else if (arrivals.n != 0) {
        const double upper = arrivals.ys[arrivals.n - 1];
        const double shift = service_times[vertex];
        const double xs[] = {0.0, upper};
        const double ys[] = {shift, upper + shift};
        compose_into({xs, ys, 2}, arrivals, out);
    } else {
        out.xs.clear();
        out.ys.clear();
    }
}

PwlfView Instance::packed_leaf(std::int32_t from, std::int32_t to) const {
    const std::size_t a = static_cast<std::size_t>(from) * num_vertices() + to;
    const std::size_t offset = leaf_offset[a];
    const auto n = static_cast<std::int64_t>((leaf_offset[a + 1] - offset) / 2);
    if (n == 0) return {};
    const double* xs = leaf_data.data() + offset;
    return {xs, xs + n, n};
}

PwlfView Instance::move_leaf(std::int32_t from, std::int32_t to, Pwlf& scratch) const {
    if (!leaf_offset.empty()) return packed_leaf(from, to);
    const PwlfView alpha = arc(from, to);
    if (to == 0) {
        if (!has_time_windows) return alpha;
        const Pwlf clamp = identity(0.0, tw_latest[0]);
        compose_into(view(clamp), alpha, scratch);
        return view(scratch);
    }
    apply_service(to, alpha, scratch);
    return view(scratch);
}

Pwlf route_ready_time_function(const Instance& inst, const std::int32_t* route,
                               std::int64_t len) {
    double points[2];
    Pwlf acc = copy_pwlf(inst.departure_window(points));
    Pwlf next;
    std::int32_t prev = 0;
    for (std::int64_t k = 0; k < len; ++k) {
        const std::int32_t v = route[k];
        if (!inst.leaf_offset.empty()) {
            // The cache already composes the arc with the window and service at its head.
            const PwlfView leaf = inst.packed_leaf(prev, v);
            compose_into(leaf, view(acc), next);
            std::swap(acc, next);
        } else {
            // The exact-component bridge supplies uncached raw arcs. Preserve its sequential composition order, including for windowed customers.
            compose_into(inst.arc(prev, v), view(acc), next);
            inst.apply_service(v, view(next), acc);
        }
        if (acc.xs.empty()) return acc;
        prev = v;
    }
    if (!inst.leaf_offset.empty()) {
        compose_into(inst.packed_leaf(prev, 0), view(acc), next);
        return next;
    }
    compose_into(inst.arc(prev, 0), view(acc), next);
    if (!inst.has_time_windows) return next;
    const Pwlf clamp = identity(0.0, inst.tw_latest[0]);
    compose_into(view(clamp), view(next), acc);
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
