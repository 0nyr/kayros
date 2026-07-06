#include <algorithm>
#include <stdexcept>
#include <utility>

#include "ls/ls.h"

namespace kayros {

namespace {

Pwlf theta_for(const Instance& inst, std::int32_t v, double image_upper) {
    const double service_time = inst.service_times[v];
    if (inst.has_time_windows) {
        return make_theta(inst.tw_earliest[v], inst.tw_latest[v], service_time);
    }
    // TDVRP: pure service shift covering the incoming function's image.
    return Pwlf{{0.0, image_upper}, {service_time, image_upper + service_time}};
}

}  // namespace

std::vector<Pwlf> route_leaves(const Instance& inst, const std::int32_t* route,
                               std::int64_t len) {
    if (len <= 0) throw std::invalid_argument("route must be non-empty");
    std::vector<Pwlf> leaves;
    leaves.reserve(static_cast<std::size_t>(len) + 1);

    double dep_lo = inst.horizon_start;
    double dep_hi = inst.horizon_end;
    if (inst.has_time_windows) {
        dep_lo = std::max(dep_lo, inst.tw_earliest[0]);
        dep_hi = std::min(dep_hi, inst.tw_latest[0]);
    }
    if (dep_lo > dep_hi) {
        leaves.push_back({});  // empty leaf: the whole fold is infeasible
        return leaves;
    }

    // L_0: depot window restriction, first arc, first theta.
    {
        const Pwlf dep = identity(dep_lo, dep_hi);
        Pwlf head = compose(inst.arc(0, route[0]), view(dep));
        if (!head.xs.empty()) {
            const Pwlf theta = theta_for(inst, route[0], head.ys.back());
            head = compose(view(theta), view(head));
        }
        leaves.push_back(std::move(head));
    }

    // L_1 .. L_{m-1}: theta_{r[i]} ∘ alpha_{r[i-1],r[i]}.
    for (std::int64_t k = 1; k < len; ++k) {
        const PwlfView alpha = inst.arc(route[k - 1], route[k]);
        if (alpha.n == 0) {
            leaves.push_back({});
            continue;
        }
        const Pwlf theta = theta_for(inst, route[k], alpha.ys[alpha.n - 1]);
        leaves.push_back(compose(view(theta), alpha));
    }

    // L_m: return arc, depot due-date restriction (no waiting clamp).
    {
        const PwlfView alpha = inst.arc(route[len - 1], 0);
        Pwlf tail{std::vector<double>(alpha.xs, alpha.xs + alpha.n),
                  std::vector<double>(alpha.ys, alpha.ys + alpha.n)};
        if (inst.has_time_windows && !tail.xs.empty()) {
            const Pwlf clamp = identity(0.0, inst.tw_latest[0]);
            tail = compose(view(clamp), view(tail));
        }
        leaves.push_back(std::move(tail));
    }
    return leaves;
}

Pwlf bridge_leaf(const Instance& inst, std::int32_t from, std::int32_t to) {
    const PwlfView alpha = inst.arc(from, to);
    if (alpha.n == 0) return {};
    const Pwlf theta = theta_for(inst, to, alpha.ys[alpha.n - 1]);
    return compose(view(theta), alpha);
}

Pwlf return_leaf(const Instance& inst, std::int32_t from) {
    const PwlfView alpha = inst.arc(from, 0);
    Pwlf tail{std::vector<double>(alpha.xs, alpha.xs + alpha.n),
              std::vector<double>(alpha.ys, alpha.ys + alpha.n)};
    if (inst.has_time_windows && !tail.xs.empty()) {
        const Pwlf clamp = identity(0.0, inst.tw_latest[0]);
        tail = compose(view(clamp), view(tail));
    }
    return tail;
}

bool build_route_state(const Instance& inst, std::vector<std::int32_t> vertices,
                       RouteState& state) {
    const std::int64_t m = static_cast<std::int64_t>(vertices.size());
    const RouteEval eval = evaluate_route(inst, vertices.data(), m);
    if (!eval.feasible) return false;
    state.vertices = std::move(vertices);
    state.tree.build(route_leaves(inst, state.vertices.data(), m));
    state.duration = eval.duration;
    state.departure = eval.departure;
    state.load = 0;
    for (const std::int32_t v : state.vertices) state.load += inst.demands[v];
    return true;
}

}  // namespace kayros
