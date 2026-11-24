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

    if (inst.fold_arm != FoldArm::F0) {
        // Fold study arms: L_0 in the arm's shape, L_1..L_m from the leaf
        // builders (cached copies under the cache arms).
        if (inst.arm_in_place()) {
            Pwlf head = restrict_domain(inst.arc(0, route[0]), dep_lo, dep_hi);
            if (!head.xs.empty()) {
                head = inst.has_time_windows
                           ? apply_ready_time(view(head), true, inst.tw_earliest[route[0]], true,
                                              inst.tw_latest[route[0]],
                                              inst.service_times[route[0]])
                           : apply_ready_time(view(head), false, 0.0, false, 0.0,
                                              inst.service_times[route[0]]);
            }
            leaves.push_back(std::move(head));
        } else {
            const Pwlf dep = identity(dep_lo, dep_hi);
            Pwlf head = compose(inst.arc(0, route[0]), view(dep));
            if (!head.xs.empty()) {
                const Pwlf theta = theta_for(inst, route[0], head.ys.back());
                head = compose(view(theta), view(head));
            }
            leaves.push_back(std::move(head));
        }
        for (std::int64_t k = 1; k < len; ++k) {
            leaves.push_back(bridge_leaf(inst, route[k - 1], route[k]));
        }
        leaves.push_back(return_leaf(inst, route[len - 1]));
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
    if (inst.arm_leaf_cache()) return cached_leaf(inst, from, to);
    if (inst.arm_in_place()) return leaf_uncached(inst, from, to);
    const PwlfView alpha = inst.arc(from, to);
    if (alpha.n == 0) return {};
    const Pwlf theta = theta_for(inst, to, alpha.ys[alpha.n - 1]);
    return compose(view(theta), alpha);
}

Pwlf return_leaf(const Instance& inst, std::int32_t from) {
    if (inst.arm_leaf_cache()) return cached_return_leaf(inst, from);
    if (inst.arm_in_place()) return return_leaf_uncached(inst, from);
    const PwlfView alpha = inst.arc(from, 0);
    Pwlf tail{std::vector<double>(alpha.xs, alpha.xs + alpha.n),
              std::vector<double>(alpha.ys, alpha.ys + alpha.n)};
    if (inst.has_time_windows && !tail.xs.empty()) {
        const Pwlf clamp = identity(0.0, inst.tw_latest[0]);
        tail = compose(view(clamp), view(tail));
    }
    return tail;
}

PwlfView bridge_leaf_view(const Instance& inst, std::int32_t from, std::int32_t to,
                          Pwlf& scratch) {
    if (inst.arm_leaf_cache()) return view(cached_leaf(inst, from, to));
    if (inst.arm_in_place()) {
        scratch = leaf_uncached(inst, from, to);
        return view(scratch);
    }
    // F0 arithmetic: theta (make_theta points, on the stack) composed with alpha.
    const PwlfView alpha = inst.arc(from, to);
    if (alpha.n == 0) return {};
    const double s = inst.service_times[to];
    double txs[3], tys[3];
    std::int64_t tn = 0;
    if (inst.has_time_windows) {
        const double e = inst.tw_earliest[to], l = inst.tw_latest[to];
        const double xs[3] = {0.0, e, l};
        const double ys[3] = {e + s, e + s, l + s};
        for (int k = 0; k < 3; ++k) {
            if (tn > 0 && xs[k] == txs[tn - 1] && ys[k] == tys[tn - 1]) continue;
            txs[tn] = xs[k];
            tys[tn] = ys[k];
            ++tn;
        }
    } else {
        const double upper = alpha.ys[alpha.n - 1];
        txs[0] = 0.0;
        tys[0] = s;
        txs[1] = upper;
        tys[1] = upper + s;
        tn = 2;
    }
    compose_into({txs, tys, tn}, alpha, scratch);
    return view(scratch);
}

PwlfView return_leaf_view(const Instance& inst, std::int32_t from, Pwlf& scratch) {
    if (inst.arm_leaf_cache()) return view(cached_return_leaf(inst, from));
    if (inst.arm_in_place()) {
        scratch = return_leaf_uncached(inst, from);
        return view(scratch);
    }
    const PwlfView alpha = inst.arc(from, 0);
    if (!inst.has_time_windows || alpha.n == 0) return alpha;
    const double due = inst.tw_latest[0];
    const double ixs[2] = {0.0, due};
    compose_into({ixs, ixs, due == 0.0 ? 1 : 2}, alpha, scratch);
    return view(scratch);
}

PwlfView departure_identity_view(const Instance& inst, double (&pts)[2]) {
    double dep_lo = inst.horizon_start;
    double dep_hi = inst.horizon_end;
    if (inst.has_time_windows) {
        dep_lo = std::max(dep_lo, inst.tw_earliest[0]);
        dep_hi = std::min(dep_hi, inst.tw_latest[0]);
    }
    if (dep_lo > dep_hi) return {};
    pts[0] = dep_lo;
    pts[1] = dep_hi;
    return {pts, pts, dep_lo == dep_hi ? 1 : 2};
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
    state.del_dur.clear();
    state.del_valid = false;
    return true;
}

}  // namespace kayros
