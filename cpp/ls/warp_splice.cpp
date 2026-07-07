#include <stdexcept>
#include <utility>

#include "ls/warp_ls.h"

namespace kayros {

namespace {

Pwlf zero_like(const Pwlf& rho) {
    if (rho.xs.empty()) return {};
    const double lo = rho.xs.front(), hi = rho.xs.back();
    if (lo == hi) return {{lo}, {0.0}};
    return {{lo, hi}, {0.0, 0.0}};
}

WarpSegment departure_segment(const Instance& inst) {
    WarpSegment out;
    out.rho = departure_identity(inst);
    out.omega = zero_like(out.rho);
    return out;
}

WarpRouteEval finish(const WarpSegment& acc, double penalty) {
    WarpRouteEval out;
    if (acc.empty() || acc.omega.xs.empty()) return out;
    out.total = true;
    out.min_warp = acc.omega.ys.front();
    MinShift zero;
    if (min_zero_warp_duration(view(acc.rho), view(acc.omega), &zero)) {
        out.feasible = true;
        out.duration = zero.value;
        out.departure = zero.argmin_x;
    }
    const MinShift pen = min_penalised(view(acc.rho), view(acc.omega), penalty);
    out.penalised = pen.value;
    out.penalised_departure = pen.argmin_x;
    return out;
}

}  // namespace

bool build_warp_route_state(const Instance& inst,
                            std::vector<std::int32_t> vertices, double penalty,
                            double t_end, WarpRouteState& state) {
    const std::int64_t m = static_cast<std::int64_t>(vertices.size());
    // Accounting values ALWAYS from the sequential augmented fold.
    const WarpRouteEval eval =
        evaluate_route_warp(inst, vertices.data(), m, penalty, t_end);
    if (!eval.total) return false;
    state.vertices = std::move(vertices);
    state.tree.build(warp_route_leaves(inst, state.vertices.data(), m, t_end));
    state.duration = eval.feasible ? eval.duration : eval.penalised;
    state.min_warp = eval.min_warp;
    state.load = 0;
    for (const std::int32_t v : state.vertices) state.load += inst.demands[v];
    return true;
}

WarpRouteEval evaluate_splice_warp(const Instance& inst, const WarpRouteState& r1,
                                   std::int64_t i1, std::int64_t j1,
                                   const WarpRouteState& r2, std::int64_t i2,
                                   std::int64_t j2, double penalty,
                                   double t_end) {
    const std::vector<std::int32_t>& route1 = r1.vertices;
    const std::vector<std::int32_t>& route2 = r2.vertices;
    const std::int64_t m1 = static_cast<std::int64_t>(route1.size());
    const std::int64_t m2 = static_cast<std::int64_t>(route2.size());
    const bool incoming = i2 <= j2;
    if (i1 < 0 || i1 > m1 || j1 < i1 - 1 || j1 >= m1) {
        throw std::invalid_argument("invalid [i1, j1]");
    }
    if (incoming && (i2 < 0 || j2 >= m2)) {
        throw std::invalid_argument("invalid [i2, j2]");
    }
    const bool head = i1 > 0;
    const bool tail = j1 + 1 < m1;
    if (!head && !incoming && !tail) return {};

    WarpSegment acc = head ? r1.tree.query(0, i1 - 1) : departure_segment(inst);
    if (acc.empty()) return {};

    std::int32_t last = head ? route1[static_cast<std::size_t>(i1 - 1)] : 0;

    if (incoming) {
        acc = warp_concat(
            warp_bridge_leaf(inst, last, route2[static_cast<std::size_t>(i2)], t_end),
            acc);
        if (acc.empty()) return {};
        if (j2 > i2) {
            acc = warp_concat(r2.tree.query(i2 + 1, j2), acc);
            if (acc.empty()) return {};
        }
        last = route2[static_cast<std::size_t>(j2)];
    }

    if (tail) {
        acc = warp_concat(
            warp_bridge_leaf(inst, last, route1[static_cast<std::size_t>(j1 + 1)], t_end),
            acc);
        if (acc.empty()) return {};
        if (j1 + 2 <= m1) {
            acc = warp_concat(r1.tree.query(j1 + 2, m1), acc);
        }
    } else {
        acc = warp_concat(warp_return_leaf(inst, last, t_end), acc);
    }
    return finish(acc, penalty);
}

}  // namespace kayros
