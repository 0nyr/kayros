#include <stdexcept>
#include <utility>

#include "ls/ls.h"

namespace kayros {

std::vector<Pwlf> route_leaves(const Instance& inst, const std::int32_t* route,
                               std::int64_t len) {
    if (len <= 0) throw std::invalid_argument("route must be non-empty");
    std::vector<Pwlf> leaves;
    leaves.reserve(static_cast<std::size_t>(len) + 1);

    double points[2];
    const PwlfView dep = inst.departure_window(points);
    if (dep.n == 0) {
        leaves.emplace_back();
        return leaves;
    }

    // The tree's first leaf restricts departure before applying service and the first customer window.
    const Pwlf first = compose(inst.arc(0, route[0]), dep);
    inst.apply_service(route[0], view(first), leaves.emplace_back());
    Pwlf scratch;
    for (std::int64_t k = 1; k < len; ++k) {
        leaves.push_back(copy_pwlf(inst.move_leaf(route[k - 1], route[k], scratch)));
    }
    leaves.push_back(copy_pwlf(inst.move_leaf(route[len - 1], 0, scratch)));
    return leaves;
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
