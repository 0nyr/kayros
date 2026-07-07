#include <algorithm>
#include <stdexcept>
#include <utility>

#include "ls/warp_ls.h"

namespace kayros {

namespace {

// The zero function on [lo, hi] (warp_eval.cpp twin, kept file-local there).
Pwlf zero_like(const Pwlf& rho) {
    if (rho.xs.empty()) return {};
    const double lo = rho.xs.front(), hi = rho.xs.back();
    if (lo == hi) return {{lo}, {0.0}};
    return {{lo, hi}, {0.0, 0.0}};
}

WarpSegment concat_or_empty(const WarpSegment& later, const WarpSegment& earlier) {
    return warp_concat(later, earlier);
}

}  // namespace

WarpSegment warp_concat(const WarpSegment& later, const WarpSegment& earlier) {
    if (later.empty() || earlier.empty()) return {};
    WarpSegment out;
    out.rho = compose(view(later.rho), view(earlier.rho));
    if (out.rho.xs.empty()) return {};  // hard wall inside the concatenation
    const Pwlf carried = compose(view(later.omega), view(earlier.rho));
    out.omega = add(view(earlier.omega), view(carried));
    return out;
}

WarpSegment warp_bridge_leaf(const Instance& inst, std::int32_t from,
                             std::int32_t to, double t_end) {
    const PwlfView alpha = inst.arc(from, to);
    if (alpha.n == 0) return {};
    WarpSegment out;
    if (inst.has_time_windows) {
        const ThetaWarp tw = make_theta_warp(inst.tw_earliest[to], inst.tw_latest[to],
                                             inst.service_times[to], t_end);
        out.rho = compose(view(tw.theta), alpha);
        out.omega = compose(view(tw.warp), alpha);
    } else {
        const double upper = alpha.ys[alpha.n - 1];
        const Pwlf theta{{0.0, upper}, {inst.service_times[to], upper + inst.service_times[to]}};
        out.rho = compose(view(theta), alpha);
        out.omega = zero_like(out.rho);
    }
    return out;
}

WarpSegment warp_return_leaf(const Instance& inst, std::int32_t from, double t_end) {
    const PwlfView alpha = inst.arc(from, 0);
    if (alpha.n == 0) return {};
    WarpSegment out;
    if (inst.has_time_windows) {
        const double due = inst.tw_latest[0];
        out.rho = compose(view(make_return_clamp(due, t_end)), alpha);
        out.omega = compose(view(make_return_warp(due, t_end)), alpha);
    } else {
        out.rho = Pwlf{std::vector<double>(alpha.xs, alpha.xs + alpha.n),
                       std::vector<double>(alpha.ys, alpha.ys + alpha.n)};
        out.omega = zero_like(out.rho);
    }
    return out;
}

std::vector<WarpSegment> warp_route_leaves(const Instance& inst,
                                           const std::int32_t* route,
                                           std::int64_t len, double t_end) {
    if (len <= 0) throw std::invalid_argument("route must be non-empty");
    std::vector<WarpSegment> leaves;
    leaves.reserve(static_cast<std::size_t>(len) + 1);

    // L_0: depot window restriction, first arc, first clamped theta.
    {
        const Pwlf dep = departure_identity(inst);
        WarpSegment head;
        if (!dep.xs.empty()) {
            const Pwlf arrival = compose(inst.arc(0, route[0]), view(dep));
            if (!arrival.xs.empty()) {
                if (inst.has_time_windows) {
                    const std::int32_t v = route[0];
                    const ThetaWarp tw = make_theta_warp(
                        inst.tw_earliest[v], inst.tw_latest[v],
                        inst.service_times[v], t_end);
                    head.rho = compose(view(tw.theta), view(arrival));
                    head.omega = compose(view(tw.warp), view(arrival));
                } else {
                    const double upper = arrival.ys.back();
                    const Pwlf theta{{0.0, upper},
                                     {inst.service_times[route[0]],
                                      upper + inst.service_times[route[0]]}};
                    head.rho = compose(view(theta), view(arrival));
                    head.omega = zero_like(head.rho);
                }
            }
        }
        leaves.push_back(std::move(head));
    }

    for (std::int64_t k = 1; k < len; ++k) {
        leaves.push_back(warp_bridge_leaf(inst, route[k - 1], route[k], t_end));
    }
    leaves.push_back(warp_return_leaf(inst, route[len - 1], t_end));
    return leaves;
}

void WarpLcaTree::build(std::vector<WarpSegment> leaves) {
    if (leaves.empty()) throw std::invalid_argument("no leaves");
    leaves_ = std::move(leaves);
    const std::int64_t num_keys = num_leaves() + 1;
    nodes_.assign(static_cast<std::size_t>(num_keys), Node{});
    build_range(0, num_keys - 1, -1, 0);
    for (std::int64_t h = 0; h < num_keys; ++h) fill_node(h);
}

void WarpLcaTree::build_range(std::int64_t lo, std::int64_t hi,
                              std::int64_t parent, std::int64_t depth) {
    if (lo > hi) return;
    const std::int64_t h = lo + (hi - lo) / 2;
    Node& node = nodes_[static_cast<std::size_t>(h)];
    node.subtree_lo = lo;
    node.subtree_hi = hi;
    node.parent = parent;
    node.depth = depth;
    if (parent < 0) root_ = h;
    build_range(lo, h - 1, h, depth + 1);
    build_range(h + 1, hi, h, depth + 1);
}

void WarpLcaTree::fill_node(std::int64_t h) {
    Node& node = nodes_[static_cast<std::size_t>(h)];
    node.stored.assign(
        static_cast<std::size_t>(node.subtree_hi - node.subtree_lo + 1),
        WarpSegment{});
    WarpSegment current;
    for (std::int64_t d = h - 1; d >= node.subtree_lo; --d) {
        current = (d == h - 1)
                      ? leaves_[static_cast<std::size_t>(d)]
                      : concat_or_empty(current, leaves_[static_cast<std::size_t>(d)]);
        node.stored[static_cast<std::size_t>(d - node.subtree_lo)] = current;
    }
    for (std::int64_t d = h + 1; d <= node.subtree_hi; ++d) {
        current = (d == h + 1)
                      ? leaves_[static_cast<std::size_t>(d - 1)]
                      : concat_or_empty(leaves_[static_cast<std::size_t>(d - 1)], current);
        node.stored[static_cast<std::size_t>(d - node.subtree_lo)] = current;
    }
}

void WarpLcaTree::refill_node_around(std::int64_t h, std::int64_t leaf) {
    Node& node = nodes_[static_cast<std::size_t>(h)];
    if (h >= leaf + 1) {
        const std::int64_t start = std::min(leaf, h - 1);
        for (std::int64_t d = start; d >= node.subtree_lo; --d) {
            node.stored[static_cast<std::size_t>(d - node.subtree_lo)] =
                (d == h - 1)
                    ? leaves_[static_cast<std::size_t>(d)]
                    : concat_or_empty(
                          node.stored[static_cast<std::size_t>(d + 1 - node.subtree_lo)],
                          leaves_[static_cast<std::size_t>(d)]);
        }
    }
    if (h <= leaf) {
        const std::int64_t start = std::max(leaf + 1, h + 1);
        for (std::int64_t d = start; d <= node.subtree_hi; ++d) {
            node.stored[static_cast<std::size_t>(d - node.subtree_lo)] =
                (d == h + 1)
                    ? leaves_[static_cast<std::size_t>(d - 1)]
                    : concat_or_empty(
                          leaves_[static_cast<std::size_t>(d - 1)],
                          node.stored[static_cast<std::size_t>(d - 1 - node.subtree_lo)]);
        }
    }
}

void WarpLcaTree::update_leaf(std::int64_t leaf, WarpSegment fn) {
    if (leaf < 0 || leaf >= num_leaves()) throw std::invalid_argument("bad leaf");
    leaves_[static_cast<std::size_t>(leaf)] = std::move(fn);
    std::int64_t a = leaf, b = leaf + 1;
    while (nodes_[static_cast<std::size_t>(a)].depth >
           nodes_[static_cast<std::size_t>(b)].depth)
        a = nodes_[static_cast<std::size_t>(a)].parent;
    while (nodes_[static_cast<std::size_t>(b)].depth >
           nodes_[static_cast<std::size_t>(a)].depth)
        b = nodes_[static_cast<std::size_t>(b)].parent;
    while (a != b) {
        a = nodes_[static_cast<std::size_t>(a)].parent;
        b = nodes_[static_cast<std::size_t>(b)].parent;
    }
    for (std::int64_t h = a; h >= 0; h = nodes_[static_cast<std::size_t>(h)].parent) {
        refill_node_around(h, leaf);
    }
}

WarpSegment WarpLcaTree::query(std::int64_t lo, std::int64_t hi) const {
    if (lo < 0 || hi >= num_leaves() || lo > hi) {
        throw std::invalid_argument("invalid query range");
    }
    const std::int64_t b1 = lo, b2 = hi + 1;
    std::int64_t a = b1, b = b2;
    while (nodes_[static_cast<std::size_t>(a)].depth >
           nodes_[static_cast<std::size_t>(b)].depth)
        a = nodes_[static_cast<std::size_t>(a)].parent;
    while (nodes_[static_cast<std::size_t>(b)].depth >
           nodes_[static_cast<std::size_t>(a)].depth)
        b = nodes_[static_cast<std::size_t>(b)].parent;
    while (a != b) {
        a = nodes_[static_cast<std::size_t>(a)].parent;
        b = nodes_[static_cast<std::size_t>(b)].parent;
    }
    const std::int64_t h = a;
    if (h == b1) return stored_fn(b1, b2);
    if (h == b2) return stored_fn(b2, b1);
    return concat_or_empty(stored_fn(h, b2), stored_fn(h, b1));
}

}  // namespace kayros
