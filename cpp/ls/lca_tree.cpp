#include "ls/lca_tree.h"

#include <stdexcept>
#include <utility>

namespace kayros {

namespace {

Pwlf compose_or_empty(const Pwlf& f, const Pwlf& g) {
    if (f.xs.empty() || g.xs.empty()) return {};
    return compose(view(f), view(g));
}

}  // namespace

void LcaTree::build(std::vector<Pwlf> leaves) {
    if (leaves.empty()) throw std::invalid_argument("no leaves");
    leaves_ = std::move(leaves);
    const std::int64_t num_keys = num_leaves() + 1;  // boundaries 0..m+1
    nodes_.assign(static_cast<std::size_t>(num_keys), Node{});
    build_range(0, num_keys - 1, -1, 0);
    for (std::int64_t h = 0; h < num_keys; ++h) fill_node(h);
}

void LcaTree::build_range(std::int64_t lo, std::int64_t hi, std::int64_t parent,
                          std::int64_t depth) {
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

void LcaTree::fill_node(std::int64_t h) {
    Node& node = nodes_[static_cast<std::size_t>(h)];
    node.stored.assign(static_cast<std::size_t>(node.subtree_hi - node.subtree_lo + 1),
                       Pwlf{});
    // Left chain: G(d, h) for subtree_lo <= d < h, decreasing d.
    Pwlf current;
    for (std::int64_t d = h - 1; d >= node.subtree_lo; --d) {
        current = (d == h - 1) ? leaves_[static_cast<std::size_t>(d)]
                               : compose_or_empty(current, leaves_[static_cast<std::size_t>(d)]);
        node.stored[static_cast<std::size_t>(d - node.subtree_lo)] = current;
    }
    // Right chain: G(h, d) for h < d <= subtree_hi, increasing d.
    for (std::int64_t d = h + 1; d <= node.subtree_hi; ++d) {
        current = (d == h + 1) ? leaves_[static_cast<std::size_t>(d - 1)]
                               : compose_or_empty(leaves_[static_cast<std::size_t>(d - 1)], current);
        node.stored[static_cast<std::size_t>(d - node.subtree_lo)] = current;
    }
}

void LcaTree::refill_node_around(std::int64_t h, std::int64_t leaf) {
    Node& node = nodes_[static_cast<std::size_t>(h)];
    if (h >= leaf + 1) {
        // Left-chain entries G(d, h) with d <= leaf < h straddle the leaf.
        const std::int64_t start = std::min(leaf, h - 1);
        for (std::int64_t d = start; d >= node.subtree_lo; --d) {
            node.stored[static_cast<std::size_t>(d - node.subtree_lo)] =
                (d == h - 1)
                    ? leaves_[static_cast<std::size_t>(d)]
                    : compose_or_empty(
                          node.stored[static_cast<std::size_t>(d + 1 - node.subtree_lo)],
                          leaves_[static_cast<std::size_t>(d)]);
        }
    }
    if (h <= leaf) {
        // Right-chain entries G(h, d) with h <= leaf < d straddle the leaf.
        const std::int64_t start = std::max(leaf + 1, h + 1);
        for (std::int64_t d = start; d <= node.subtree_hi; ++d) {
            node.stored[static_cast<std::size_t>(d - node.subtree_lo)] =
                (d == h + 1)
                    ? leaves_[static_cast<std::size_t>(d - 1)]
                    : compose_or_empty(
                          leaves_[static_cast<std::size_t>(d - 1)],
                          node.stored[static_cast<std::size_t>(d - 1 - node.subtree_lo)]);
        }
    }
}

void LcaTree::update_leaf(std::int64_t leaf, Pwlf fn) {
    if (leaf < 0 || leaf >= num_leaves()) throw std::invalid_argument("bad leaf");
    leaves_[static_cast<std::size_t>(leaf)] = std::move(fn);
    // Affected stored pairs straddle boundaries (leaf, leaf+1); their storing
    // node's subtree contains both, i.e. the ancestors of LCA(leaf, leaf+1).
    std::int64_t a = leaf, b = leaf + 1;
    while (nodes_[static_cast<std::size_t>(a)].depth > nodes_[static_cast<std::size_t>(b)].depth)
        a = nodes_[static_cast<std::size_t>(a)].parent;
    while (nodes_[static_cast<std::size_t>(b)].depth > nodes_[static_cast<std::size_t>(a)].depth)
        b = nodes_[static_cast<std::size_t>(b)].parent;
    while (a != b) {
        a = nodes_[static_cast<std::size_t>(a)].parent;
        b = nodes_[static_cast<std::size_t>(b)].parent;
    }
    for (std::int64_t h = a; h >= 0; h = nodes_[static_cast<std::size_t>(h)].parent) {
        refill_node_around(h, leaf);
    }
}

Pwlf LcaTree::query(std::int64_t lo, std::int64_t hi) const {
    if (lo < 0 || hi >= num_leaves() || lo > hi) {
        throw std::invalid_argument("invalid query range");
    }
    const std::int64_t b1 = lo, b2 = hi + 1;
    // LCA of boundary keys b1 < b2.
    std::int64_t a = b1, b = b2;
    while (nodes_[static_cast<std::size_t>(a)].depth > nodes_[static_cast<std::size_t>(b)].depth)
        a = nodes_[static_cast<std::size_t>(a)].parent;
    while (nodes_[static_cast<std::size_t>(b)].depth > nodes_[static_cast<std::size_t>(a)].depth)
        b = nodes_[static_cast<std::size_t>(b)].parent;
    while (a != b) {
        a = nodes_[static_cast<std::size_t>(a)].parent;
        b = nodes_[static_cast<std::size_t>(b)].parent;
    }
    const std::int64_t h = a;
    if (h == b1) return stored_fn(b1, b2);  // G(b1, b2) stored at ancestor b1
    if (h == b2) return stored_fn(b2, b1);  // G(b1, b2) stored at ancestor b2
    return compose_or_empty(stored_fn(h, b2), stored_fn(h, b1));
}

}  // namespace kayros
