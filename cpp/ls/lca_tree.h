#pragma once

#include <cstdint>
#include <vector>

#include "pwlf/pwlf.h"

namespace kayros {

// Blauth et al. 2024 (Theorem 12) LCA-BST over the m+1 route leaves, ported
// from the gate-tested td-route-trees study implementation (Stream 4, P4.3;
// decision memo: tdvrptw-workspace reports/design/td-ls-structures-decision.md).
// No normalization anywhere: compositions carry the exact checker semantics.
//
// Boundaries 0..m+1 delimit the leaves (leaf i lies between boundaries i and
// i+1). G(b1, b2) := L_{b2-1} ∘ ... ∘ L_{b1} for b1 < b2, with the splice
// property G(b1, b3) = G(b2, b3) ∘ G(b1, b2). A static balanced BST is built
// over the boundaries; every node h stores G(d, h) for each descendant d < h
// and G(h, d) for each descendant d > h, computed incrementally with one
// compose each (O(m log m) composes total).
//
// query(lo, hi) returns the composition of leaves lo..hi = G(lo, hi+1) via
// the lowest common ancestor h of boundaries lo and hi+1: at most ONE compose
// G(h, hi+1) ∘ G(lo, h) (zero when one boundary is an ancestor of the other).
//
// update_leaf(i, fn) recomputes the O(m) stored functions straddling leaf i
// (Blauth's localized update — no full rebuild; bitwise ≡ rebuild, gate-tested
// in the study repo).
class LcaTree {
  public:
    void build(std::vector<Pwlf> leaves);
    void update_leaf(std::int64_t leaf, Pwlf fn);

    std::int64_t num_leaves() const { return static_cast<std::int64_t>(leaves_.size()); }
    const Pwlf& leaf(std::int64_t i) const { return leaves_[static_cast<std::size_t>(i)]; }

    // Composition of leaves lo..hi inclusive (empty Pwlf propagates).
    Pwlf query(std::int64_t lo, std::int64_t hi) const;

  private:
    // Static balanced BST over boundary keys 0..m+1, array of nodes indexed by
    // key; parent/subtree ranges implicit through build recursion results.
    struct Node {
        std::int64_t subtree_lo = 0, subtree_hi = 0;  // key range of the subtree
        std::int64_t parent = -1;
        std::int64_t depth = 0;
        // stored[d - subtree_lo] = G(min(d,h), max(d,h)) for descendant key d
        // (entry for d == h unused and kept empty).
        std::vector<Pwlf> stored;
    };

    void build_range(std::int64_t lo, std::int64_t hi, std::int64_t parent,
                     std::int64_t depth);
    void fill_node(std::int64_t h);
    void refill_node_around(std::int64_t h, std::int64_t leaf);
    const Pwlf& stored_fn(std::int64_t h, std::int64_t d) const {
        return nodes_[static_cast<std::size_t>(h)]
            .stored[static_cast<std::size_t>(d - nodes_[static_cast<std::size_t>(h)].subtree_lo)];
    }

    std::vector<Pwlf> leaves_;
    std::vector<Node> nodes_;  // indexed by boundary key
    std::int64_t root_ = -1;
};

}  // namespace kayros
