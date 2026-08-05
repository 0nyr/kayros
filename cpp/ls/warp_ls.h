#pragma once

#include <cstdint>
#include <vector>

#include "core/warp_eval.h"
#include "ls/ls.h"

namespace kayros {

// Warp-augmented route segments and their LCA-BST (Stream 8, P8.2; design
// memo §5). A segment carries the PAIR
//   rho:   entry ready-time -> clamped exit ready-time (NDCPWLF)
//   omega: entry ready-time -> warp accrued inside the segment (ND PWLF)
// with concatenation (a then b):
//   rho_ab = rho_b ∘ rho_a,   omega_ab = omega_a + omega_b ∘ rho_a
// — a monoid in exact arithmetic, so the Blauth LCA-BST applies unchanged at
// ~2x storage / ~3 PWLF ops per concat (compose, compose, add).
//
// P4.1 discipline transposed: warp dissolves the domain wall, so BOTH channels
// are value channels subject to association dust. Trees RANK penalised moves;
// every accepted move is rebuilt and repriced by the sequential augmented fold
// (warp_route_functions), which is the only accountant for duration AND warp.

struct WarpSegment {
    Pwlf rho;
    Pwlf omega;  // same domain as rho by construction

    bool empty() const { return rho.xs.empty(); }
};

// b ∘ a in route order (a entered first). Empty segments propagate (hard
// horizon wall — time windows never empty a warp segment).
WarpSegment warp_concat(const WarpSegment& later, const WarpSegment& earlier);

// Warp leaves, P4.0 convention transposed (route_leaves twin):
//   L_0 = (theta~_{r0} ∘ alpha ∘ id_dep,  w_{r0} ∘ alpha ∘ id_dep)
//   L_i = (theta~_{r[i]} ∘ alpha,         w_{r[i]} ∘ alpha)
//   L_m = (return-clamp ∘ alpha,          return-warp ∘ alpha)
// TDVRP leaves carry an exactly-zero omega on rho's domain.
std::vector<WarpSegment> warp_route_leaves(const Instance& inst,
                                           const std::int32_t* route,
                                           std::int64_t len, double t_end);
WarpSegment warp_bridge_leaf(const Instance& inst, std::int32_t from,
                             std::int32_t to, double t_end);
WarpSegment warp_return_leaf(const Instance& inst, std::int32_t from,
                             double t_end);

// LcaTree twin over warp segments (same static BST, same localized update;
// bitwise ≡ rebuild is gate-tested). Kept as a plain copy-adapt of LcaTree for
// the P8.2 prototype — merging the two via a template is a P8.5 decision.
class WarpLcaTree {
  public:
    void build(std::vector<WarpSegment> leaves);
    void update_leaf(std::int64_t leaf, WarpSegment fn);

    std::int64_t num_leaves() const {
        return static_cast<std::int64_t>(leaves_.size());
    }
    const WarpSegment& leaf(std::int64_t i) const {
        return leaves_[static_cast<std::size_t>(i)];
    }

    WarpSegment query(std::int64_t lo, std::int64_t hi) const;

  private:
    struct Node {
        std::int64_t subtree_lo = 0, subtree_hi = 0;
        std::int64_t parent = -1;
        std::int64_t depth = 0;
        std::vector<WarpSegment> stored;
    };

    void build_range(std::int64_t lo, std::int64_t hi, std::int64_t parent,
                     std::int64_t depth);
    void fill_node(std::int64_t h);
    void refill_node_around(std::int64_t h, std::int64_t leaf);
    const WarpSegment& stored_fn(std::int64_t h, std::int64_t d) const {
        return nodes_[static_cast<std::size_t>(h)]
            .stored[static_cast<std::size_t>(
                d - nodes_[static_cast<std::size_t>(h)].subtree_lo)];
    }

    std::vector<WarpSegment> leaves_;
    std::vector<Node> nodes_;
    std::int64_t root_ = -1;
};

// Per-route state for the penalised search (RouteState twin; accounting
// values always from the sequential augmented fold). The fold's (rho, warp)
// functions are stored so the penalised cost can be re-scanned exactly when
// the penalty weight changes (dynamic penalty management) without a rebuild.
struct WarpRouteState {
    std::vector<std::int32_t> vertices;
    WarpLcaTree tree;
    Pwlf fold_rho;           // sequential-fold delta~ (the accountant's copy)
    Pwlf fold_warp;          // sequential-fold W
    double duration = 0.0;   // zero-warp duration when feasible, else the
                             // fold's penalised value at the build penalty
    double min_warp = 0.0;   // fold-accounted minimal warp (0.0 == feasible)
    std::int64_t load = 0;
};

// Fold-accounted penalised cost at an arbitrary penalty (exact re-scan of the
// stored fold functions — same evaluator the accounting uses).
double warp_state_cost(const WarpRouteState& state, double penalty);

// Build (or rebuild after surgery). Returns false only on a hard wall
// (horizon-empty rho) — warp-infeasible routes are valid states here.
bool build_warp_route_state(const Instance& inst,
                            std::vector<std::int32_t> vertices,
                            double penalty, double t_end, WarpRouteState& state);

// Penalised splice ranking (evaluate_splice twin): r1' = r1[0..i1-1] ++
// r2[i2..j2] ++ r1[j1+1..]. Returns ranking values only (repricing rule).
WarpRouteEval evaluate_splice_warp(const Instance& inst, const WarpRouteState& r1,
                                   std::int64_t i1, std::int64_t j1,
                                   const WarpRouteState& r2, std::int64_t i2,
                                   std::int64_t j2, double penalty, double t_end);

}  // namespace kayros
