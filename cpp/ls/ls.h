#pragma once

#include <cstdint>
#include <vector>

#include "core/instance.h"
#include "ls/lca_tree.h"

namespace kayros {

// TD local search (M3.7), per the Stream-4 decision memo
// (tdvrptw-workspace reports/design/td-ls-structures-decision.md):
// - the LCA-BST ranks candidate moves (≤ 1 compose per partial composition);
// - every ACCEPTED move is repriced by the sequential checker-identical fold
//   (evaluate_route) before being committed — trees rank, the fold accounts.

// Route leaves per the P4.0 convention, for r = (o, r0..r_{m-1}, o):
//   L_0 = theta_{r0} ∘ alpha_{o,r0} ∘ identity(depot departure window)
//   L_i = theta_{r[i]} ∘ alpha_{r[i-1],r[i]}          (1 <= i <= m-1)
//   L_m = [depot due-date clamp ∘] alpha_{r[m-1],o}
// so that delta_r = L_m ∘ ... ∘ L_0, with m+1 leaves in route order.
std::vector<Pwlf> route_leaves(const Instance& inst, const std::int32_t* route,
                               std::int64_t len);

// theta_to ∘ alpha_{from,to}: the leaf created by an arc that is not in either
// stored route (move bridges). Empty when the arc's ATF is absent.
Pwlf bridge_leaf(const Instance& inst, std::int32_t from, std::int32_t to);

// [depot due-date clamp ∘] alpha_{from,0}: the closing leaf after `from`.
Pwlf return_leaf(const Instance& inst, std::int32_t from);

// Per-route state owned by the LS layer (memo interface contract).
struct RouteState {
    std::vector<std::int32_t> vertices;  // customer ids, depot excluded
    LcaTree tree;                        // m+1 leaves, P4.0 convention
    double duration = 0.0;               // ALWAYS the checker-fold repriced value
    double departure = 0.0;              // earliest optimal depot departure
    std::int64_t load = 0;               // demand sum (capacity bookkeeping)
};

// Build (or rebuild after surgery) the full state: leaves + tree + checker
// fold repricing. Returns false when the route is time-infeasible.
bool build_route_state(const Instance& inst, std::vector<std::int32_t> vertices,
                       RouteState& state);

// Candidate evaluation (ranking only — never stored as a cost): the spliced
// route r1' = r1[0..i1-1] ++ r2[i2..j2] ++ r1[j1+1..], with the incoming
// segment empty when i2 > j2. Insertion, deletion, relocate segment donation,
// swap sides and 2-opt* tails are all instances of this shape.
RouteEval evaluate_splice(const Instance& inst, const RouteState& r1,
                          std::int64_t i1, std::int64_t j1,
                          const RouteState& r2, std::int64_t i2, std::int64_t j2);

// Intra-route relocate ranking: route r without the customer at position i,
// re-inserted before position p (p in 0..m, p != i, p != i+1; p == m appends
// before the depot return). Seam bridges recomposed, unchanged runs served by
// tree queries.
RouteEval evaluate_intra_relocate(const Instance& inst, const RouteState& r,
                                  std::int64_t i, std::int64_t p);

struct LsStats {
    std::int64_t applied = 0;   // committed moves (repriced improvements)
    std::int64_t reverted = 0;  // tree-ranked candidates the fold rejected
};

// First-improvement descent over inter-route relocate, intra-route relocate,
// inter-route swap and 2-opt*, iterated until a full cycle yields nothing.
// `routes` is modified in place (empty routes dropped, order otherwise kept);
// the return value is the canonical checker Duration of the final solution
// (solution_duration), kInfeasible when the input itself is infeasible.
double local_search(const Instance& inst,
                    std::vector<std::vector<std::int32_t>>& routes,
                    LsStats* stats = nullptr);

}  // namespace kayros
