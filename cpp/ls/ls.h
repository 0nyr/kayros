#pragma once

#include <cstdint>
#include <vector>

#include "core/instance.h"
#include "ls/lca_tree.h"
#include "ls/neighbours.h"

namespace kayros {

// TD local search (M3.7), per the Stream-4 decision memo
// (tdvrptw-workspace reports/design/td-ls-structures-decision.md):
// - the LCA-BST ranks candidate moves (≤ 1 compose per partial composition);
// - every ACCEPTED move is repriced by the sequential checker-identical fold
//   (evaluate_route) before being committed — trees rank, the fold accounts.

// Identity function over the feasible depot departure window (empty when the
// depot TW is inverted). The left end of every route fold.
Pwlf departure_identity(const Instance& inst);

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
    // M7.0 bookkeeping (reset by build_route_state):
    std::vector<double> del_dur;         // cached deletion rankings per position
    bool del_valid = false;              // del_dur filled for current vertices
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

// M7.0 promising-set / staleness state, persistent across descents (the ILS
// loop keeps one SearchState alive; perturbation marks the ruined clients
// touched so the following descent rescans only around the kick).
//
// Epoch discipline: every committed move bumps `epoch`, stamps both touched
// routes' last_modified and every client of both post-move routes in
// `touched`. A pass that completes without committing stamps the clients it
// enumerated in `last_tested[op]`. A client is (re)enumerated by operator op
// iff its own context or some granular neighbour's context changed since its
// stamp; with exhaustive lists (k == 0) the neighbour term degrades to "any
// commit since the stamp", which makes the whole scheme skip work only after
// the descent has already proven a fixed point — enumeration order and hence
// the search trajectory are identical to the pre-M7.0 VND.
struct SearchState {
    std::vector<RouteState> states;
    std::vector<std::int64_t> touched;                    // per vertex id, size n+1
    std::vector<std::vector<std::int64_t>> last_tested;   // [4][n+1], per operator
    std::int64_t epoch = 1;
};

// Build a fresh SearchState (everything touched, nothing tested). Returns
// false when some route is time-infeasible.
bool init_search_state(const Instance& inst,
                       const std::vector<std::vector<std::int32_t>>& routes,
                       SearchState& ss);

// First-improvement VND descent on a live SearchState (relocate, intra
// relocate, swap, 2-opt*; granular enumeration under `nb`, staleness-gated).
// Returns the canonical checker Duration of the final solution.
double ls_descend(const Instance& inst, const NeighbourLists& nb,
                  SearchState& ss, LsStats* stats = nullptr);

// First-improvement descent over inter-route relocate, intra-route relocate,
// inter-route swap and 2-opt*, iterated until a full cycle yields nothing.
// `routes` is modified in place (empty routes dropped, order otherwise kept);
// the return value is the canonical checker Duration of the final solution
// (solution_duration), kInfeasible when the input itself is infeasible.
// The two-argument overload is the exhaustive pre-M7.0 behavior.
double local_search(const Instance& inst,
                    std::vector<std::vector<std::int32_t>>& routes,
                    LsStats* stats = nullptr);
double local_search(const Instance& inst, const NeighbourLists& nb,
                    std::vector<std::vector<std::int32_t>>& routes,
                    LsStats* stats = nullptr);

}  // namespace kayros
