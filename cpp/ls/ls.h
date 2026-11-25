#pragma once

#include <chrono>
#include <cstdint>
#include <vector>

#include "core/instance.h"
#include "ls/lca_tree.h"
#include "ls/neighbours.h"

namespace kayros {

// The Blauth range tree ranks splice candidates. Each accepted move must also improve after the affected routes are rebuilt with the binary64 search fold.

// Identity function over the feasible depot departure window (empty when the
// depot TW is inverted). The left end of every route fold.
Pwlf departure_identity(const Instance& inst);

// For a route (o, r0..r[m-1], o), the m+1 leaves are
//   L_0 = theta_{r0} ∘ alpha_{o,r0} ∘ identity(depot departure window)
//   L_i = theta_{r[i]} ∘ alpha_{r[i-1],r[i]}          (1 <= i <= m-1)
//   L_m = [depot due-date clamp ∘] alpha_{r[m-1],o}
// so that delta_r = L_m ∘ ... ∘ L_0, with m+1 leaves in route order.
std::vector<Pwlf> route_leaves(const Instance& inst, const std::int32_t* route,
                               std::int64_t len);

// Per-route state owned by local search.
struct RouteState {
    std::vector<std::int32_t> vertices;  // customer ids, depot excluded
    LcaTree tree;                        // One leaf per arc, including the return to the depot.
    double duration = 0.0;               // binary64 route fold value
    double departure = 0.0;              // earliest minimizing departure on the rebuilt binary64 curve
    std::int64_t load = 0;               // demand sum (capacity bookkeeping)
    // Search stamps reset by build_route_state.
    std::vector<double> del_dur;         // cached deletion rankings per position
    bool del_valid = false;              // del_dur filled for current vertices
};

// Build the leaves, tree and binary64 route price. Return false if no feasible departure is found.
bool build_route_state(const Instance& inst, std::vector<std::int32_t> vertices,
                       RouteState& state);

// Candidate evaluation (ranking only — never stored as a cost): the spliced
// route r1' = r1[0..i1-1] ++ r2[i2..j2] ++ r1[j1+1..], with the incoming
// segment empty when i2 > j2. Insertion, deletion, relocate segment donation,
// swap sides and 2-opt* tails are all instances of this shape.
RouteEval evaluate_splice(const Instance& inst, const RouteState& r1,
                          std::int64_t i1, std::int64_t j1,
                          const RouteState& r2, std::int64_t i2, std::int64_t j2);

// Price inserting c at position p of recv, starting with its already computed prefix.
bool price_insertion(const Instance& inst, const RouteState& recv, PwlfView prefix,
                     std::int32_t before, std::int32_t c, std::int64_t p,
                     std::int64_t mb, MinShift& out);

// Intra-route relocate ranking: route r without the customer at position i,
// re-inserted before position p (p in 0..m, p != i, p != i+1; p == m appends
// before the depot return). Seam bridges recomposed, unchanged runs served by
// tree queries.
RouteEval evaluate_intra_relocate(const Instance& inst, const RouteState& r,
                                  std::int64_t i, std::int64_t p);

struct LsStats {
    std::int64_t applied = 0;   // committed moves (repriced improvements)
    std::int64_t reverted = 0;  // tree-ranked candidates the fold rejected
    // Candidate pricings entered (one per move whose PWLF compose chain
    // starts, counted after the cheap screens). The deterministic LS work
    // unit: same seed => same count, and at a fixed instance the rate per
    // wall second is near-constant, so work-based triggers (fd_period_work,
    // restart_no_improvement_work) self-scale across instance sizes where
    // flat iteration counts span orders of magnitude.
    std::int64_t evaluated = 0;
};

// Keep search epochs across descents. Each committed move stamps its changed routes and all their customers. After a pass without a commit, stamp each tested customer for that operator. A later pass retests customers whose context or a relevant granular neighbour changed. With exhaustive lists any intervening commit permits a retest.
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

// Apply first-improvement relocate, intra-route relocate, swap and 2-opt* with the supplied neighbourhoods. The optional deadline stops the descent between operator passes, with all accepted routes repriced. Return the final binary64 objective.
double ls_descend(const Instance& inst, const NeighbourLists& nb,
                  SearchState& ss, LsStats* stats = nullptr,
                  const std::chrono::steady_clock::time_point* deadline = nullptr);

// Invalidate every staleness stamp (one epoch, all clients touched): required
// when the enumeration mode widens (granular -> exhaustive polish), since
// granular-era stamps certify a weaker coverage than exhaustive enumeration.
void mark_all_touched(SearchState& ss);

// Modify the supplied routes in place until a full operator cycle finds no improvement. Drop empty routes, preserve the order of remaining routes, and return the binary64 objective or kInfeasible for an infeasible input. The two-argument overload uses exhaustive enumeration.
double local_search(const Instance& inst,
                    std::vector<std::vector<std::int32_t>>& routes,
                    LsStats* stats = nullptr);
double local_search(const Instance& inst, const NeighbourLists& nb,
                    std::vector<std::vector<std::int32_t>>& routes,
                    LsStats* stats = nullptr);

}  // namespace kayros
