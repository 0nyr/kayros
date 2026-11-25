#pragma once

#include <cstdint>
#include <random>

#include "ls/ls.h"
#include "ls/neighbours.h"

namespace kayros {

// Ruin and recreate preserves a feasible complete search state or restores the supplied routes. Draw a target removal count, clamped at n, and remove customers around seeds in a shuffled order using their granular neighbours. Reinsert them in random order at the first tree-ranked position that passes a rebuild. If no insertion works, open a singleton when the fleet bound allows. Otherwise undo the kick and redraw, up to max_redraws. A successful kick advances the epoch and marks all customers in changed routes touched.
// With a positive route charge and at least two routes, a dissolve kick may seed the removal with every customer on a smallest route. Draws use the supplied RNG and shared modulo and Fisher-Yates operations.
struct PerturbParams {
    std::int32_t min_perturbations = 1;
    std::int32_t max_perturbations = 25;
    std::int32_t max_redraws = 3;
    std::int32_t dissolve_pct = 50;  // % of kicks that dissolve a smallest route
};

struct PerturbOutcome {
    bool applied = false;        // a kick was ruined and fully repaired
    std::int32_t removed = 0;    // clients removed and reinserted
    std::int32_t redraws = 0;    // failed attempts that were undone
    std::int32_t new_routes = 0; // singleton routes opened by the repair
    bool dissolved = false;      // the applied kick seeded a whole route
};

PerturbOutcome perturb(const Instance& inst, const NeighbourLists& nb,
                       SearchState& ss, std::mt19937_64& rng,
                       const PerturbParams& params);

}  // namespace kayros
