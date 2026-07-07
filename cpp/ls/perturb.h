#pragma once

#include <cstdint>
#include <random>

#include "ls/ls.h"
#include "ls/neighbours.h"

namespace kayros {

// M7.1 ruin-and-recreate perturbation (Stream 7, td-ils-design.md),
// feasible-only: the SearchState holds a complete feasible solution before
// and after every call.
//
// Ruin: draw K ~ uniform[min_removals, max_removals] (clamped to n), walk a
// seeded random client order, and for each seed client remove it and its
// granular neighbours until K clients are out (removals keep every route
// feasible: a subsequence of a feasible route is feasible).
// Recreate: removed clients in random order, each committed at its best
// tree-ranked feasible position over all routes x positions; the committed
// route is rebuilt through the checker fold (repricing rule: trees rank, the
// fold accounts). A client with no feasible position opens a singleton route
// when the fleet bound allows; otherwise the whole kick is undone and redrawn
// (up to max_redraws), and the state is left unperturbed when every redraw
// fails.
// On success the SearchState epoch advances once and every client of every
// modified route is marked touched, so the following granular descent
// rescans exactly around the kick.
//
// Determinism: all draws come from the caller's rng via modulo/Fisher-Yates
// (no std::uniform_*_distribution — implementation-defined across platforms).
struct PerturbParams {
    std::int32_t min_removals = 1;
    std::int32_t max_removals = 25;
    std::int32_t max_redraws = 3;
};

struct PerturbOutcome {
    bool applied = false;        // a kick was ruined and fully repaired
    std::int32_t removed = 0;    // clients removed and reinserted
    std::int32_t redraws = 0;    // failed attempts that were undone
    std::int32_t new_routes = 0; // singleton routes opened by the repair
};

PerturbOutcome perturb(const Instance& inst, const NeighbourLists& nb,
                       SearchState& ss, std::mt19937_64& rng,
                       const PerturbParams& params);

}  // namespace kayros
