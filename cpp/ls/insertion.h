#pragma once

#include <cstdint>
#include <vector>

#include "ls/ls.h"

namespace kayros {

// Tree-ranked feasible insertion candidates of one client over all routes x
// positions (shared-prefix scan per route, capacity-screened), best-first by
// (delta, route, position). Ranking only: commits go through the checker-fold
// rebuild (build_route_state), next-best on a fold disagreement. Hoisted
// verbatim from perturb.cpp (M3, plan 12); shared by the ruin-and-recreate
// repair and the fleet-descent ladder.
struct InsertionCandidate {
    double delta;
    std::size_t route;
    std::int64_t position;
};

std::vector<InsertionCandidate> insertion_candidates(
    const Instance& inst, const std::vector<RouteState>& states,
    std::int32_t c, const Pwlf& dep);

}  // namespace kayros
