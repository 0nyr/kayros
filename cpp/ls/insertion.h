#pragma once

#include <cstdint>
#include <vector>

#include "ls/ls.h"

namespace kayros {

// Rank capacity-screened insertions over every route and position by (delta, route, position). Ruin repair and fleet descent try these in order, rebuilding each proposed route with the search fold before accepting it.
struct InsertionCandidate {
    double delta;
    std::size_t route;
    std::int64_t position;
};

std::vector<InsertionCandidate> insertion_candidates(
    const Instance& inst, const std::vector<RouteState>& states,
    std::int32_t c, const Pwlf& dep);

}  // namespace kayros
