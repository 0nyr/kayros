#pragma once

#include <cstdint>
#include <vector>

#include "core/instance.h"

namespace kayros {

// Granular candidate lists (Stream 7 M7.0, per tdvrptw-workspace
// reports/design/td-ils-design.md): per client, the num_neighbours nearest
// other clients under a TD adaptation of the Vidal 2013 proximity used by
// PyVRP:
//   prox(i,j) = mindur(i,j) + weight_wait * minwait(i,j)
// with mindur the exact minimum of alpha_ij(t) - t over the feasible
// departure domain [e_i + s_i, l_i + s_i] (from the ATF breakpoints),
// minwait = max(0, e_j - alpha_ij(latest feasible departure)) the waiting
// even the latest departure cannot avoid, TW-infeasible edges excluded, the
// proximity matrix symmetrised by min, and the resulting adjacency
// symmetrised by union so that is_neighbour(i, j) == is_neighbour(j, i).
// The union symmetrisation is load-bearing: the staleness rule in the LS
// ("retest i when some v in N(i) changed") is sound only if every granular
// enumeration justification is_neighbour(i, v) implies v in N(i).
//
// Lists cover clients 1..n only (the depot has no list and is never a
// granular justification). k == 0 is the exhaustive sentinel: no restriction,
// is_neighbour always true, neighbours_of unused.
struct NeighbourLists {
    std::int32_t k = 0;                    // requested size; 0 = exhaustive
    std::int32_t num_vertices = 0;         // n + 1 (bitset row stride)
    std::vector<std::int32_t> offsets;     // size n + 2; CSR over flat
    std::vector<std::int32_t> flat;        // union-symmetrised adjacency
    std::vector<std::uint64_t> bits;       // (n+1)^2 bits, row-major

    bool restricted() const { return k > 0; }

    // True when j is a granular neighbour of i (symmetric), or when the lists
    // are the exhaustive sentinel. Depot rows/columns are always false in
    // restricted mode.
    bool is_neighbour(std::int32_t i, std::int32_t j) const {
        if (k == 0) return true;
        const std::size_t bit =
            static_cast<std::size_t>(i) * static_cast<std::size_t>(num_vertices) +
            static_cast<std::size_t>(j);
        return (bits[bit >> 6] >> (bit & 63)) & 1u;
    }

    const std::int32_t* neighbours_begin(std::int32_t i) const {
        return flat.data() + offsets[static_cast<std::size_t>(i)];
    }
    const std::int32_t* neighbours_end(std::int32_t i) const {
        return flat.data() + offsets[static_cast<std::size_t>(i) + 1];
    }
};

// Build the lists from the instance ATFs. num_neighbours <= 0 returns the
// exhaustive sentinel. Deterministic: proximity ties break on client id.
NeighbourLists build_neighbour_lists(const Instance& inst,
                                     std::int32_t num_neighbours,
                                     double weight_wait);

}  // namespace kayros
