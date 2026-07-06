#pragma once

#include "nyr/vrp/types.h"
#include "nyr/vrp/instance.h"

#include <vector>
#include <goc/goc.h>

namespace solver
{

typedef std::vector<nyr::VertexSet> TDNeighborhoods; // Neighborhoods for each partition of the time horizon.

// Enum of the strategies to use to partition the time
// horizon into intervals for which the neighborhoods are.
enum class NeighborhoodHorizonPartitioningStrategy
{
    TimeStepSpecific, // Time-dependent neighborhoods are specific for each time step.
    PartitionedHorizon, // Time-dependent neighborhoods are given for a partitioned horizon into periods of same duration.
    Static // Time-dependent neighborhoods are static, they do not change over time.
};
typedef NeighborhoodHorizonPartitioningStrategy NHPS;

// Partition the time horizon into intervals following the given strategy.
// If there is no time steps or only one time step, the horizon is the only interval.
goc::PartitionedInterval partition_time_horizon(
    const goc::Interval& horizon,
    const std::vector<goc::Interval>& time_steps,
    NHPS horizon_partitioning_strategy
);

// A struct containing necessary params for TD-NG-Routes.
struct TDNGRoutesParams
{
    uint32_t nb_neighbors_to_keep; // Number of neighbors to keep for each vertex in the preprocessing step.
    uint32_t max_nb_neighbors; // Maximum number of neighbors to keep (Dynamic Neighborhood Augmentation DNA).
    goc::PartitionedInterval partitioned_horizon; // Time horizon, partitioned into successive intervals.
};

// Time-dependent neighborhoods.
class TDNGNeighborhoods
{
public:
    // Constructs a TDNGNeighborhoods.
    // The time horizon is partitioned into successive intervals
    // based on the strategy to use.
    TDNGNeighborhoods(
        const nyr::VRPInstance& vrp, 
        const goc::PartitionedInterval& partitioned_horizon,
        uint32_t nb_neighbors_to_keep
    );

    // Getter: the neighbors of the vertex i at time t.
    const nyr::VertexSet& neighbors(goc::Vertex i, nyr::TimeUnit t) const;

private:
    const goc::PartitionedInterval& partitioned_horizon_; // Time horizon, partitioned into successive intervals.

    // For each vertex, a map from interval end-time to its neighborhood.
    // Neighborhood is valid from previous end-time (or 0 initially) up to the key.
    std::vector<goc::VectorMap<nyr::TimeUnit, nyr::VertexSet>> ng_td_neighborhoods_;
};
} // namespace