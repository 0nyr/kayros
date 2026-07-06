#pragma once

#include <goc/goc.h>

namespace solver
{
// Enum of the strategies to use for the TDNGNeighborhoods.
enum class TDNGNeighborhoodsTimeStrategy
{
    TimeStepSpecific, // Time-dependent neighborhoods are specific for each time step.
    PartitionedHorizon, // Time-dependent neighborhoods are given for a partitioned horizon into periods of same duration.
    RepresentativeTimePeriods, // Time-dependent neighborhoods are given for predefined time periods, with predetermined time periods less numerous and more representative of the time horizon changes in travel times.
    Static // Time-dependent neighborhoods are static, they do not change over time.
};

// Determine the NG neighborhoods of each vertex.
// Assumes preprocess_service_waiting was called (i.e. instance has no service nor waiting times).
// Assumes preprocess_time_windows was called (i.e. instance has shrinked time windows).
void preprocess_ng_neighborhoods(
    nlohmann::json& instance,
    TDNGNeighborhoodsTimeStrategy horizon_partitioning_strategy,
    uint32_t nb_neighbors_to_keep
);
} // namespace