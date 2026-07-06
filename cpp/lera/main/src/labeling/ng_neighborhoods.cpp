#include "labeling/ng_neighborhoods.h"
#include "goc/collection/bitset_utils.h"

#include <nyr/math/fast_math.h>
#include <nyr/vrp/instance.h>
#include <nyr/vrp/types.h>

using namespace std;
using namespace goc;
using namespace nyr;
using namespace nlohmann;

namespace solver
{
PartitionedInterval partition_time_horizon(
    const Interval& horizon,
    const vector<Interval>& time_steps,
    NHPS horizon_partitioning_strategy
) {
    // Precondition: If there is only one time step, then the horizon is the only interval.
    if (time_steps.size() <= 1)
    {
        return PartitionedInterval(horizon);
    }

    if (horizon_partitioning_strategy == NHPS::TimeStepSpecific)
    {
        // Reuse the time steps directly.
        return PartitionedInterval(time_steps);
    }
    else if (horizon_partitioning_strategy == NHPS::PartitionedHorizon)
    {
        // Aggregate the time steps.
        size_t nb_partitions_of_horizon = nyr::fast_log2(time_steps.size()) + 1; // better than just dividing by some value.
        
        vector<double> breakpoints;
        // Add first breakpoint.
        breakpoints.push_back(horizon.left);
        
        // Add the rest of the breakpoints, except the last one.
        double period_duration = (horizon.right - horizon.left) / nb_partitions_of_horizon;
        for (size_t i = 1; i < nb_partitions_of_horizon - 1; ++i)
        {
            breakpoints.push_back(i*period_duration + horizon.left);
        }
        // Add last breakpoint.
        breakpoints.push_back(horizon.right);

        assert (breakpoints.size() == nb_partitions_of_horizon && "Number of breakpoints must be equal to the number of partitions of the horizon.");
        
        return PartitionedInterval(breakpoints);
    }
    else if (horizon_partitioning_strategy == NHPS::Static)
    {
        // Only one interval: the horizon.
        return PartitionedInterval(horizon);
    }
    else
    {
        throw runtime_error("Unknown time strategy.");
    }
}

TDNGNeighborhoods::TDNGNeighborhoods(
    const VRPInstance& vrp, 
    const goc::PartitionedInterval& partitioned_horizon,
    uint32_t nb_neighbors_to_keep
):
    partitioned_horizon_(partitioned_horizon)
{
    const int n = vrp.D.NbVertices();
	const auto& V = vrp.D.Vertices();
    const int nb_horizon_partitions = partitioned_horizon.nb_intervals();

    // Compute MTT static neighborhoods
    clog << "Static MTT Neighbors processing..." << endl;
    Matrix<double> EAT(n, n);
    for (int i = 0; i < n; ++i) EAT[i] = compute_earliest_arrival_time(
        vrp.D, 
        i, 
        vrp.tw[i].left, // TW earliest arrival
        [&] (Vertex u, Vertex v, double t0) {
            return vrp.TravelTime({u, v}, t0);
        }
    );

    // Compute Neighbourhoods for all i in V, based on their Min Travel Time (MTT) of arc {i, j}
    //vector<VertexSet> MTT_static_neighborhood(n);
    vector<vector<Vertex>> MTT_static_ordered_neighbors(n);
    for (Vertex i: V)
    {
        vector<pair<double, Vertex>> i_neighbors_by_dist;
        for (Vertex j: vrp.D.Vertices())
        {
            // Filtering: obvious cases, and infeasible cases due to time windows, determined using EATs.
            if (i == j || j == vrp.o || j == vrp.d) continue;
            if (epsilon_bigger(EAT[j][i], vrp.tw[i].right)) continue;
            if (epsilon_bigger(EAT[i][j], vrp.tw[j].right)) continue;
            i_neighbors_by_dist.push_back({vrp.MinimumTravelTime({i,j}), j});
        }
        sort(i_neighbors_by_dist.begin(), i_neighbors_by_dist.end());
        // for (int k = 0; k < min((int)nb_neighbors_to_keep, (int)i_neighbors_by_dist.size()); ++k)
        // {
        //     MTT_static_neighborhood[i].set(i_neighbors_by_dist[k].second);
        // }
        // MTT_static_neighborhood[i].set(i); // add i to its own neighbourhood.
        i_neighbors_by_dist.resize(
            min((int)nb_neighbors_to_keep, (int)i_neighbors_by_dist.size())
        );
        vector<Vertex> neighbors = vector<Vertex>(i_neighbors_by_dist.size());
        for (int k = 0; k < (int)i_neighbors_by_dist.size(); ++k)
        {
            neighbors[k] = i_neighbors_by_dist[k].second;
        }
        MTT_static_ordered_neighbors[i] = neighbors;
        #ifdef PRINT_NEIGHBORHOODS_PREPROCESSING
        clog << " - Vertex " << i << " MTT neighbors: " << MTT_static_ordered_neighbors[i] << endl;
        #endif
    }

    // for each vertex, for each time period.
    ng_td_neighborhoods_ = vector<VectorMap<TimeUnit, VertexSet>>(n, // for each vertex
        goc::VectorMap<TimeUnit, VertexSet>() // period-end -> neighborhood
    );
    clog << "TD NG Neighbors processing..." << endl;
    for (Vertex i: V)
    {
        VertexSet prev_neighbors;
        for (int interval_index = 0; interval_index < nb_horizon_partitions; ++interval_index)
        {
            vector<double> makespans_i_t = compute_earliest_arrival_time(
                vrp.D, 
                i, 
                partitioned_horizon.get_interval(interval_index).left, // start of time period (subinterval of the horizon)
                [&] (Vertex u, Vertex v, double t0) {
                    return vrp.TravelTime({u, v}, t0);
                }
            );

            // Step 3: Determine the neighborhoods, i.e., the closest 
            // neighbors for each vertex, for each time period.
            vector<Vertex> neighbors = V;
            // Remove all vertices that have INFTY makespan.
            neighbors.erase(remove_if(neighbors.begin(), neighbors.end(), 
                [&makespans_i_t](Vertex j) -> bool
                {
                    return makespans_i_t[j] == INFTY;
                }
            ), neighbors.end());
            if (neighbors.empty()) continue; // No neighbors for the vertex.
            
            // Sort the vertices by makespan.
            sort(neighbors.begin(), neighbors.end(), 
                [&makespans_i_t](Vertex u, Vertex v) -> bool
                {
                    return makespans_i_t[u] < makespans_i_t[v];
                }
            );

            #ifdef PRINT_NEIGHBORHOODS_PREPROCESSING
            // print each vertex and its makespan
            clog << " - Vertex " << i << " in period " << interval_index << " at time " << partitioned_horizon.get_interval(interval_index).left << " makespans: ";
            for (Vertex j: neighbors)
            {
                clog << j;
                if (j == i) clog << " (self)";
                clog << " -> " << makespans_i_t[j];
                if (j != neighbors.back()) clog << ", ";
            }
            clog << endl;
            #endif

            // Remove the vertex itself
            neighbors.erase(remove(neighbors.begin(), neighbors.end(), i), neighbors.end());

            // Keep only the closest neighbors.
            VertexSet current_neighbors;
            if (neighbors.size() > nb_neighbors_to_keep)
            {
                neighbors.resize(nb_neighbors_to_keep);
                current_neighbors = create_bitset<MAX_N>(neighbors);
            }
            else
            {
                // Not enough neighbors, add the closest ones in static neighborhood.
                current_neighbors = create_bitset<MAX_N>(neighbors);
                for (Vertex j: MTT_static_ordered_neighbors[i])
                {
                    if (j == i) continue; // skip itself
                    if (current_neighbors.test(j)) continue; // already in the neighborhood
                    current_neighbors.set(j);
                    if (current_neighbors.count() >= nb_neighbors_to_keep) break;
                }
            }
            current_neighbors.set(i); // add i to its own neighbourhood.
            
            // Store the neighbors, if not already stored.
            TimeUnit interval_end = partitioned_horizon.get_interval(interval_index).right;
            if (interval_index == 0 || current_neighbors != prev_neighbors)
            {
                // Neighborhood has changed: store a new interval end and neighbors.
                ng_td_neighborhoods_[i].Insert(interval_end, current_neighbors);
                prev_neighbors = current_neighbors;
            }
            else
            {
                // Neighborhood has not changed: extend the last interval to cover up to interval_end.
                auto it = ng_td_neighborhoods_[i].end() - 1; // last interval
                it->first = interval_end;
            }
        }
    }

    #ifdef PRINT_NEIGHBORHOODS_PREPROCESSING
    print complete TD neighborhoods
    for (Vertex i: V)
    {
        clog << " - Vertex " << i << ":" << endl;
        for (auto [t, neighbors]: ng_td_neighborhoods_[i])
        {
            clog << "   - Time period [t-end=" << t << "]: ";
            clog << "     " << neighbors << endl;
        }
    }
    #endif
}

const VertexSet& TDNGNeighborhoods::neighbors(
    goc::Vertex i, 
    TimeUnit t
) const
{
    const auto& time_intervals = ng_td_neighborhoods_[i];
    // Gets the first interval whose end-time ≥ t.
    auto it = std::lower_bound(
        time_intervals.begin(), time_intervals.end(), t,
        [](const std::pair<TimeUnit, VertexSet>& interval, TimeUnit t)
        {
            return interval.first < t;
        }
    );
    if (it == time_intervals.end())
        return time_intervals.end()->second; // beyond last interval
    return it->second;
}

} // namespace