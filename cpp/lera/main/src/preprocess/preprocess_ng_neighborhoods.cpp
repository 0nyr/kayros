#include "preprocess/preprocess_ng_neighborhoods.h"
#include "nyr/vrp/instance.h"
#include <nyr/math/fast_math.h>

#include <vector>
#include <queue>
#include <algorithm>
#include <cstdint>

using namespace std;
using namespace goc;
using namespace nyr;
using namespace nlohmann;

namespace solver
{
namespace
{
// Solve a one-to-all makespan minimization time dependent 
// shortest path for a given vertex (source), at a given departure time.
// Returns: the optimal makespan of every other vertex from the source.
// NOTE: Due to FIFO property, waiting is not allowed.
// WARNING: Different from compute_EAT_from_departure_time
// because this function considers the time windows.
vector<double> compute_one_to_all_earliest_arrival_time(
    const VRPInstance& vrp,
    Vertex source,
    TimeUnit departure_time
) {
    int n = vrp.D.NbVertices();

    if (vrp.tw[source].right < departure_time)
        // The source is not reachable at the given departure time
        return {};

    vector<double> arrival_times(n, INFTY); // all arrival times are in [0, INFTY).
    arrival_times[source] = max(departure_time, vrp.tw[source].left); // arrival time, considering no waiting time and start at departure_time.

    // Priority queue to select the vertex with the smallest arrival time.
    priority_queue<pair<double, Vertex>, vector<pair<double, Vertex>>, greater<pair<double, Vertex>>> Q;
    Q.push({arrival_times[source], source});

    // TD Dijkstra's algorithm, no "visited" vector needed.
    while (!Q.empty())
    {
        auto [arrival_time_at_i, i] = Q.top();
        Q.pop();

        // Update the makespan of the successors.
        for (Vertex j: vrp.D.Successors(i))
        {
            // Note: TW and service times are already considered in the function.
            double arrival_time_at_j = vrp.ArrivalTime({i, j}, arrival_time_at_i);
            // Check for improvement, and TW feasibility.
            if (arrival_time_at_j < arrival_times[j])
            {
                arrival_times[j] = arrival_time_at_j;
                Q.push({arrival_time_at_j, j});
            }
        }
    }

    return arrival_times;
}

// Partition the time horizon into intervals following the given strategy.
// Precondition: There must be more than one time step.
PartitionedInterval partition_time_horizon(
    const Interval& horizon,
    const vector<Interval>& time_steps,
    TDNGNeighborhoodsTimeStrategy time_strategy
) {
    if (time_strategy == TDNGNeighborhoodsTimeStrategy::TimeStepSpecific)
    {
        // Reuse the time steps directly.
        return PartitionedInterval(time_steps);
    }
    else if (time_strategy == TDNGNeighborhoodsTimeStrategy::PartitionedHorizon)
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
    else if (time_strategy == TDNGNeighborhoodsTimeStrategy::RepresentativeTimePeriods)
    {
        // Hard. Dynamically determine the representative time periods for every travel time functions and aggregate them.
        // TODO: Implement this.
        throw runtime_error("Not implemented.");
    }
    else if (time_strategy == TDNGNeighborhoodsTimeStrategy::Static)
    {
        // Only one interval: the horizon.
        return PartitionedInterval(horizon);
    }
    else
    {
        throw runtime_error("Unknown time strategy.");
    }
}
} // anonymous namespace  

void preprocess_ng_neighborhoods(
    nlohmann::json& instance,
    TDNGNeighborhoodsTimeStrategy horizon_partitioning_strategy,
    uint32_t nb_neighbors_to_keep
) {
    clog << " - NG neighborhoods" << endl;

    // Step 1: Determine time periods to compute the neighborhoods on.
    const Interval horizon = instance["horizon"];
    const vector<Interval> time_steps = instance["time_steps"];
    PartitionedInterval partitioned_horizon = partition_time_horizon(
        horizon, 
        time_steps, 
        horizon_partitioning_strategy
    );

    VRPInstance vrp = instance;
	const int n = vrp.D.NbVertices();
	const auto& V = vrp.D.Vertices();
    const Matrix<PWLFunction> taus = instance["travel_times"];
    const vector<Interval> tws = instance["time_windows"];
    const int nb_horizon_partitions = partitioned_horizon.nb_intervals();
    vector<vector<vector<Vertex>>> td_ng_neighbors(n, // for each vertex
        vector<vector<Vertex>>(nb_horizon_partitions, // for each time period
            vector<Vertex>() // neighbors, empty (no neighbors) by default
        )
    );
    clog << "TD NG Neighbors processing..." << endl;
    for (Vertex i: V)
    {
        for (int t = 0; t < nb_horizon_partitions; ++t)
        {   
            // Step 2: Solve a one-to-all makespan minimization 
            // time dependent shortest path for each vertex, 
            // for each time period.
            vector<double> makespans_i_t = compute_one_to_all_earliest_arrival_time(
                vrp, 
                i, 
                partitioned_horizon.get_interval(t).left
            );
            if (makespans_i_t.empty()) continue; // The vertex is not reachable at the given departure time for the given time period.

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

            // print each vertex and its makespan
            clog << " - Vertex " << i << " in period " << t << " at time " << partitioned_horizon.get_interval(t).left << " makespans: ";
            for (Vertex j: neighbors)
            {
                clog << j;
                if (j == i) clog << " (self)";
                clog << " -> " << makespans_i_t[j];
                if (j != neighbors.back()) clog << ", ";
            }
            clog << endl;
            // Remove the vertex itself
            neighbors.erase(remove(neighbors.begin(), neighbors.end(), i), neighbors.end());
            // Remove the vertex itself, which is always the closest. WRONG in TD context.
            //neighbors.erase(neighbors.begin());

            // Keep only the closest neighbors.
            if (neighbors.size() > nb_neighbors_to_keep)
                neighbors.resize(nb_neighbors_to_keep);
            // Store the neighbors.
            td_ng_neighbors[i][t] = neighbors;
        }
    }

    // Step 4: Store the neighborhoods.
    instance["td_ng_neighbors"] = td_ng_neighbors;

    // debug print
    // #ifndef PRINT_NEIGHBORHOODS_PREPROCESSING
    // #define PRINT_NEIGHBORHOODS_PREPROCESSING
    // #endif
    #ifdef PRINT_NEIGHBORHOODS_PREPROCESSING
    clog << "TD NG Neighbors:" << endl;
    for (Vertex i: V)
    {
        clog << " - Vertex " << i << ":" << endl;
        for (int t = 0; t < nb_horizon_partitions; ++t)
        {
            clog << "   - Time period " << t << "[t=" << partitioned_horizon.get_interval(t).left << "]: ";
            clog << "     " << td_ng_neighbors[i][t] << endl;
        }
    }
    #endif
}
} // namespace