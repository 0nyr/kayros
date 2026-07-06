#include "nyr/solutions/conversions.h"

#include <vector>

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace nyr
{

/**
 * Converts a VRPSolution from Makespan to Duration.
 */
VRPSolutionDuration convert_makespan_solution_to_duration(
    const VRPSolutionMakespan& makespan_solution,
    const VRPInstance& vrp
) {
    vector<RouteDuration> routes = vector<RouteDuration>(makespan_solution.routes.size());
    double total_duration = 0.0;
    for (size_t i = 0; i < makespan_solution.routes.size(); ++i)
    {
        routes[i] = vrp.BestDurationRoute(makespan_solution.routes[i].path);
        total_duration += routes[i].value;
    }
    return VRPSolutionDuration(total_duration, routes);
}

/**
 * Converts a VRPSolution from Makespan to TravelTime.
 */
VRPSolutionTravelTime convert_makespan_solution_to_travel_time(
    const VRPSolutionMakespan& makespan_solution,
    const VRPInstance& vrp
) {
    // TODO: implement.
}

} // namespace solver