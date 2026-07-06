#pragma once

#include <goc/goc.h>

#include "nyr/vrp/instance.h"
#include "nyr/solutions/vrp_solution.h"

namespace nyr
{

VRPSolutionDuration convert_makespan_solution_to_duration(
    const VRPSolutionMakespan& makespan_solution,
    const VRPInstance& vrp
);

VRPSolutionTravelTime convert_makespan_solution_to_travel_time(
    const VRPSolutionMakespan& makespan_solution,
    const VRPInstance& vrp
);

template<typename Solution>
Solution auto_convert_makespan_solution(
    const VRPSolutionMakespan& makespan_solution, 
    const VRPInstance& vrp
) {
    if constexpr (IsMakespanSolution<Solution>)
    {
        return makespan_solution; // No conversion needed
    }
    else if constexpr (IsDurationSolution<Solution>)
    {
        return convert_makespan_solution_to_duration(makespan_solution, vrp);
    }
    else if constexpr (IsTravelTimeSolution<Solution>)
    {
        return convert_makespan_solution_to_travel_time(makespan_solution, vrp);
    }
    else
    {
        static_assert(std::same_as<Solution, void>, "Unsupported solution type for GMH1");
    }
}

} // namespace nyr