// In a header, for example "nyr/vrp/route_traits.h"

#pragma once

#include "nyr/solutions/route.h"
#include "nyr/vrp/types.h"
#include "nyr/solutions/objectives.h"

namespace nyr
{

template<typename Route>
struct ObjectiveFunctionOf;

template<>
struct ObjectiveFunctionOf<RouteMakespan>
{
    static constexpr ObjectiveFunction value = ObjectiveFunction::Makespan;
};

template<>
struct ObjectiveFunctionOf<RouteDuration>
{
    static constexpr ObjectiveFunction value = ObjectiveFunction::Duration;
};

template<>
struct ObjectiveFunctionOf<RouteTravelTime>
{
    static constexpr ObjectiveFunction value = ObjectiveFunction::TravelTime;
};

} // namespace nyr
