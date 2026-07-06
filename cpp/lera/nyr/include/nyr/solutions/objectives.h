#pragma once

#include <iostream>
#include <magic_enum/magic_enum.hpp>

namespace nyr
{

/**
 * ### Global Objective Function for the solver
 */
enum class ObjectiveFunction
{
    Makespan,   // Minimize makespan sum of all routes
    Duration,   // Minimize duration sum of all routes
    TravelTime, // Minimize travel time sum of all routes
};

inline std::ostream& operator<<(std::ostream& os, ObjectiveFunction value)
{
    return os << magic_enum::enum_name(value);
}

void throw_invalid_objective_function(
    const ObjectiveFunction& objective
);


} // namespace nyr