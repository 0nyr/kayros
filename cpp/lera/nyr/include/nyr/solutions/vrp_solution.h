#pragma once

#include <vector>
#include <iostream>
#include <concepts>

#include <magic_enum/magic_enum.hpp>

#include "goc/lib/json.hpp"
#include "goc/print/printable.h"

#include "nyr/solutions/route.h"
#include "nyr/solutions/route_traits.h"
#include "nyr/solutions/objectives.h"
#include "nyr/vrp/types.h"

namespace nyr
{

/**
 * @brief Abstract class representing a solution to 
 * an optimization problem.
 * Every solution has a value as a double.
 */
class AbstractSolution: public goc::Printable
{
public:
	// The value of the solution.
	double value;
	
	// Constructor.
	AbstractSolution(double value) : value(value) {}
	
	// Destructor.
	virtual ~AbstractSolution() = default;
	
	// Prints the JSON representation of the solution.
	virtual void Print(std::ostream& os) const = 0;
};

// Represents a solution to a Vehicle Routing Problem.
// This solution has a value, and a set of routes.
// - It knows how to serialize itself in JSON to be compatible with Kaleidoscope kd_type "vrp_solution".
template<std::derived_from<AbstractRoute> Route>
class VRPSolution : public AbstractSolution
{
public:
	std::vector<Route> routes; // Solution routes.
	
	VRPSolution(): 
		AbstractSolution(0.0), 
		routes() 
	{}
	
	// Creates the solution with the specified parameters.
	VRPSolution(
		double value,
		const std::vector<Route>& routes
	):
		AbstractSolution(value), 
		routes(routes) 
	{}

	// Retrieve the associated objective function based on the route type
	ObjectiveFunction get_objective_function() const
	{
		return ObjectiveFunctionOf<Route>::value;
	}
	
	// Prints the JSON representation of the solution.
	void Print(std::ostream& os) const override
	{
		os << nlohmann::json(*this);
	}
};

typedef VRPSolution<RouteMakespan> VRPSolutionMakespan;
typedef VRPSolution<RouteDuration> VRPSolutionDuration;
typedef VRPSolution<RouteTravelTime> VRPSolutionTravelTime;

// Serializes the solution.
template<std::derived_from<AbstractRoute> Route>
void to_json(nlohmann::json& j, const VRPSolution<Route>& solution)
{
	j["objective"] = solution.get_objective_function();
    j["value"] = solution.value;
    j["routes"] = solution.routes;
}

// Parses a solution.
template<std::derived_from<AbstractRoute> Route>
void from_json(const nlohmann::json& j, VRPSolution<Route>& solution)
{
    solution.value = j["value"];
    solution.routes = j["routes"].get<std::vector<Route>>();
}

// Returns: if two solutions are equal.
template<std::derived_from<AbstractRoute> Route>
bool operator==(const VRPSolution<Route>& s1, const VRPSolution<Route>& s2)
{
    return s1.value == s2.value && s1.routes == s2.routes;
}

// Concepts for compile time checks.
template<typename Solution>
concept IsMakespanSolution = std::same_as<Solution, VRPSolutionMakespan>;
template<typename Solution>
concept IsDurationSolution = std::same_as<Solution, VRPSolutionDuration>;
template<typename Solution>
concept IsTravelTimeSolution = std::same_as<Solution, VRPSolutionTravelTime>;

template<typename VRPSolutionType>
requires std::same_as<VRPSolutionType, VRPSolutionMakespan> || 
		 std::same_as<VRPSolutionType, VRPSolutionDuration> || 
		 std::same_as<VRPSolutionType, VRPSolutionTravelTime>
void try_print_routes(const nyr::AbstractSolution& sol)
{
	if (auto* vrp_sol = dynamic_cast<const VRPSolutionType*>(&sol))
	{
		std::clog << "\tRoutes:" << std::endl;
		for (const auto& r : vrp_sol->routes)
			std::clog << "\t\t" << r << std::endl;
	}
	else
		std::clog << "Could not cast to correct VRPSolutionType." << std::endl;
}

} // namespace nyr