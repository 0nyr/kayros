#pragma once

#include <goc/goc.h>
#include "nyr/vrp/types.h"

#include <iostream>
#include <vector>

namespace nyr
{

/**
 * @brief Abstract class representing a route in a 
 * TSP/VRP problem. A Route is a path composed of
 * successive vertices, associated with a value
 * corresponding to the objective function.
 */
class AbstractRoute: public goc::Printable
{
public:
	// The path of the route.
	goc::GraphPath path;
	TimeUnit value; // Objective function value.
	
	// Constructor. Takes ownership of the path.
	AbstractRoute(
		const goc::GraphPath& path, 
		double value
	): 
		path(path), value(value) {}
	
	// Destructor.
	virtual ~AbstractRoute() = default;
	
	// Pure virtual, need to be implemented by derived classes.
	virtual void Print(std::ostream& os) const = 0;
};

/**
 * @brief A route with the Makespan objective.
 * Most simple route, just a path and a makespan.
 * The dispatch time is always 0.
 */
class RouteMakespan : public AbstractRoute
{
public:
	// Initializes the empty route.
	RouteMakespan():
		AbstractRoute({}, 0.0) {}
	
	// Initializes route with specified parameters.
	RouteMakespan(
		const goc::GraphPath& path, 
		TimeUnit makespan
	): 
		AbstractRoute(path, makespan) {}

	void Print(std::ostream& os) const override
	{
		os << nlohmann::json(*this);
	}
};

/**
 * @brief A route with the Duration objective.
 * The route is a path, and has a dispatch time t0
 * and a duration.
 */
class RouteDuration : public AbstractRoute
{
public:
	TimeUnit t0;
	
	// Initializes the empty route with t0=0, duration=0.
	RouteDuration():
		AbstractRoute({}, 0.0), t0(0.0) {}
	
	// Initializes route with specified parameters.
	RouteDuration(
		const goc::GraphPath& path, 
		TimeUnit t0, 
		TimeUnit duration
	):
		AbstractRoute(path, duration), t0(t0) {}
	
	void Print(std::ostream& os) const override
	{
		os << nlohmann::json(*this);
	}
};

/**
 * @brief A route with the TravelTime objective.
 * The route is a path, and has a departure (dispatch) time
 * for each vertex except the last one.
 */
class RouteTravelTime : public AbstractRoute
{
public:
    // Every non-last vertex has a departure time.
	std::vector<TimeUnit> t0s;

	// Initializes the empty route with t0=0, duration=0.
	RouteTravelTime()
		: AbstractRoute({}, 0.0), t0s() {}
	
	// Initializes route with specified parameters.
	RouteTravelTime(
		const goc::GraphPath& path, 
		const std::vector<TimeUnit>& t0s, 
		TimeUnit travel_time
	):
		AbstractRoute(path, travel_time), t0s(t0s) {}
	
	void Print(std::ostream& os) const override
	{
		os << nlohmann::json(*this);
	}
};

// Format: {"path": ..., "makespan": ...}
void to_json(nlohmann::json& j, const RouteMakespan& r);
void from_json(const nlohmann::json& j, RouteMakespan& r);
// Format: {"path": ..., "t0": ..., "duration": ...}
void to_json(nlohmann::json& j, const RouteDuration& r);
void from_json(const nlohmann::json& j, RouteDuration& r);
// Format: {"path": ..., "t0s": ..., "travel_time": ...}
void to_json(nlohmann::json& j, const RouteTravelTime& r);
void from_json(const nlohmann::json& j, RouteTravelTime& r);

// Returns: if two routes are equal.
bool operator==(const RouteMakespan& r1, const RouteMakespan& r2);
bool operator==(const RouteDuration& r1, const RouteDuration& r2);
bool operator==(const RouteTravelTime& r1, const RouteTravelTime& r2);
inline bool operator!=(const RouteMakespan& r1, const RouteMakespan& r2)
{
	return !(r1 == r2);
}
inline bool operator!=(const RouteDuration& r1, const RouteDuration& r2)
{
	return !(r1 == r2);
}
inline bool operator!=(const RouteTravelTime& r1, const RouteTravelTime& r2)
{
	return !(r1 == r2);
}

} // namespace goc