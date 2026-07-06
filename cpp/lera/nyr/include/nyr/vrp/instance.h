#pragma once

#include <vector>
#include <goc/goc.h>

#include "nyr/vrp/types.h"
#include "nyr/solutions/route.h"


namespace nyr
{

// This class represents an instance of a vehicle routing problem.
// Considerations:
// 	- It considers two depots (origin and destination).
class VRPInstance : public goc::Printable
{
public:
	goc::Digraph D; // digraph representing the network.
	goc::Vertex o, d; // origin and destination depot.
	TimeUnit T; // end of planning horizon ([0,T]). TODO: Remove, replace by horizon.right.
	goc::Interval horizon; // horizon of the instance [0, T]
	//goc::Interval horizon; // horizon of the instance.
	std::vector<goc::Interval> tw; // time window of customers (tw[i] = time window of customer i).
	CapacityUnit Q; // vehicle capacity.
	std::vector<CapacityUnit> q; // demand of customers (q[i] = demand of customer i).
	
	// WARN: \tau and \alpha (arr) are modified with Lera's preprocessing.
	// SHOULD NOT BE USED WITH TravelTime objective.
	goc::Matrix<goc::PWLFunction> tau; // tau[i][j](t) = travel time of arc (i, j) if departing from i at t.
	goc::Matrix<goc::PWLFunction> pretau; // pretau[i][j](t) = travel time of arc (i, j) if arriving at j at t.
	goc::Matrix<goc::PWLFunction> dep; // dep[i][j](t) = departing time of arc (i, j) if arriving to j at t.
	goc::Matrix<goc::PWLFunction> arr; // arr[i][j](t) = arrival time of arc (i, j) if departing from i at t.
	goc::Matrix<TimeUnit> LDT; // LDT[i][j] = latest time i can depart from i to reach j before its deadline.
	std::vector<goc::Interval> time_steps; // time steps.

	// Returns: the travel time for arc e if departing at t0.
	// If departure at t0 is infeasible, returns INFTY.
	TimeUnit TravelTime(goc::Arc e, TimeUnit t0) const;
	
	// Returns: the travel time for arc e if arriving at tf.
	// If arrival at tf is infeasible, returns INFTY.
	TimeUnit PreTravelTime(goc::Arc e, TimeUnit tf) const;
	
	// Returns: the arrival time for arc e if departing at t0.
	// If departure at t0 is infeasible, returns INFTY.
	TimeUnit ArrivalTime(goc::Arc e, TimeUnit t0) const;
	
	// Returns: the departure time for arc e if arriving at tf.
	// If arrival at tf is infeasible, returns INFTY.
	TimeUnit DepartureTime(goc::Arc e, TimeUnit tf) const;
	
	// Returns: the time we finish visiting the last vertex if departing at t0.
	// If infeasible, returns a route with empty path and INFTY duration.
	TimeUnit ReadyTime(const goc::GraphPath& p, TimeUnit t0=0) const;

	// Returns: the route with minimum duration using path p.
	// If the route is infeasible it returns empty route with INFTY duration.
	RouteDuration BestDurationRoute(const goc::GraphPath& p) const;

	// Returns: the route with minimum travel time using path p.
	// If the route is infeasible it returns empty route with INFTY TravelTime.
	RouteTravelTime BestTravelTimeRoute(const goc::GraphPath& p) const;
	
	// Returns: a set of all vertices which are unreachable if departing from v at t0.
	VertexSet Unreachable(goc::Vertex v, TimeUnit t0) const;
	
	// Returns: a set of some vertices which are unreachable if departing from v at t0.
	VertexSet WeakUnreachable(goc::Vertex v, TimeUnit t0) const;
	
	// @return the minimum travel time for arc e if departing at or after t0.
	// @details if departing at or after t0 is infeasible it returns INFTY.
	TimeUnit MinimumTravelTime(goc::Arc e, TimeUnit t0=0.0, TimeUnit tf=goc::INFTY) const;

	// Prints the JSON representation of the instance.
	virtual void Print(std::ostream& os) const;

	inline size_t nb_vertices() const {
		return D.NbVertices();
	}

	inline size_t nb_clients() const {
		return D.NbVertices() - 2; // Exclude origin and destination depots.
	}

	/// Returns: a vector with all the clients (vertices except depots).
	std::vector<goc::Vertex> copy_clients() const;

	/// Returns: first (included) vertex in the range of clients.
	inline size_t clients_range_start() const {
		return 1;
	}
	/// Returns: last (excluded) vertex in the range of clients.
	inline size_t clients_range_end() const {
		return d; // Exclude destination depot.
	}
	/// Returns: The range of clients [first client, d] to be iterated
	/// over with '<' operator.
	inline std::pair<size_t, size_t> clients_range() const {
		return {1, d};
	}
};

// Serializes the instance.
void to_json(nlohmann::json& j, const VRPInstance& instance);

// Parses an instance.
void from_json(const nlohmann::json& j, VRPInstance& instance);
} // namespace

