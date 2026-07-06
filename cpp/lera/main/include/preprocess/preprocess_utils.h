#pragma once

#include <goc/goc.h>

namespace solver
{
// Removes the arc ij from the instance.
void remove_arc(nlohmann::json& instance, goc::Vertex i, goc::Vertex j);

// Calculates the time to depart to traverse arc e arriving at tf.
// Returns: INFTY if it is infeasible to depart inside the horizon.
double departing_time(const nlohmann::json& instance, goc::Arc e, double tf);

// Calculates the travel time to traverse arc e departing at t0.
// Returns: INFTY if it is infeasible to arrive inside the horizon.
double travel_time(goc::PWLFunction& tau, goc::Arc e, double t0);
inline double travel_time(
    goc::Matrix<goc::PWLFunction>& travel_times,
    goc::Arc e,
    double t0
) {
    goc::PWLFunction& tau = travel_times[e.tail][e.head];
    return travel_time(tau, e, t0);
}
inline double travel_time(
    const nlohmann::json& instance, 
    goc::Arc e, 
    double t0
) {
	goc::PWLFunction tau = instance["travel_times"][e.tail][e.head];
    return travel_time(tau, e, t0);
}

// Returns: the latest we can arrive to k if departing from i (and traversing arc (i, k)) without waiting.
double latest_arrival(const nlohmann::json& instance, goc::Vertex i, goc::Vertex k);

// Returns: the earliest we can depart from i, to reach k inside its time window without waiting.
double earliest_departure(const nlohmann::json& instance, goc::Vertex i, goc::Vertex k);
} // namespace