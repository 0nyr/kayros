#include "preprocess/preprocess_utils.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace solver
{
// Removes the arc ij from the instance.
void remove_arc(
    json& instance,
    Vertex i, 
    Vertex j
) {
	if (instance["arcs"][i][j] == 0) return;
	
	instance["arcs"][i][j] = 0;
	int arc_count = instance["arc_count"];
	instance["arc_count"] = arc_count - 1;

	// Remove arc from the digraph.
	if (has_key(instance, "travel_times")) 
		instance["travel_times"][i][j] = vector<json>({});
}

// Calculates the time to depart to traverse arc e arriving at tf.
// Returns: INFTY if it is infeasible to depart inside the horizon.
double departing_time(const json& instance, Arc e, double tf)
{
	PWLFunction tau_e = instance["travel_times"][e.tail][e.head];
	PWLFunction arr_e = tau_e + PWLFunction::IdentityFunction(dom(tau_e));
	if (epsilon_smaller(tf, min(img(arr_e)))) return INFTY;
	else if (epsilon_bigger(tf, max(img(arr_e)))) return max(dom(arr_e));
	return arr_e.PreValue(tf);
}

double travel_time(
	PWLFunction& tau,
	Arc e,
	double t0
) {
	if (!tau.Domain().Includes(t0)) return INFTY;
	return tau(t0);
}

// Returns: the latest we can arrive to k if departing from i (and traversing arc (i, k)) without waiting.
double latest_arrival(const json& instance, Vertex i, Vertex k)
{
	vector<Interval> tw = instance["time_windows"];
	if (departing_time(instance, {i, k}, tw[k].right) != INFTY) return tw[k].right;
	return tw[i].right + travel_time(instance, {i, k}, tw[i].right);
}

// Returns: the earliest we can depart from i, to reach k inside its time window without waiting.
double earliest_departure(const json& instance, Vertex i, Vertex k)
{
	vector<Interval> tw = instance["time_windows"];
	double dep = departing_time(instance, {i, k}, tw[k].left);
	if (!is_plus_infty(dep)) return dep;
	return tw[i].left;
}

// Earliest arrival time from i to all vertices if departing at departure_time.
vector<double> compute_EAT_from_departure_time(
    const Digraph& D,
    Matrix<PWLFunction>& travel_times,
    Vertex source,
    double departure_time
) {
    return compute_latest_departure_time(
		D, 
        source,
        departure_time,
        [&] (Vertex u, Vertex v, double t0) {
            return travel_time(travel_times, {u, v}, t0);
        }
    );
}

} // namespace
