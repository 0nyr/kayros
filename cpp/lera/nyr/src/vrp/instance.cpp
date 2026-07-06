#include "nyr/vrp/instance.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace nyr
{

TimeUnit VRPInstance::TravelTime(goc::Arc e, TimeUnit t0) const
{
	auto& tau_e = tau[e.tail][e.head];
	if (epsilon_bigger(t0, max(dom(tau_e)))) return INFTY;
	else if (epsilon_smaller(t0, min(dom(tau_e)))) return min(dom(tau_e))+tau_e.Value(min(dom(tau_e)))-t0;
	return tau_e.Value(t0);
}

TimeUnit VRPInstance::PreTravelTime(goc::Arc e, TimeUnit tf) const
{
	auto& pretau_e = pretau[e.tail][e.head];
	if (epsilon_smaller(tf, min(dom(pretau_e)))) return INFTY;
	else if (epsilon_bigger(tf, max(dom(pretau_e)))) return tf-max(dom(pretau_e))+pretau_e.Value(max(dom(pretau_e)));
	return pretau_e.Value(tf);
}

TimeUnit VRPInstance::ArrivalTime(goc::Arc e, TimeUnit t0) const
{
	auto& arr_e = arr[e.tail][e.head];
	if (epsilon_bigger(t0, max(dom(arr_e)))) return INFTY;
	else if (epsilon_smaller(t0, min(dom(arr_e)))) return min(img(arr_e));
	return arr_e.Value(t0);
}

TimeUnit VRPInstance::DepartureTime(goc::Arc e, TimeUnit tf) const
{
	auto& dep_e = dep[e.tail][e.head];
	if (epsilon_smaller(tf, min(dom(dep_e)))) return INFTY;
	else if (epsilon_bigger(tf, max(dom(dep_e)))) return max(img(dep_e));
	return dep_e.Value(tf);
}

TimeUnit VRPInstance::ReadyTime(const GraphPath& p, TimeUnit t0) const
{
	CapacityUnit qq = q[0];
	TimeUnit t = t0;
	for (int k = 0; k < (int)p.size()-1; ++k)
	{
		Vertex i = p[k], j = p[k+1];
		if (!tau[i][j].Domain().Includes(t)) return INFTY;
		t += tau[i][j](t);
		qq += q[j];
	}
	if (epsilon_bigger(qq, Q)) return INFTY;
	return t;
}

// Sequential chain of compositions.
RouteDuration VRPInstance::BestDurationRoute(const GraphPath& p) const
{
	PWLFunction Delta = arr[p[0]][p[0]];
	if (Delta.Empty()) return {{}, 0.0, INFTY};
	for (int k = 0; k < (int)p.size()-1; ++k)
	{
		Vertex i = p[k], j = p[k+1];
		Delta = arr[i][j].Compose(Delta);
		if (Delta.Empty()) return {{}, 0.0, INFTY};
	}
	Delta = Delta - PWLFunction::IdentityFunction(dom(Delta));
	double min_img_Delta = min(img(Delta));
	return RouteDuration(
		p, 
		Delta.PreValue(min_img_Delta), 
		min_img_Delta
	);
}

VertexSet VRPInstance::Unreachable(Vertex v, TimeUnit t0) const
{
	VertexSet U;
	for (Vertex w: D.Vertices()) if (epsilon_bigger(t0, LDT[v][w])) U.set(w);
	return U;
}

VertexSet VRPInstance::WeakUnreachable(goc::Vertex v, TimeUnit t0) const
{
	VertexSet U;
	double min_tt = INFTY;
	for (Vertex w: D.Successors(v)) min_tt = min(min_tt, TravelTime({v,w}, t0));
	for (Vertex w: D.Vertices()) if (epsilon_bigger(t0+min_tt, tw[w].right)) U.set(w);
	return U;
}

RouteTravelTime VRPInstance::BestTravelTimeRoute(const goc::GraphPath& p) const
{
	
}

TimeUnit VRPInstance::MinimumTravelTime(Arc e, TimeUnit t0, TimeUnit tf) const
{
	TimeUnit tmin = INFTY;
	int j = 0;
	int v = e.tail, w = e.head;
	while (j < tau[v][w].PieceCount())
	{
		// Check if the piece starts beyond the time interval (end)
		if (epsilon_bigger(tau[v][w][j].domain.left, tf)) break;
		if (tau[v][w][j].domain.Intersects({t0, tf}))
			// the min of the current piece travel time is the min (left bound) of its image.
			tmin = min(tmin, tau[v][w][j].image.left); 
		// Check if the piece ends before the time interval (start)
		if (epsilon_bigger_equal(tau[v][w][j].domain.right, tf)) break;
		++j;
	}
	return tmin;
}

void VRPInstance::Print(ostream& os) const
{
	os << json(*this);
}

std::vector<goc::Vertex> VRPInstance::copy_clients() const {
	std::vector<goc::Vertex> vertices = D.Vertices();

	#ifndef NDEBUG
	assert(vertices[0] == this->o);
	assert(vertices.back() == this->d);
	#endif

	return std::vector<goc::Vertex>(
		vertices.begin() + 1, 
		vertices.end() - 1
	);
}

void to_json(json& j, const VRPInstance& instance)
{
	j["digraph"] = instance.D;
	j["start_depot"] = instance.o;
	j["end_depot"] = instance.d;
	j["horizon"] = vector<TimeUnit>({0, instance.T});
	j["time_windows"] = instance.tw;
	j["vehicle_capacity"] = instance.Q;
	j["demands"] = instance.q;
	j["travel_times"] = instance.tau;
}

void from_json(const json& j, VRPInstance& instance)
{
	int n = j["nb_vertices"];
	instance.D = j;
	instance.o = j["start_depot"];
	instance.d = j["end_depot"];
	instance.T = j["horizon"][1];
	instance.horizon = Interval(0.0, instance.T);
	//instance.horizon = j["horizon"];
	instance.tw = vector<Interval>(j["time_windows"].begin(), j["time_windows"].end());
	instance.Q = value_or_default(j, "vehicle_capacity", 1.0);
	instance.q = vector<CapacityUnit>(j["demands"].begin(), j["demands"].end());
	if (has_key(j, "time_steps"))
		instance.time_steps = vector<Interval>(j["time_steps"].begin(), j["time_steps"].end());
	else
		instance.time_steps = vector<Interval>(1, Interval(0.0, instance.T));

	// Add travel time functions.
	instance.tau = instance.arr = instance.dep = instance.pretau = Matrix<PWLFunction>(n, n);
	for (Vertex u: instance.D.Vertices())
	{
		for (Vertex v: instance.D.Successors(u))
		{
			instance.tau[u][v] = j["travel_times"][u][v];
			instance.arr[u][v] = instance.tau[u][v] + PWLFunction::IdentityFunction(instance.tau[u][v].Domain());
			instance.dep[u][v] = instance.arr[u][v].Inverse();
			instance.pretau[u][v] = PWLFunction::IdentityFunction(instance.dep[u][v].Domain()) - instance.dep[u][v];
		}
	}
	
	// Add travel functions for (i, i) (for boundary reasons).
	for (Vertex u: instance.D.Vertices())
	{
		instance.tau[u][u] = instance.pretau[u][u] = PWLFunction::ConstantFunction(0.0, instance.tw[u]);
		instance.dep[u][u] = instance.arr[u][u] = PWLFunction::IdentityFunction(instance.tw[u]);
	}
	
	// Set LDT.
	instance.LDT = Matrix<TimeUnit>(n, n);
	for (Vertex i: instance.D.Vertices())
	{
		vector<TimeUnit> LDT_i = compute_latest_departure_time(instance.D, i, instance.tw[i].right, [&] (Vertex u, Vertex v, double tf) { return instance.DepartureTime({u,v}, tf); });
		for (Vertex k: instance.D.Vertices()) instance.LDT[k][i] = LDT_i[k];
	}
}

} // namespace