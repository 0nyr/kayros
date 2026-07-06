//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "preprocess/preprocess_time_windows.h"
#include "preprocess/preprocess_utils.h"

#include <vector>
#include <queue>

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace solver
{
void preprocess_time_windows(json& instance)
{
	clog << " - Time Windows" << endl;

	Digraph D = instance;
	int n = D.NbVertices();
	auto& V = D.Vertices();
	auto a = [&] (Vertex i) -> double { return instance["time_windows"][i][0]; };
	auto b = [&] (Vertex i) -> double { return instance["time_windows"][i][1]; };
	Vertex o = instance["start_depot"];
	Vertex d = instance["end_depot"];
	auto set_a = [&] (Vertex i, double t) { instance["time_windows"][i][0] = t; };
	auto set_b = [&] (Vertex i, double t) { instance["time_windows"][i][1] = t; };
	
	// Rule 1: (3.12) 	Upper bound adjustment derived from the latest arrival time at node k from its predecessors,
	//					for k \in N - {o, d}.
	for (Vertex k:exclude(V, {o,d}))
	{
		double max_arrival = -INFTY;
		for (Vertex i: D.Predecessors(k)) max_arrival = max(max_arrival, latest_arrival(instance, i, k));
		set_b(k, min(b(k), max(a(k), max_arrival)));
	}
	
	// Rule 2: (3.13)	Lower bound adjustment derived from the earliest departure time from node k to its successors,
	//					for k \in N - {o,d}.
	for (Vertex k:exclude(V, {o,d}))
	{
		double min_dep = INFTY;
		for (Vertex j: D.Successors(k)) min_dep = min(min_dep, earliest_departure(instance, k, j));
		set_a(k, max(a(k), min(b(k), min_dep)));
	}
	
	// Remove infeasible tw arcs.
	for (Arc ij: D.Arcs())
	{
		goc::Vertex i = ij.tail, j = ij.head;
		if (epsilon_bigger(a(i)+travel_time(instance, {i, j}, a(i)), b(j))) remove_arc(instance, i, j);
	}
}
} // namespace