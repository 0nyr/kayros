//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "preprocess/preprocess_service_waiting.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace solver
{
void preprocess_service_waiting(json& instance)
{
	clog << " - Service Waiting" << endl;

	Digraph D = instance;
	Matrix<PWLFunction> tau = instance["travel_times"];
	auto s = [&] (Vertex i) -> double { return instance["service_times"][i]; };
	auto a = [&] (Vertex i) -> double { return instance["time_windows"][i][0]; };
	auto b = [&] (Vertex i) -> double { return instance["time_windows"][i][1]; };
	
	/// (iii) \tau'_ij(t) = max(a_j, t+\tau_ij(t)) - t + s_j for each ij \in A.
	for (Vertex i: D.Vertices())
	{
		for (Vertex j: D.Successors(i))
		{
			if (!tau[i][j].Domain().Includes(a(i)+s(i))) continue;
			PWLFunction id = PWLFunction::IdentityFunction(tau[i][j].Domain());
			PWLFunction arr = Max(a(j), id+tau[i][j]) + s(j);
			arr = Max(arr, arr.Value(a(i)+s(i)));
			arr = arr.RestrictImage({a(j)+s(j), b(j)+s(j)});
			arr = arr.RestrictDomain({a(i)+s(i), b(i)+s(i)});
			tau[i][j] = arr - id;
		}
	}
	instance["travel_times"] = tau; // After this, travel times now include service times, waiting if early arrival (TW.left), and shrinked domain due to TW.right.
	
	// (ii) a'_i = a_i + s_i, b'_i = b_i + s_i for each i \in V.
	if (has_key(instance, "time_windows"))
	{
		for (Vertex i: D.Vertices()) instance["time_windows"][i][0] = a(i) + s(i);
		for (Vertex i: D.Vertices()) instance["time_windows"][i][1] = b(i) + s(i);
	}
	
	// (i) 	s'i = 0.
	if (has_key(instance, "service_times"))
	{
		for (Vertex i: D.Vertices()) instance["service_times"][i] = 0.0;
	}
}
} // namespace