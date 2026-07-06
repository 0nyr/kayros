//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "preprocess/preprocess_triangle_depot.h"
#include "preprocess/preprocess_utils.h"

#include "nyr/vrp/instance.h"

using namespace std;
using namespace goc;
using namespace nyr;
using namespace nlohmann;

namespace solver
{
void preprocess_triangle_depot(json& instance)
{
	VRPInstance vrp = instance;
	Vertex o = vrp.o, d = vrp.d;
	for (Vertex i: vrp.D.Vertices())
	{
		for (Vertex j: vrp.D.Successors(i))
		{
			if (i == o || j == d) continue;
			TimeUnit b_i = max(vrp.tw[i]), a_j = min(vrp.tw[j]);
			double t0_ij = vrp.TravelTime({i, d}, b_i) + vrp.TravelTime({o, j}, vrp.ArrivalTime({i, d}, b_i));
			if (epsilon_smaller_equal(t0_ij, a_j - b_i))
				remove_arc(instance, i, j);
		}
	}
}
} // namespace