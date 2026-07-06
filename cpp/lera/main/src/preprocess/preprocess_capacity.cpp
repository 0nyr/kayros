//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "preprocess/preprocess_capacity.h"
#include "preprocess/preprocess_utils.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace solver
{
void preprocess_capacity(json& instance)
{
	clog << " - Capacity" << endl;
	
	double Q = instance["vehicle_capacity"];
	vector<double> q = instance["demands"];
	Digraph D = instance;
	for (Vertex i: D.Vertices())
		for (Vertex j: D.Successors(i))
			if (epsilon_bigger(q[i]+q[j], Q))
				remove_arc(instance, i, j);
}
} // namespace