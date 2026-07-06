//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_PREPROCESS_TRIANGLE_DEPOT_H
#define NETWORKS2019_PREPROCESS_TRIANGLE_DEPOT_H

#include <goc/goc.h>

namespace solver
{
// Takes a JSON instance of the vehicle routing problems with the following attributes:
//	- digraph
//	- travel_times
//	- time_windows
//	- start_depot
// 	- end_depot
// Removes arcs that are worse than going to the depot and leaving again.
void preprocess_triangle_depot(nlohmann::json& instance);
} // namespace

#endif //NETWORKS2019_PREPROCESS_TRIANGLE_DEPOT_H
