//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_PREPROCESS_SERVICE_WAITING_H
#define NETWORKS2019_PREPROCESS_SERVICE_WAITING_H

#include <goc/goc.h>

namespace solver
{
// Takes a JSON instance of the vehicle routing problems with the following attributes:
//	- digraph
//	- travel_times
//	- time_windows (optional)
//	- service_times (optional)
// We transform the instances into ones without service times nor waiting times (due to TW earliest arrival). In order to do this we apply
// the preprocessing technique from Lera-Romero & Miranda-Bront 2019 (2.1).
// 	(i) 	s'i = 0.
// 	(ii) 	a'_i = a_i + s_i, b'_i = b_i + s_i for each i \in V.
// 	(iii) 	\tau'_ij(t) = max(a_j, t+\tau_ij(t)) - t + s_j for each ij \in A.
void preprocess_service_waiting(nlohmann::json& instance);
} // namespace

#endif //NETWORKS2019_PREPROCESS_SERVICE_WAITING_H
