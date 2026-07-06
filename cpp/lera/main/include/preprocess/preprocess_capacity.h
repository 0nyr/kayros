//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_PREPROCESS_CAPACITY_H
#define NETWORKS2019_PREPROCESS_CAPACITY_H

#include <goc/goc.h>

namespace solver
{
// Takes a JSON instance of the vehicle routing problems with the following attributes:
//	- digraph
//	- capacity
//	- demands
// Removes arcs (i, j) such that q_i+q_j > Q.
void preprocess_capacity(nlohmann::json& instance);
} // namespace

#endif //NETWORKS2019_PREPROCESS_CAPACITY_H
