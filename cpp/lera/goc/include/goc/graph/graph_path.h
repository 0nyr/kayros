//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#pragma once

#include <limits.h>
#include <vector>

#include "goc/graph/vertex.h"

namespace goc
{
// Represents a path in a graph or digraph.
typedef std::vector<Vertex> GraphPath;

// Returns: if the path 'p' contains a cycle of size 'max_size' vertices or less.
bool has_cycle(GraphPath p, int max_size=INT_MAX);

// Returns: if two paths are equal.
bool operator==(const GraphPath& p1, const GraphPath& p2);

// Returns: if two paths are different.
inline bool operator!=(const GraphPath& p1, const GraphPath& p2) {
    return !(p1 == p2);
}

} // namespace goc

