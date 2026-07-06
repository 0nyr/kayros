//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_LABEL_H
#define NETWORKS2019_LABEL_H

#include <iostream>

#include "goc/goc.h"
#include "nyr/vrp/types.h"

namespace solver
{
class Label : public goc::Printable
{
public:
	Label* parent;
	goc::Vertex v; // Last vertex.
	nyr::CapacityUnit q; // Label demand.
	nyr::ProfitUnit p; // Label profit.
	int length; // Label path length.
	nyr::VertexSet S; // Set S of vertices for domination. For NG-routes, this set is limited to the neighborhood of recently visited vertices.
	nyr::VertexSet U; // S \cup (union) "unreachables": vertices that are not reachable from the label.
	goc::PWLFunction duration; // duration(t) = "minimum duration for reaching v at time t".
	goc::Interval rw; // rw = dom(duration).
	double min_cost; // Label minimum cost (min{duration(t)-p-cut_cost : t \in dom(duration)}).
	std::vector<int> cut_visited; // visited vertices of the cut.
	std::vector<int> cut_nz; // cut indices that have cut_visited[i] % 2 > 0.
	double cut_cost; // total cost inflicted by the cuts duals.
	
	goc::GraphPath Path() const;
	
	virtual void Print(std::ostream& os) const;
};
} // namespace

#endif //NETWORKS2019_LABEL_H
