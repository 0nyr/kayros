//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_PRICING_PROBLEM_H
#define NETWORKS2019_PRICING_PROBLEM_H

#include <vector>
#include <iostream>

#include "goc/goc.h"

#include "spf.h"

namespace solver
{
class PricingProblem : public goc::Printable
{
public:
	std::vector<goc::Arc> A; // Forbidden arcs.
	std::vector<double> P; // Profits of the vertices.
	std::vector<SubsetRowCut> S; // Cuts in the formulation.
	std::vector<double> sigma; // Duals associated with the cuts C.
	
	virtual void Print(std::ostream& os) const;
};

void to_json(nlohmann::json& j, const PricingProblem& p);

} // namespace

#endif //NETWORKS2019_PRICING_PROBLEM_H
