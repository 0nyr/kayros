//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "bcp/pricing_problem.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace solver
{
void PricingProblem::Print(std::ostream& os) const
{
	os << json(*this);
}

void to_json(json& j, const PricingProblem& p)
{
	j["forbidden_arcs"] = p.A;
	j["profits"] = p.P;
	j["cuts"] = p.S;
	j["cuts_duals"] = p.sigma;
}
} // namespace