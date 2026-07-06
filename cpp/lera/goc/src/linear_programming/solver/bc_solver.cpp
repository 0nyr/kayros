//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/linear_programming/solver/bc_solver.h"

// kayros (M5.1): compile-time LP backend switch (CPLEX default, HiGHS optional).
#ifdef GOC_LP_BACKEND_HIGHS
#include "goc/linear_programming/highs/matrix_formulation.h"
#include "goc/linear_programming/highs/highs_solver.h"
#else
#include "goc/linear_programming/cplex/cplex_formulation.h"
#include "goc/linear_programming/cplex/cplex_solver.h"
#endif
#include "goc/time/duration.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
BCSolver::BCSolver()
{
	time_limit = Duration::Max();
	config = {};
	screen_output = nullptr;
}

BCExecutionLog BCSolver::Solve(Formulation* formulation, const std::unordered_set<BCOption>& options) const
{
#ifdef GOC_LP_BACKEND_HIGHS
	return highs::solve_bc((MatrixFormulation*)formulation, screen_output, time_limit, config, initial_solutions,
						  branch_priorities, separation_strategy, options);
#else
	return cplex::solve_bc((CplexFormulation*)formulation, screen_output, time_limit, config, initial_solutions,
						  branch_priorities, separation_strategy, options);
#endif
}

Formulation* BCSolver::NewFormulation()
{
#ifdef GOC_LP_BACKEND_HIGHS
	return new MatrixFormulation();
#else
	return new CplexFormulation();
#endif
}
} // namespace goc