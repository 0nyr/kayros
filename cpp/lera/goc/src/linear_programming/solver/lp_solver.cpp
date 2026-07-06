//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/linear_programming/solver/lp_solver.h"

// kayros (M5.1): compile-time LP backend switch (CPLEX default, HiGHS optional).
#ifdef GOC_LP_BACKEND_HIGHS
#include "goc/linear_programming/highs/matrix_formulation.h"
#include "goc/linear_programming/highs/highs_solver.h"
#else
#include "goc/linear_programming/cplex/cplex_formulation.h"
#include "goc/linear_programming/cplex/cplex_solver.h"
#endif

using namespace std;
using namespace nlohmann;

namespace goc
{
LPSolver::LPSolver()
{
	// Set default values.
	time_limit = Duration::Max();
	config = {};
	screen_output = nullptr;
}

LPExecutionLog LPSolver::Solve(Formulation* formulation, const unordered_set<LPOption>& options) const
{
#ifdef GOC_LP_BACKEND_HIGHS
	return highs::solve_lp((MatrixFormulation*)formulation, screen_output, time_limit, config, options);
#else
	return cplex::solve_lp((CplexFormulation*)formulation, screen_output, time_limit, config, options);
#endif
}

Formulation* LPSolver::NewFormulation()
{
#ifdef GOC_LP_BACKEND_HIGHS
	return new MatrixFormulation();
#else
	return new CplexFormulation();
#endif
}
} // namespace goc