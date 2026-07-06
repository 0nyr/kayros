//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/linear_programming/solver/cg_solver.h"

#include "goc/linear_programming/colgen/colgen.h"
// kayros (M5.1): unused cplex includes removed (backend selection lives in
// lp_solver.cpp / bc_solver.cpp; this file only delegates to LPSolver).
#include "goc/time/duration.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
namespace
{
LPSolver default_lp_solver;
}

CGSolver::CGSolver()
{
	// Set default values.
	time_limit = Duration::Max();
	lp_solver = &default_lp_solver;
	screen_output = nullptr;
}

CGExecutionLog CGSolver::Solve(Formulation* formulation, const std::unordered_set<CGOption>& options) const
{
	return solve_colgen(formulation, screen_output, time_limit, pricing_function, lp_solver, options);
}

Formulation* CGSolver::NewFormulation()
{
	return default_lp_solver.NewFormulation();
}
} // namespace goc