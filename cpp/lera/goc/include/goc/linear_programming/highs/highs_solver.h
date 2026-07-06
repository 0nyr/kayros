//
// kayros addition (plan 2, Stream 5 M5.1) — NOT part of Lera-Romero's goc.
// HiGHS backend mirroring goc/linear_programming/cplex/cplex_solver.h.
// Stateless: the HiGHS model is materialized from the Formulation per solve
// (single source of truth; duals come postsolved to the original space, so
// the presolve-weakening dance the CPLEX backend needed does not apply).
//

#ifndef GOC_LINEAR_PROGRAMMING_HIGHS_HIGHS_SOLVER_H
#define GOC_LINEAR_PROGRAMMING_HIGHS_HIGHS_SOLVER_H

#include <iostream>
#include <string>
#include <unordered_set>

#include "goc/linear_programming/highs/matrix_formulation.h"
#include "goc/linear_programming/cuts/separation_strategy.h"
#include "goc/linear_programming/model/branch_priority.h"
#include "goc/linear_programming/solver/lp_solver.h"
#include "goc/linear_programming/solver/bc_solver.h"
#include "goc/log/bc_execution_log.h"
#include "goc/log/lp_execution_log.h"

namespace goc
{
namespace highs
{
// Solves the LP relaxation of the formulation with HiGHS (simplex, 1 thread).
LPExecutionLog solve_lp(MatrixFormulation* formulation,
						std::ostream* screen_output,
						Duration time_limit,
						const nlohmann::json& config,
						const std::unordered_set<LPOption>& options);

// Solves the formulation as a MIP with HiGHS (1 thread).
// Limitations vs the CPLEX backend (documented, unused by the BPC path):
// separation strategies, lazy constraints, initial solutions and branch
// priorities are not supported and fail loudly if provided.
BCExecutionLog solve_bc(MatrixFormulation* formulation,
				std::ostream* screen_output,
				Duration time_limit,
				const nlohmann::json& config,
				const std::vector<Valuation>& initial_solutions,
				const std::vector<BranchPriority>& branch_priorities,
				const goc::SeparationStrategy& separation_strategy,
				const std::unordered_set<BCOption>& options);
} // namespace highs
} // namespace goc

#endif //GOC_LINEAR_PROGRAMMING_HIGHS_HIGHS_SOLVER_H
