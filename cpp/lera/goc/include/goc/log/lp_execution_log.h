//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_LOG_LP_EXECUTION_LOG_H
#define GOC_LOG_LP_EXECUTION_LOG_H

#include <iostream>
#include <string>
#include <vector>

#include "goc/base/maybe.h"
#include "goc/linear_programming/model/valuation.h"
#include "goc/log/log.h"
#include "goc/time/duration.h"

namespace goc
{
// All the status that can result from a simplex execution. Unresolved is any
// outcome of the backend that is none of the others, or an Optimum whose
// objective value, requested duals or requested solution are not available.
// It carries no value, no duals and no solution. The basis is an exception: it
// is a hint for the exact layer, which needs no trust in it, so a requested
// basis that the backend cannot give leaves {basis} unset and the status as it is.
enum class LPStatus { DidNotStart, Infeasible, Unbounded, TimeLimitReached, MemoryLimitReached, Optimum, Unresolved };

// Final simplex basis of an LP: the indices of the basic columns and of the rows
// whose logical (slack) variable is basic, each in increasing order. Every other
// variable is nonbasic. Only set when the backend reports a basis with exactly
// 1 basic variable per row.
struct LPBasis
{
	std::vector<int> columns;
	std::vector<int> rows;
};

// This class stores information about the execution of a LP relaxation solver.
// It is compatible with the Kaleidoscope kd_type "lp".
class LPExecutionLog : public Log
{
public:
	Maybe<std::string> screen_output; // output of the algorithm in the screen.
	Maybe<Duration> time; // total time spent solving the relaxation.
	Maybe<LPStatus> status; // the status of the execution
	Maybe<int> simplex_iterations; // number of iterations the simplex algorithm did.
	Maybe<Valuation> incumbent; // best solution found.
	Maybe<double> incumbent_value; // value of the best solution found.
	Maybe<int> variable_count; // number of variables in the lp.
	Maybe<int> constraint_count; // number of constraints in the lp.
	Maybe<std::vector<double>> duals; // vector of the dual variables associated to the rows in the solution.
	Maybe<LPBasis> basis; // final basis, on request (LPOption::Basis).
	
	LPExecutionLog() = default;
	
	// Serialize log.
	virtual nlohmann::json ToJSON() const;
};

std::ostream& operator<<(std::ostream& os, LPStatus status);
} // namespace goc

#endif //GOC_LOG_LP_EXECUTION_LOG_H
