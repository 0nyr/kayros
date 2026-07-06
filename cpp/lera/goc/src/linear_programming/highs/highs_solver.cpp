//
// kayros addition (plan 2, Stream 5 M5.1) — NOT part of Lera-Romero's goc.
//

#include "goc/linear_programming/highs/highs_solver.h"

#include <map>

#include "Highs.h"

#include "goc/collection/collection_utils.h"
#include "goc/exception/exception_utils.h"
#include "goc/math/number_utils.h"
#include "goc/time/stopwatch.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
namespace highs
{
namespace
{
double to_highs_bound(double bound)
{
	if (bound >= INFTY) return kHighsInf;
	if (bound <= -INFTY) return -kHighsInf;
	return bound;
}

// Materializes the formulation as a HighsModel (rowwise matrix).
//	relax: if true, all variables are continuous (LP relaxation, mirroring the
//	       CPLEX backend's chgprobtype(CPXPROB_LP)).
HighsModel materialize(MatrixFormulation* formulation, bool relax)
{
	HighsModel model;
	HighsLp& lp = model.lp_;
	int n = formulation->VariableCount();
	int m = formulation->ConstraintCount();
	lp.num_col_ = n;
	lp.num_row_ = m;
	lp.sense_ = formulation->GetObjectiveSense() == Formulation::Minimization ? ObjSense::kMinimize : ObjSense::kMaximize;

	Expression objective = formulation->ObjectiveFunction();
	lp.offset_ = objective.Scalar();
	lp.col_cost_ = vector<double>(n, 0.0);
	for (auto& term: objective.Terms()) lp.col_cost_[term.first.Index()] = term.second;

	lp.col_lower_ = vector<double>(n);
	lp.col_upper_ = vector<double>(n);
	for (int j = 0; j < n; ++j)
	{
		auto bounds = formulation->GetVariableBound(formulation->VariableAtIndex(j));
		lp.col_lower_[j] = to_highs_bound(bounds.first);
		lp.col_upper_[j] = to_highs_bound(bounds.second);
	}

	if (!relax)
	{
		lp.integrality_ = vector<HighsVarType>(n, HighsVarType::kContinuous);
		for (int j = 0; j < n; ++j)
		{
			VariableDomain domain = formulation->GetVariableDomain(formulation->VariableAtIndex(j));
			if (domain == VariableDomain::Integer || domain == VariableDomain::Binary)
				lp.integrality_[j] = HighsVarType::kInteger;
		}
	}

	lp.row_lower_ = vector<double>(m);
	lp.row_upper_ = vector<double>(m);
	lp.a_matrix_.format_ = MatrixFormat::kRowwise;
	lp.a_matrix_.start_ = {0};
	auto constraints = formulation->Constraints();
	for (int i = 0; i < m; ++i)
	{
		const Constraint& constraint = constraints[i];
		switch (constraint.Sense())
		{
			case Constraint::LessEqual: lp.row_lower_[i] = -kHighsInf; lp.row_upper_[i] = constraint.RightSide(); break;
			case Constraint::GreaterEqual: lp.row_lower_[i] = constraint.RightSide(); lp.row_upper_[i] = kHighsInf; break;
			case Constraint::Equality: lp.row_lower_[i] = lp.row_upper_[i] = constraint.RightSide(); break;
		}
		for (auto& term: constraint.LeftSide().Terms())
		{
			if (term.second == 0.0) continue;
			lp.a_matrix_.index_.push_back(term.first.Index());
			lp.a_matrix_.value_.push_back(term.second);
		}
		lp.a_matrix_.start_.push_back((HighsInt)lp.a_matrix_.index_.size());
	}
	return model;
}

// Configures a Highs instance: silent by default, single-threaded (kayros
// one-run-one-thread convention), time limit, then user config overrides.
void configure(Highs& solver, ostream* screen_output, Duration time_limit, const json& config)
{
	solver.setOptionValue("output_flag", screen_output != nullptr);
	solver.setOptionValue("threads", 1);
	solver.setOptionValue("time_limit", max(0.0, time_limit.Amount(DurationUnit::Seconds)));
	for (auto it = config.begin(); it != config.end(); ++it)
	{
		if (it.value().is_number_integer()) solver.setOptionValue(it.key(), (HighsInt)it.value());
		else if (it.value().is_number()) solver.setOptionValue(it.key(), (double)it.value());
		else if (it.value().is_boolean()) solver.setOptionValue(it.key(), (bool)it.value());
		else solver.setOptionValue(it.key(), (string)it.value());
	}
}

Valuation values_to_valuation(MatrixFormulation* formulation, const vector<double>& values)
{
	Valuation valuation;
	for (int j = 0; j < formulation->VariableCount(); ++j)
		valuation.SetValue(formulation->VariableAtIndex(j), values[j]);
	return valuation;
}
} // anonymous namespace

LPExecutionLog solve_lp(MatrixFormulation* formulation, ostream* screen_output, Duration time_limit,
						const json& config, const unordered_set<LPOption>& options)
{
	LPExecutionLog execution_log;

	Highs solver;
	configure(solver, screen_output, time_limit, config);
	solver.passModel(materialize(formulation, /*relax=*/true));

	Stopwatch rolex(true);
	solver.run();
	rolex.Pause();

	execution_log.time = rolex.Peek();
	execution_log.variable_count = formulation->VariableCount();
	execution_log.constraint_count = formulation->ConstraintCount();
	execution_log.simplex_iterations = (int)solver.getInfo().simplex_iteration_count;

	map<HighsModelStatus, LPStatus> status_mapper = {
		{HighsModelStatus::kOptimal, LPStatus::Optimum},
		{HighsModelStatus::kInfeasible, LPStatus::Infeasible},
		{HighsModelStatus::kUnboundedOrInfeasible, LPStatus::Infeasible},
		{HighsModelStatus::kUnbounded, LPStatus::Unbounded},
		{HighsModelStatus::kTimeLimit, LPStatus::TimeLimitReached},
		{HighsModelStatus::kMemoryLimit, LPStatus::MemoryLimitReached}};
	execution_log.status = status_mapper[solver.getModelStatus()];

	const HighsSolution& solution = solver.getSolution();
	if (solution.value_valid)
	{
		execution_log.incumbent_value = solver.getInfo().objective_function_value;
		if (includes(options, LPOption::Incumbent))
			execution_log.incumbent = values_to_valuation(formulation, solution.col_value);
	}
	if (includes(options, LPOption::Duals) && solution.dual_valid)
		execution_log.duals = solution.row_dual;

	return execution_log;
}

BCExecutionLog solve_bc(MatrixFormulation* formulation, ostream* screen_output, Duration time_limit,
						const json& config, const vector<Valuation>& initial_solutions,
						const vector<BranchPriority>& branch_priorities,
						const SeparationStrategy& separation_strategy, const unordered_set<BCOption>& options)
{
	// The BPC path (FreezeHeuristic) uses none of these; fail loudly rather
	// than silently diverge from the CPLEX backend.
	if (!initial_solutions.empty()) fail("HiGHS backend: initial solutions (MIP starts) are not supported yet.");
	if (!branch_priorities.empty()) fail("HiGHS backend: branch priorities are not supported yet.");
	if (!separation_strategy.Families().empty()) fail("HiGHS backend: separation strategies (cut callbacks) are not supported yet.");
	if (!formulation->LazyConstraints().empty()) fail("HiGHS backend: lazy constraints are not supported yet.");

	BCExecutionLog execution_log;

	Highs solver;
	configure(solver, screen_output, time_limit, config);
	solver.passModel(materialize(formulation, /*relax=*/false));

	Stopwatch rolex(true);
	solver.run();
	rolex.Pause();

	execution_log.time = rolex.Peek();
	execution_log.variable_count = formulation->VariableCount();
	execution_log.constraint_count = formulation->ConstraintCount();
	execution_log.nodes_open = 0;
	execution_log.nodes_closed = (int)solver.getInfo().mip_node_count;
	execution_log.best_bound = solver.getInfo().mip_dual_bound;

	map<HighsModelStatus, BCStatus> status_mapper = {
		{HighsModelStatus::kOptimal, BCStatus::Optimum},
		{HighsModelStatus::kInfeasible, BCStatus::Infeasible},
		{HighsModelStatus::kUnboundedOrInfeasible, BCStatus::Infeasible},
		{HighsModelStatus::kUnbounded, BCStatus::Unbounded},
		{HighsModelStatus::kTimeLimit, BCStatus::TimeLimitReached},
		{HighsModelStatus::kMemoryLimit, BCStatus::MemoryLimitReached},
		{HighsModelStatus::kSolutionLimit, BCStatus::NodeLimitReached}};
	execution_log.status = status_mapper[solver.getModelStatus()];

	const HighsSolution& solution = solver.getSolution();
	if (solution.value_valid && solver.getInfo().primal_solution_status == kSolutionStatusFeasible)
	{
		execution_log.best_int_value = solver.getInfo().objective_function_value;
		if (includes(options, BCOption::BestIntSolution))
			execution_log.best_int_solution = values_to_valuation(formulation, solution.col_value);
	}

	// Root LP value correction mirroring the CPLEX backend.
	if (includes(options, BCOption::RootInformation))
		if (*execution_log.nodes_closed == 0 && *execution_log.status == BCStatus::Optimum)
			execution_log.root_lp_value = *execution_log.best_bound;

	return execution_log;
}
} // namespace highs
} // namespace goc
