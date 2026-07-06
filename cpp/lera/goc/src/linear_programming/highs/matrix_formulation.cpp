//
// kayros addition (plan 2, Stream 5 M5.1) — NOT part of Lera-Romero's goc.
//

#include "goc/linear_programming/highs/matrix_formulation.h"

#include "goc/collection/collection_utils.h"
#include "goc/exception/exception_utils.h"
#include "goc/math/number_utils.h"

using namespace std;

namespace goc
{
MatrixFormulation::MatrixFormulation() : objective_sense_(Minimization)
{
}

MatrixFormulation::~MatrixFormulation()
{
	for (int* index: variable_indices_) delete index;
}

int MatrixFormulation::AddConstraint(const Constraint& constraint)
{
	constraints_.push_back(constraint);
	return (int)constraints_.size() - 1;
}

void MatrixFormulation::RemoveConstraint(int constraint_index)
{
	if (constraint_index < 0 || constraint_index >= (int)constraints_.size()) return;
	constraints_.erase(constraints_.begin() + constraint_index);
}

void MatrixFormulation::AddLazyConstraint(SeparationRoutine* lazy_constraint)
{
	if (!lazy_constraint) return;
	lazy_constraints_.push_back(lazy_constraint);
}

void MatrixFormulation::RemoveLazyConstraint(SeparationRoutine* lazy_constraint)
{
	if (!lazy_constraint) return;
	for (size_t i = 0; i < lazy_constraints_.size(); ++i)
	{
		if (lazy_constraints_[i] == lazy_constraint)
		{
			lazy_constraints_.erase(lazy_constraints_.begin() + i);
			break;
		}
	}
}

Variable MatrixFormulation::AddVariable(const string& name, VariableDomain domain, double lower_bound, double upper_bound)
{
	variable_names_.push_back(name);
	variable_domains_.push_back(domain);
	variable_bounds_.push_back({lower_bound, upper_bound});
	variable_indices_.push_back(new int((int)variable_indices_.size()));
	return Variable(name, variable_indices_.back());
}

void MatrixFormulation::RemoveVariable(const Variable& variable)
{
	int i = variable.Index();
	if (i < 0 || i >= (int)variable_indices_.size()) return;
	delete variable_indices_[i];
	variable_indices_.erase(variable_indices_.begin() + i);
	variable_names_.erase(variable_names_.begin() + i);
	variable_domains_.erase(variable_domains_.begin() + i);
	variable_bounds_.erase(variable_bounds_.begin() + i);
	for (int j = i; j < (int)variable_indices_.size(); ++j) *variable_indices_[j] = j;
}

void MatrixFormulation::SetVariableDomain(const Variable& variable, VariableDomain domain)
{
	variable_domains_[variable.Index()] = domain;
}

void MatrixFormulation::SetVariableBound(const Variable& v, double lower_bound, double upper_bound)
{
	variable_bounds_[v.Index()] = {lower_bound, upper_bound};
}

void MatrixFormulation::SetVariableLowerBound(const Variable& v, double lower_bound)
{
	variable_bounds_[v.Index()].first = lower_bound;
}

void MatrixFormulation::SetVariableUpperBound(const Variable& v, double upper_bound)
{
	variable_bounds_[v.Index()].second = upper_bound;
}

void MatrixFormulation::Minimize(const Expression& objective_function)
{
	objective_sense_ = Minimization;
	objective_ = objective_function;
}

void MatrixFormulation::Maximize(const Expression& objective_function)
{
	objective_sense_ = Maximization;
	objective_ = objective_function;
}

void MatrixFormulation::SetConstraintRightHandSide(int constraint_index, double value)
{
	ReplaceConstraint(constraint_index, constraints_[constraint_index].LeftSide(), value);
}

void MatrixFormulation::SetConstraintCoefficient(int constraint_index, const Variable& variable, double coefficient)
{
	Expression left = constraints_[constraint_index].LeftSide();
	left.SetVariableCoefficient(variable, coefficient);
	ReplaceConstraint(constraint_index, left, constraints_[constraint_index].RightSide());
}

void MatrixFormulation::SetObjectiveCoefficient(const Variable& variable, double coefficient)
{
	objective_.SetVariableCoefficient(variable, coefficient);
}

Formulation::ObjectiveSense MatrixFormulation::GetObjectiveSense() const
{
	return objective_sense_;
}

double MatrixFormulation::GetObjectiveCoefficient(const Variable& variable) const
{
	return objective_.VariableCoefficient(variable);
}

double MatrixFormulation::GetConstraintRightHandSide(int constraint_index) const
{
	return constraints_[constraint_index].RightSide();
}

double MatrixFormulation::GetConstraintCoefficient(int constraint_index, const Variable& variable)
{
	return constraints_[constraint_index].LeftSide().VariableCoefficient(variable);
}

VariableDomain MatrixFormulation::GetVariableDomain(const Variable& variable) const
{
	return variable_domains_[variable.Index()];
}

pair<double, double> MatrixFormulation::GetVariableBound(const Variable& variable) const
{
	return variable_bounds_[variable.Index()];
}

Expression MatrixFormulation::ObjectiveFunction() const
{
	return objective_;
}

vector<Variable> MatrixFormulation::Variables() const
{
	vector<Variable> variables;
	for (int i = 0; i < (int)variable_indices_.size(); ++i) variables.push_back(VariableAtIndex(i));
	return variables;
}

vector<Constraint> MatrixFormulation::Constraints() const
{
	return constraints_;
}

const vector<SeparationRoutine*>& MatrixFormulation::LazyConstraints() const
{
	return lazy_constraints_;
}

int MatrixFormulation::VariableCount() const
{
	return (int)variable_indices_.size();
}

int MatrixFormulation::ConstraintCount() const
{
	return (int)constraints_.size();
}

Variable MatrixFormulation::VariableAtIndex(int variable_index) const
{
	return Variable(variable_names_[variable_index], variable_indices_[variable_index]);
}

double MatrixFormulation::EvaluateValuation(const Valuation& v) const
{
	double value = objective_.Scalar();
	for (auto& term: objective_.Terms()) value += term.second * v[term.first];
	return value;
}

bool MatrixFormulation::IsFeasibleValuation(const Valuation& v, bool verbose) const
{
	for (auto& constraint: constraints_)
	{
		if (!constraint.Holds(v))
		{
			if (verbose) clog << "Violated constraint: " << constraint << endl;
			return false;
		}
	}
	return true;
}

Formulation* MatrixFormulation::Copy() const
{
	auto* copy = new MatrixFormulation();
	copy->objective_sense_ = objective_sense_;
	copy->objective_ = objective_;
	copy->constraints_ = constraints_;
	copy->variable_names_ = variable_names_;
	copy->variable_domains_ = variable_domains_;
	copy->variable_bounds_ = variable_bounds_;
	copy->lazy_constraints_ = lazy_constraints_;
	for (int i = 0; i < (int)variable_indices_.size(); ++i) copy->variable_indices_.push_back(new int(i));
	return copy;
}

void MatrixFormulation::Print(ostream& os) const
{
	os << (objective_sense_ == Minimization ? "min " : "max ") << objective_ << endl;
	for (auto& constraint: constraints_) os << constraint << endl;
}

void MatrixFormulation::ReplaceConstraint(int constraint_index, const Expression& left, double rhs)
{
	Constraint replacement = constraints_[constraint_index].Sense() == Constraint::LessEqual
		? left.LEQ(Expression(rhs))
		: constraints_[constraint_index].Sense() == Constraint::GreaterEqual
			? left.GEQ(Expression(rhs))
			: left.EQ(Expression(rhs));
	constraints_[constraint_index] = replacement;
}
} // namespace goc
