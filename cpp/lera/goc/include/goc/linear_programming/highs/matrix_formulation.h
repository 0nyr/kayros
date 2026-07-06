//
// kayros addition (plan 2, Stream 5 M5.1) — NOT part of Lera-Romero's goc.
// Solver-agnostic in-memory Formulation used by the HiGHS backend: the model
// lives entirely in goc data structures; the solver materializes it per solve.
//

#ifndef GOC_LINEAR_PROGRAMMING_HIGHS_MATRIX_FORMULATION_H
#define GOC_LINEAR_PROGRAMMING_HIGHS_MATRIX_FORMULATION_H

#include <iostream>
#include <string>
#include <vector>

#include "goc/linear_programming/model/formulation.h"

namespace goc
{
class MatrixFormulation : public Formulation
{
public:
	MatrixFormulation();
	virtual ~MatrixFormulation();

	virtual int AddConstraint(const Constraint& constraint);
	virtual void RemoveConstraint(int constraint_index);
	virtual void AddLazyConstraint(SeparationRoutine* lazy_constraint);
	virtual void RemoveLazyConstraint(SeparationRoutine* lazy_constraint);
	virtual Variable AddVariable(const std::string& name, VariableDomain domain=VariableDomain::Real, double lower_bound=-INFTY, double upper_bound=INFTY);
	virtual void RemoveVariable(const Variable& variable);
	virtual void SetVariableDomain(const Variable& variable, VariableDomain domain);
	virtual void SetVariableBound(const Variable& v, double lower_bound, double upper_bound);
	virtual void SetVariableLowerBound(const Variable& v, double lower_bound);
	virtual void SetVariableUpperBound(const Variable& v, double upper_bound);
	virtual void Minimize(const Expression& objective_function);
	virtual void Maximize(const Expression& objective_function);
	virtual void SetConstraintRightHandSide(int constraint_index, double value);
	virtual void SetConstraintCoefficient(int constraint_index, const Variable& variable, double coefficient);
	virtual void SetObjectiveCoefficient(const Variable& variable, double coefficient);
	virtual ObjectiveSense GetObjectiveSense() const;
	virtual double GetObjectiveCoefficient(const Variable& variable) const;
	virtual double GetConstraintRightHandSide(int constraint_index) const;
	virtual double GetConstraintCoefficient(int constraint_index, const Variable& variable);
	virtual VariableDomain GetVariableDomain(const Variable& variable) const;
	virtual std::pair<double, double> GetVariableBound(const Variable& variable) const;
	virtual Expression ObjectiveFunction() const;
	virtual std::vector<Variable> Variables() const;
	virtual std::vector<Constraint> Constraints() const;
	virtual const std::vector<SeparationRoutine*>& LazyConstraints() const;
	virtual int VariableCount() const;
	virtual int ConstraintCount() const;
	virtual Variable VariableAtIndex(int variable_index) const;
	virtual double EvaluateValuation(const Valuation& v) const;
	virtual bool IsFeasibleValuation(const Valuation& v, bool verbose=false) const;
	virtual Formulation* Copy() const;
	virtual void Print(std::ostream& os) const;

private:
	// Rebuilds constraints_[constraint_index] with the given left side, keeping sense and rhs.
	void ReplaceConstraint(int constraint_index, const Expression& left, double rhs);

	ObjectiveSense objective_sense_;
	Expression objective_;
	std::vector<Constraint> constraints_;
	std::vector<std::string> variable_names_;
	std::vector<VariableDomain> variable_domains_;
	std::vector<std::pair<double, double>> variable_bounds_;
	std::vector<int*> variable_indices_; // heap ints referenced by Variable objects (kept in sync on removal).
	std::vector<SeparationRoutine*> lazy_constraints_;
};
} // namespace goc

#endif //GOC_LINEAR_PROGRAMMING_HIGHS_MATRIX_FORMULATION_H
