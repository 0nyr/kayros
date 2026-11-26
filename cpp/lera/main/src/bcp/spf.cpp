//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "bcp/spf.h"

#include "bcp/pricing_problem.h"

#include <algorithm>
#include <stdexcept>

using namespace std;
using namespace goc;

namespace solver
{
SPF::SPF(int n) : n(n)
{
	formulation = LPSolver::NewFormulation();
	formulation->Minimize(Expression()); // Minimization.
	formulation->AddConstraint(Expression().EQ(0.0)); // Start depot.
	for (int i = 1; i < n-1; ++i) formulation->AddConstraint(Expression().EQ(1.0)); // Customers.
	formulation->AddConstraint(Expression().EQ(0.0)); // End depot.
	
	// Init structures.
	omega_by_arc = Matrix<vector<int>>(n, n);
	forbidden_arcs = {};
}

SPF::~SPF()
{
	delete formulation;
}

void SPF::AddRoute(Route r, const ExactCost& c)
{
	// Add route r to Omega.
	int j = omega.size();
	r.duration = nearest_binary64(c);
	omega.push_back(r);
	cost.push_back(c);
	artificial.push_back(false);
	
	// Add variable y_j to the formulation.
	Variable y_j = formulation->AddVariable("y_" + STR(j), VariableDomain::Binary, 0.0, INFTY);
	y.push_back(y_j);
	
	// Set coefficient 1.0 in vertices visited.
	for (int k = 1; k < (int)r.path.size()-1; ++k) formulation->SetConstraintCoefficient(r.path[k], y_j, 1.0);
	
	// Set duration(r) as c_j in the objective function.
	formulation->SetObjectiveCoefficient(y_j, r.duration);
	
	// Add the route to the structure indexed by arcs.
	for (int k = 0; k < (int)r.path.size()-1; ++k) omega_by_arc[r.path[k]][r.path[k+1]].push_back(j);
	
	// Update cuts coefficients.
	for (int i = 0; i < cuts.size(); ++i)
		if (sum<Vertex>(r.path, [&] (Vertex v) { return cuts[i].test(v) ? 1 : 0; }) >= 2)
			formulation->SetConstraintCoefficient(n+i, y[j], 1.0);
}

void SPF::AddArtificial(Vertex i, const ExactCost& c)
{
	if (ExactCost(nearest_binary64(c)) != c) throw std::invalid_argument("the cost of an artificial column must be a binary64 value");
	int j = omega.size();
	// The path only records the customer. The column is not indexed by arc, so
	// SetForbiddenArcs never fixes it, and it visits 1 customer of any cut.
	omega.push_back(Route({0, i, n-1}, 0.0, nearest_binary64(c)));
	cost.push_back(c);
	artificial.push_back(true);
	Variable y_j = formulation->AddVariable("a_" + STR(j), VariableDomain::Binary, 0.0, INFTY);
	y.push_back(y_j);
	formulation->SetConstraintCoefficient(i, y_j, 1.0);
	formulation->SetObjectiveCoefficient(y_j, nearest_binary64(c));
}

void SPF::AddCut(const SubsetRowCut& cut)
{
	int i = cuts.size(); // index of this new cut.
	cuts.push_back(cut);
	formulation->AddConstraint(Expression().LEQ(1.0)); // Add row for cut. floor(n/k) = floor(3/2) = 1.
	
	// Update routes' coefficient that visit at least 2 customers in the cut.
 	for (int j = 0; j < omega.size(); ++j)
		if (sum<Vertex>(omega[j].path, [&] (Vertex v) { return cut.test(v) ? 1 : 0; }) >= 2)
			formulation->SetConstraintCoefficient(n+i, y[j], 1.0);
}

void SPF::SetForbiddenArcs(const vector<Arc>& A)
{
	SetForbiddenArcs(A, {});
}

void SPF::SetForbiddenArcs(const vector<Arc>& A, const vector<Arc>& master_only)
{
	// Restore previously forbidden arcs.
	for (const auto* L: {&forbidden_arcs, &master_only_arcs})
		for (Arc e: *L)
			for (int j: omega_by_arc[e.tail][e.head])
				formulation->SetVariableBound(y[j], 0.0, INFTY);

	// All routes with forbidden arcs must be set to 0.
	for (const auto* L: {&A, &master_only})
		for (Arc e: *L)
			for (int j: omega_by_arc[e.tail][e.head])
				formulation->SetVariableBound(y[j], 0.0, 0.0);

	forbidden_arcs = A;
	master_only_arcs = master_only;
}

kayros::exact_lp::Master SPF::ExactMaster() const
{
	namespace xl = kayros::exact_lp;
	xl::Master M;
	const int rows = n + (int)cuts.size();
	M.sense.assign(rows, xl::Sense::Equal);
	M.rhs.assign(rows, mpq_class(0));
	for (int i = 1; i < n-1; ++i) M.rhs[i] = 1;
	for (int c = 0; c < (int)cuts.size(); ++c) { M.sense[n+c] = xl::Sense::LessEqual; M.rhs[n+c] = 1; }
	vector<bool> fixed(omega.size(), false);
	for (const auto* L: {&forbidden_arcs, &master_only_arcs})
		for (Arc e: *L)
			for (int j: omega_by_arc[e.tail][e.head]) fixed[j] = true;
	M.columns.resize(omega.size());
	for (int j = 0; j < (int)omega.size(); ++j)
	{
		xl::Column& col = M.columns[j];
		col.cost = cost[j];
		col.fixed_at_zero = fixed[j];
		// The coefficients that AddRoute, AddArtificial and AddCut write. They
		// set a coefficient to 1, so a vertex visited twice still gives 1.
		const GraphPath& p = omega[j].path;
		vector<int> rows_of;
		for (int k = 1; k < (int)p.size()-1; ++k) rows_of.push_back(p[k]);
		for (int c = 0; c < (int)cuts.size(); ++c)
		{
			int visits = 0;
			for (Vertex v: p) visits += cuts[c].test(v) ? 1 : 0;
			if (visits >= 2) rows_of.push_back(n+c);
		}
		sort(rows_of.begin(), rows_of.end());
		rows_of.erase(unique(rows_of.begin(), rows_of.end()), rows_of.end());
		for (int r: rows_of) col.entries.push_back({r, 1});
	}
	return M;
}

std::array<int, 3> SPF::CutCustomers(int c) const
{
	std::array<int, 3> out{0, 0, 0};
	int k = 0;
	for (int v = 0; v < n; ++v)
		if (cuts[c].test(v))
		{
			if (k == 3) throw std::logic_error("a subset-row cut with more than 3 customers");
			out[k++] = v;
		}
	if (k != 3) throw std::logic_error("a subset-row cut with fewer than 3 customers");
	return out;
}

const Route& SPF::RouteOf(const Variable& variable) const
{
	return omega[variable.Index()];
}

const ExactCost& SPF::ExactCostOf(const Variable& variable) const
{
	return cost[variable.Index()];
}

bool SPF::IsArtificial(const Variable& variable) const
{
	return artificial[variable.Index()];
}

PricingProblem SPF::InterpretDuals(const vector<double>& duals) const
{
	PricingProblem pp;
	pp.P = vector<double>(duals.begin(), duals.begin()+n);
	pp.A = forbidden_arcs;
	// Only add cuts to the pricing problem with duals != 0.
	for (int i = 0; i < cuts.size(); ++i)
	{
		if (epsilon_different(duals[n+i], 0.0))
		{
			pp.S.push_back(cuts[i]);
			pp.sigma.push_back(duals[n+i]);
		}
	}
	return pp;
}

vector<Route> SPF::InterpretSolution(const Valuation& z) const
{
	vector<Route> solution;
	for (auto& y_value: z) solution.push_back(omega[y_value.first.Index()]);
	return solution;
}
} // namespace