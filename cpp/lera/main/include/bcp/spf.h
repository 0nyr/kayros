//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_SPF_H
#define NETWORKS2019_SPF_H

#include <array>
#include <vector>

#include <goc/goc.h>
#include <nyr/nyr.h>

#include "bcp/exact_cost.h"
#include "exact_lp/exact_lp.h"

namespace solver
{
typedef nyr::VertexSet SubsetRowCut; // We only consider cuts with n = 3, k = 2. So the set of vertices define the cut.
class PricingProblem;

// Represents a set-partitioning formulation for the VRP.
// min c_j y_j															(0)
// s.t.
//		sum_{j \in \Omega} a_ij y_j = 1 	\forall i \in V - {o, d}	(1)
//		y_j >= 0							\forall j \in \Omega		(2)
class SPF
{
public:
	goc::Formulation* formulation; // formulation for the restricted master problem
	std::vector<SubsetRowCut> cuts; // cuts added to the formulation.
	
	// Creates a Set-Partitioning formulation for VRP with n vertices.
	// Assumes start-depot is 0 and end depot is n-1.
	explicit SPF(int n);
	
	// Frees formulation memory.
	~SPF();
	
	// Adds the specified route to the formulation. The master keeps the exact
	// cost of the route and gives its nearest binary64 value to the LP, which
	// also becomes the duration of the stored route.
	void AddRoute(goc::Route r, const ExactCost& cost);
	
	// Adds the artificial column of customer i: coefficient 1 in the row of i
	// and in no cut, exact cost `cost`, which must be a binary64 value. Branching
	// never fixes it, so every node has an LP optimum.
	void AddArtificial(goc::Vertex i, const ExactCost& cost);
	
	// Adds a subset row cut with n = 3, k = 2.
	// \sum_{j \in Omega and #(r_j \cap cut) >= 2} y_j <= 1.0.
	void AddCut(const SubsetRowCut& cut);
	
	//	- Sets y_j = 0 for all j that contains any arc in A.
	void SetForbiddenArcs(const std::vector<goc::Arc>& A);
	
	// Same, and also sets y_j = 0 for every column that uses an arc of
	// master_only. Those arcs fix columns only. They are not handed to the
	// pricing problem (InterpretDuals), whose labeling would add them to its
	// digraph when it restores the arcs of the previous node.
	void SetForbiddenArcs(const std::vector<goc::Arc>& A, const std::vector<goc::Arc>& master_only);
	
	// Returns the restricted master as exact data for kayros::exact_lp, row for
	// row and column for column as the formulation (same indices, so that a
	// basis of the backend is a basis of it). It holds the depot rows "= 0",
	// the customer rows "= 1", the cut rows "<= 1", the exact cost of every
	// column, and fixed_at_zero for every column that the current forbidden arcs fix.
	kayros::exact_lp::Master ExactMaster() const;
	
	// Returns the 3 customers of cut c, in increasing order.
	std::array<int, 3> CutCustomers(int c) const;
	
	// Returns the number of columns.
	int ColumnCount() const { return (int)omega.size(); }
	
	// The route, the exact cost and the artificial mark of column j.
	const goc::Route& RouteAt(int j) const { return omega.at(j); }
	const ExactCost& ExactCostAt(int j) const { return cost.at(j); }
	bool IsArtificialAt(int j) const { return artificial.at(j); }
	
	// Returns: the route associated to the variable.
	const goc::Route& RouteOf(const goc::Variable& variable) const;
	
	// Returns: the exact cost of the column of the variable.
	const ExactCost& ExactCostOf(const goc::Variable& variable) const;
	
	// Returns: if the variable is an artificial column.
	bool IsArtificial(const goc::Variable& variable) const;
	
	// Returns: the pricing problem for the duals.
	PricingProblem InterpretDuals(const std::vector<double>& duals) const;
	
	// Interprets an integer solution of the formulation and returns the routes selected.
	std::vector<goc::Route> InterpretSolution(const goc::Valuation& z) const;
	
private:
	std::vector<goc::Arc> forbidden_arcs; // arcs that are removed from the SPF (i.e. all routes containing one of these arcs are set to 0).
	std::vector<goc::Arc> master_only_arcs; // arcs that fix columns but are not handed to the pricing problem.
	int n; // number of vertices.
	std::vector<goc::Route> omega; // set of routes in the restricted master problem.
	std::vector<ExactCost> cost; // cost[j] = exact cost of column j.
	std::vector<bool> artificial; // artificial[j] = column j is artificial.
	std::vector<goc::Variable> y; // variables associated with routes in omega.
	goc::Matrix<std::vector<int>> omega_by_arc; // omega_by_arc[i][j] = { r \in omega : (i, j) \in path(r) }.
};
} // namespace

#endif //NETWORKS2019_SPF_H
