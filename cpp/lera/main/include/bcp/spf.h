//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_SPF_H
#define NETWORKS2019_SPF_H

#include <vector>

#include <goc/goc.h>
#include <nyr/nyr.h>

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
	
	// Adds the specified route to the formulation (using the duration as its cost).
	void AddRoute(const goc::Route& r);
	
	// Adds a subset row cut with n = 3, k = 2.
	// \sum_{j \in Omega and #(r_j \cap cut) >= 2} y_j <= 1.0.
	void AddCut(const SubsetRowCut& cut);
	
	//	- Sets y_j = 0 for all j that contains any arc in A.
	void SetForbiddenArcs(const std::vector<goc::Arc>& A);
	
	// Returns: the route associated to the variable.
	const goc::Route& RouteOf(const goc::Variable& variable) const;
	
	// Returns: the pricing problem for the duals.
	PricingProblem InterpretDuals(const std::vector<double>& duals) const;
	
	// Interprets an integer solution of the formulation and returns the routes selected.
	std::vector<goc::Route> InterpretSolution(const goc::Valuation& z) const;
	
private:
	std::vector<goc::Arc> forbidden_arcs; // arcs that are removed from the SPF (i.e. all routes containing one of these arcs are set to 0).
	int n; // number of vertices.
	std::vector<goc::Route> omega; // set of routes in the restricted master problem.
	std::vector<goc::Variable> y; // variables associated with routes in omega.
	goc::Matrix<std::vector<int>> omega_by_arc; // omega_by_arc[i][j] = { r \in omega : (i, j) \in path(r) }.
};
} // namespace

#endif //NETWORKS2019_SPF_H
