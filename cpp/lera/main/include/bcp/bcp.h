//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_BP_H
#define NETWORKS2019_BP_H

#include <functional>
#include <optional>
#include <string>

#include "goc/goc.h"
#include "pricing_problem.h"
#include "spf.h"

namespace solver
{
typedef std::function<void(const PricingProblem& pricing_problem, int node_number, goc::Duration time_limit, goc::CGExecutionLog* cg_execution_log)> BCPPricingFunction;

// Callback invoked whenever the BCP finds a new incumbent (upper bound).
// elapsed is measured on the BCP's own stopwatch (starts at Run()).
typedef std::function<void(goc::Duration elapsed, const goc::VRPSolution& incumbent, const std::string& origin)> BCPIncumbentCallback;

// This class represents a branch cut and price algorithm. It is a one use object.
class BCP
{
public:
	int cg_optimum_count = 0, cg_certified_count = 0, cg_unresolved_count = 0;
    int cg_infeasible_count = 0, strong_rmp_infeasible_count = 0, unbranched_fractional_count = 0, selected_rmp_infeasible_count = 0;
	// Integer solutions of the master offered as incumbents (integer nodes and
	// the freeze heuristic) that select an artificial column, that the exact
	// evaluation rejects (customers not visited exactly once, fleet limit
	// exceeded), and that it accepts at a value other than the LP or MIP value.
	int artificial_candidate_count = 0, rejected_candidate_count = 0, repriced_candidate_count = 0;
	// Maximum number of routes of a solution, or -1 when the fleet is unbounded.
	// The master has no fleet row, so only the exact evaluation enforces it.
	int fleet_limit = -1;
	goc::Duration time_limit;
	// Absolute deadline derived from time_limit at Run(). All internal
	// components and the external pricing_solver take their residual budgets
	// from here (single source, no per-level stopwatch re-derivation).
	goc::Deadline deadline;
	int node_limit;
	int cut_limit;
	BCPPricingFunction pricing_solver;
	std::function<void(goc::Duration, double, const std::string&)> on_bound;
	BCPIncumbentCallback on_incumbent; // optional; called on every new UB.

	// Initializes the Branch cut and price solver.
	// 	D: digraph that the VRP is based on.
	//	spf: set-partitioning formulation to use (it must include an initial solution).
	BCP(const goc::Digraph& D, SPF* spf);
	
	// Sets an initial upper bound known by the caller: the exact cost of a
	// solution whose columns are in the SPF (e.g. the warm-start columns), used
	// for pruning from the start through its nearest binary64 value. Call before
	// Run(). No incumbent callback fires for it, and no solution routes are
	// attached: if the BCP proves optimality without improving on it, Run()
	// reports the value with an empty route set and the caller keeps its own
	// solution.
	void SetInitialIncumbent(const ExactCost& value);
	
	// Returns: the exact cost of the incumbent, when there is one. The binary64
	// upper bound of the search is its nearest binary64 value.
	const std::optional<ExactCost>& ExactIncumbentValue() const { return z_ub_exact; }

	// Cut separation exposed to the pricing driver, so that subset-row
	// cuts can be separated on the current restricted-master solution before
	// an Exact pricing call. Cuts are valid inequalities whatever the master's
	// convergence state, so calling this early never affects soundness: node
	// optimality is still only ever concluded by an Exact pricing pass that
	// finishes without a new column, followed by a separation round that
	// finds no violated cut.
	// CanSeparateCuts: root node being processed and cut budget left.
	// SeparateCutRounds: re-solves the master LP and adds the most violated
	// cut, until none is violated or the cut limit is hit. Returns the number
	// of cuts added.
	bool CanSeparateCuts() const;
	int SeparateCutRounds();

	// Executes a Branch-Cut-Price algorithm on the SPF.
	// If a solution is found, *solution is filled with the best one.
	goc::BCPExecutionLog Run(goc::VRPSolution* solution);
	
private:
	struct Node
	{
		int index;
		double bound;
		std::vector<goc::Arc> A; // forbidden arcs.
		bool rmp_infeasible = false;
		goc::Valuation opt; // relaxation optimum.
		
		struct Comparator { inline bool operator() (Node* n1, Node* n2) { return n1->bound > n2->bound; } };
	};
	
	// Returns: an estimate of the node's bound.
	// If the node is infeasible or unbounded it returns INFTY.
	double EstimateBound(Node* node);
	
	// Solves the node relaxation using CG and sets its bound and opt attributes.
	// Adds it to the queue if it is feasible and fractional.
	void ProcessNode(Node* node);

	// Branches the node using strong branching.
	void BranchNode(Node* node);

	// The freeze heuristic consists in solving the SPF with the existing columns using a BC solver.
	// The best solution there is an UB to the problem.
	void FreezeHeuristic();
	
	// Evaluates an integer solution of the master exactly, from the exact costs
	// of its columns, and makes it the incumbent when it is a solution whose
	// exact cost is strictly below the exact incumbent cost. native_value is
	// the LP or MIP value, kept for the count of re-priced candidates.
	// Returns: if the solution became the incumbent.
	bool AcceptIncumbent(const goc::Valuation& z, double native_value, const std::string& origin);
	
	// Separates subset row cuts with n = 3, k = 2.
	// Returns: if any cut was added.
	bool SeparateCuts(const goc::Valuation& z);
	
	std::priority_queue<Node*, std::vector<Node*>, Node::Comparator> q; // queue of nodes in the BB tree.
	double z_lb, z_ub; // z_lb = value of the worst open node. z_ub = nearest binary64 value of z_ub_exact.
	std::optional<ExactCost> z_ub_exact; // exact cost of the best int solution.
	goc::Valuation ub; // Best int solution found so far.
	int node_seq; // number of nodes created.
	goc::Stopwatch rolex; // Stopwatch to measure the time spent in the algorithm.
	
	goc::Digraph D;
	SPF* spf;
	goc::LPSolver lp_solver;
	goc::CGSolver cg_solver;
	goc::BCPExecutionLog log;
};
} // namespace

#endif //NETWORKS2019_BP_H
