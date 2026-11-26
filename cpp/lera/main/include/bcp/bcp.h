//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_BP_H
#define NETWORKS2019_BP_H

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "goc/goc.h"
#include "pricing_problem.h"
#include "spf.h"
#include "exact/ledger.h"
#include "bcp/exact_wiring.h"

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

	// Exact mode (M2 design note of session 139, sections 11 and 12). When set,
	// the ledger alone removes nodes, holds the incumbent and decides the
	// status Optimum (Ledger::certified). The search prunes against its U only.
	// When it is null (the bridge could not build it), the search runs as
	// before and its status is never Optimum.
	kayros::exact::Ledger* ledger = nullptr;
	// Result of admitting a route returned by the ledger as a column.
	enum class Admission { Added, Duplicate, Rejected, CostMismatch };
	// Evaluates the search path with the exact column evaluation of the master,
	// compares its exact cost with expected (the cost of the route as the ledger
	// priced it) and adds it as a column only when they are equal and the path
	// is new. Set by the caller in exact mode.
	std::function<Admission(const goc::GraphPath& path, const ExactCost& expected)> admit_route;
	// Why the exact mode ended without claim, the first reason met ("" if none),
	// and the count of each reason.
	std::string unresolved_reason;
	std::map<std::string, int> unresolved_counts;
	// 1 entry per run of the node loop of section 3, at processing or at a pop.
	struct ExactNodeVisit {
		int ledger_id = -1;
		bool at_pop = false;       // the loop started from the multipliers stored at processing
		int rounds = 0;            // calls of Ledger::close
		int negative_rounds = 0;   // closes that returned exactly negative routes
		int columns_added = 0;     // routes of the ledger added as columns
		int pivots = 0;            // exact pivots, all rounds
		double exact_lp_s = 0, exact_pass_s = 0;
		std::string outcome;       // removed, deferred, open, no_incumbent, or the reason of a stop
	};
	std::vector<ExactNodeVisit> exact_visits;
	double exact_lp_time = 0, exact_pass_time = 0;
	int ledger_offers = 0, ledger_offers_accepted = 0, ledger_removed_by_close = 0, ledger_removed_by_prune = 0;
	int ledger_branches = 0;
	// Lazy close (S139-D11). The exact pass runs only at a node that would be
	// removed, i.e. whose exact LP value reaches U. passes_run counts the calls of
	// Ledger::close, passes_avoided the visits that ended with an incumbent and a
	// value below U without a pass, closes_at_pop the pops whose stored value
	// reached U, and closes_at_processing every other close. pop_retries counts
	// the column generations run again after a close at a pop returned negative
	// routes, and pop_stale_columns the negative routes of such a close that were
	// already columns of the master (added at other nodes since).
	int passes_run = 0, passes_avoided = 0, closes_at_processing = 0, closes_at_pop = 0;
	int pop_retries = 0, pop_stale_columns = 0;
	// Arcs that the ledger forbids at a child and that the search had not
	// forbidden (added to the search's own set), and arcs absent from the
	// search digraph that only fix columns of the master.
	int ledger_arcs_added_to_search = 0, ledger_arcs_master_only = 0;

	// Offers routes given by the caller (customers only, MAMUT numbering) to
	// the ledger and sets the incumbent of the search from its U. Exact mode.
	// Returns true when the ledger accepted them.
	bool OfferInitialRoutes(const std::vector<std::vector<int>>& routes);

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
		int ledger_id = -1; // id of the node in the ledger (exact mode).
		std::vector<goc::Arc> A_master; // forbidden by the ledger, absent from D: fix columns only.
		bool ledger_bound = false; // a close of the ledger finished on the node, so it holds a bound for prune.
		// The multipliers of the last exact LP solve of the node and their value
		// (sum of pi and sigma), kept for a close at its pop when U drops to it.
		std::optional<ExactCost> exact_bound;
		wiring::Multipliers exact_mult;
		
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
	
	// Exact mode. A node is either removed by the ledger, or still open in the
	// ledger (to be queued or branched), or the run stops without claim.
	enum class Exit { Removed, Open, Stop };
	// Column generation at the node (forbidden arcs set first), then the node
	// loop of section 3 while it is CG optimal, then a prune.
	Exit SolveNodeExact(Node* node);
	// The node loop of section 3 on a node whose column generation is optimal.
	// With at_pop, the first round closes with the multipliers stored on the
	// node, whose value reaches U, without column generation or exact LP.
	Exit ExactLoop(Node* node, goc::CGExecutionLog& cg_log, bool at_pop = false);
	// Offers the columns (indices of the SPF) as a solution to the ledger, and
	// sets the incumbent of the search from its U when it accepts.
	bool OfferColumns(const std::vector<int>& columns, double native_value, const std::string& origin);
	// Takes the incumbent of the search from the ledger.
	void SyncIncumbent(const std::vector<int>& columns, const std::string& origin);
	// Ends the exact mode without claim for the given reason.
	void Unresolve(const std::string& reason);
	bool freeze_done = false;
	bool root_logged = false;

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
