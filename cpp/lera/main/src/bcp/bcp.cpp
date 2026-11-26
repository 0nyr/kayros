//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "bcp/bcp.h"
#include "bcp/branch_trace.h"
#include "bcp/ordered_valuation.h"
#include "bcp/exact_wiring.h"

#include <chrono>
#include <climits>
#include <set>

using namespace std;
using namespace goc;
using namespace nyr;

namespace solver
{
BCP::BCP(
	const Digraph& D, 
	SPF* spf
): 
	z_lb(-INFTY), 
	z_ub(INFTY), 
	node_seq(0),
	D(D), 
	spf(spf)
{
	time_limit = Duration::Max();
	node_limit = cut_limit = INT_MAX;
	
	// Init pricing solver.
	pricing_solver = [] (const PricingProblem&, int, Duration, CGExecutionLog*) { fail("Pricing solver not implemented."); };
	cg_solver.screen_output = &clog;
	cg_solver.lp_solver = &lp_solver;
	cg_solver.pricing_function = [&] (const vector<double>& duals, double incumbent_value, Duration time_limit, CGExecutionLog* cg_execution_log) {
		int variable_count = this->spf->formulation->VariableCount();
		Stopwatch iteration_rolex(true);
		auto pp = this->spf->InterpretDuals(duals);

		// Solve pricing problem.
		pricing_solver(pp, 0, time_limit, cg_execution_log);
		*log.pricing_time += iteration_rolex.Peek();
		
		// If no variable were added and we are in root node, separate cuts.
		if (node_seq == 1 && variable_count == this->spf->formulation->VariableCount())
		{
			int cuts_added = SeparateCutRounds();
			if (cuts_added > 0) clog << "\tCuts: " << cuts_added << endl;
		}
	};
}

bool BCP::CanSeparateCuts() const
{
	return node_seq == 1 && (int) spf->cuts.size() < cut_limit;
}

int BCP::SeparateCutRounds()
{
	int cuts_added = 0;
	while (this->spf->cuts.size() < cut_limit)
	{
		if (deadline.Reached()) break;
		Stopwatch cut_rolex(true);
		lp_solver.time_limit = deadline.Remaining();
		auto lp_log = lp_solver.Solve(this->spf->formulation, {LPOption::Incumbent});
		if (lp_log.status != LPStatus::Optimum) break;
		bool added_cuts = SeparateCuts(lp_log.incumbent);
		log.cut_family_iteration_count->at("SR")++;
		log.cut_family_cut_time->at("SR") += cut_rolex.Peek();
		*log.cut_time += cut_rolex.Peek();
		if (!added_cuts) break;
		log.cut_family_cut_count->at("SR")++;
		log.cut_count++;
		cuts_added++;
	}
	return cuts_added;
}

void BCP::SetInitialIncumbent(const ExactCost& value)
{
	if (z_ub_exact && *z_ub_exact <= value) return;
	z_ub_exact = value;
	z_ub = nearest_binary64(value);
}

bool BCP::AcceptIncumbent(const Valuation& z, double native_value, const string& origin)
{
	ExactCost cost = 0;
	int routes = 0;
	vector<int> visits(D.NbVertices(), 0);
	for (const auto& y_val: z)
	{
		if (y_val.second < 0.5) continue;
		if (spf->IsArtificial(y_val.first)) { ++artificial_candidate_count; return false; }
		cost += spf->ExactCostOf(y_val.first);
		++routes;
		const auto& path = spf->RouteOf(y_val.first).path;
		for (int k = 1; k < (int)path.size()-1; ++k) ++visits[path[k]];
	}
	bool partition = routes > 0;
	for (Vertex v = 1; v < D.NbVertices()-1; ++v) partition &= visits[v] == 1;
	if (!partition || (fleet_limit >= 0 && routes > fleet_limit)) { ++rejected_candidate_count; return false; }
	if (ledger)
	{
		// In exact mode the ledger evaluates the routes itself and decides.
		vector<int> columns;
		for (const auto& y_val: z) if (y_val.second >= 0.5) columns.push_back(y_val.first.Index());
		return OfferColumns(columns, native_value, origin);
	}
	if (z_ub_exact && *z_ub_exact <= cost) return false;
	z_ub_exact = cost;
	z_ub = nearest_binary64(cost);
	if (z_ub != native_value) ++repriced_candidate_count;
	ub = z;
	Duration tsol = rolex.Peek();
	clog << "New UB: " << z_ub << " (" << origin << ") - time: " << tsol << endl;
	if (on_incumbent) on_incumbent(tsol, goc::VRPSolution(z_ub, spf->InterpretSolution(ub)), origin);
	return true;
}

void BCP::Unresolve(const string& reason)
{
	log.status = BCStatus::Unresolved;
	if (unresolved_reason.empty()) unresolved_reason = reason;
	++unresolved_counts[reason];
	clog << "Exact mode: no claim (" << reason << ")" << endl;
}

void BCP::SyncIncumbent(const vector<int>& columns, const string& origin)
{
	z_ub_exact = *ledger->U();
	z_ub = nearest_binary64(*z_ub_exact);
	// The routes of the search, for the incumbent callback. The result takes the
	// routes of the ledger.
	vector<Route> routes;
	for (int j: columns) routes.push_back(spf->RouteAt(j));
	Duration tsol = rolex.Peek();
	clog << "New UB: " << z_ub << " (" << origin << ") - time: " << tsol << endl;
	if (on_incumbent && !columns.empty()) on_incumbent(tsol, goc::VRPSolution(z_ub, routes), origin);
}

bool BCP::OfferColumns(const vector<int>& columns, double native_value, const string& origin)
{
	wiring::VertexMap map{D.NbVertices() - 2};
	vector<vector<int>> routes;
	for (int j: columns)
	{
		if (spf->IsArtificialAt(j)) { ++artificial_candidate_count; return false; }
		routes.push_back(map.customers_of(spf->RouteAt(j).path));
	}
	++ledger_offers;
	const auto o = ledger->offer(routes);
	if (!o.accepted)
	{
		// An equal or larger cost is no rejection. Anything else is a candidate
		// that is not a solution under the exact evaluation.
		if (!o.cost) ++rejected_candidate_count;
		return false;
	}
	++ledger_offers_accepted;
	SyncIncumbent(columns, origin);
	if (z_ub != native_value) ++repriced_candidate_count;
	return true;
}

bool BCP::OfferInitialRoutes(const vector<vector<int>>& routes)
{
	++ledger_offers;
	const auto o = ledger->offer(routes);
	if (!o.accepted) return false;
	++ledger_offers_accepted;
	SyncIncumbent({}, "warm_start");
	return true;
}

BCPExecutionLog BCP::Run(goc::VRPSolution* solution)
{
	// Init variables.
	log.variable_count = spf->formulation->VariableCount();
	log.constraint_count = spf->formulation->ConstraintCount();
	log.time = log.lp_time = log.branching_time = log.pricing_time = 0.0_sec;
	log.status = BCStatus::DidNotStart;
	log.nodes_open = log.nodes_closed = 0;
	log.cut_count = 0;
	log.cut_time = 0.0_sec;
	log.cut_families = {"SR"};
	log.cut_family_cut_count = {{"SR", 0}};
	log.cut_family_cut_time = {{"SR", 0.0_sec}};
	log.cut_family_iteration_count = {{"SR", 0}};
	
	// Start algorithm. One absolute deadline for every component below.
	deadline = Deadline::In(time_limit);
	rolex.Resume();
	
	// Create root node and solve.
	clog << "Processing root node." << endl;
	Node* root = new Node{0, INFTY, {}};
	root->ledger_id = 0; // the root of the ledger
	ProcessNode(root);
	log.root_time = rolex.Peek();
	if (log.root_lp_value.IsSet()) {
		z_lb = *log.root_lp_value;
		if (on_bound) on_bound(rolex.Peek(), z_lb, "root_complete");
	}

	if (!q.empty())
	{
		clog << "Solving BC with existing columns to get an UB." << endl;
		if (!freeze_done) FreezeHeuristic();
		
		clog << "Branching." << endl;
		
		TableStream tstream(&clog, 1.0);
		tstream.AddColumn("time", 10).AddColumn("#closed", 10).AddColumn("#open", 10).AddColumn("LB", 10).AddColumn("UB", 10).AddColumn("#cols", 10);
		tstream.WriteHeader();
		
		while (!q.empty())
		{
			if (log.status == BCStatus::TimeLimitReached || log.status == BCStatus::MemoryLimitReached || log.status == BCStatus::Unresolved) break;
			if (deadline.Reached()) { log.status = BCStatus::TimeLimitReached; break; }
			if (MemoryMonitor::Exceeded()) { log.status = BCStatus::MemoryLimitReached; break; }
			if (log.nodes_closed >= node_limit) { log.status = BCStatus::NodeLimitReached; break; }
			
			// Pop node.
			Node* bbnode = q.top();
			q.pop();
			log.nodes_open--;
			log.nodes_closed++;
			
			if (ledger)
			{
				// In exact mode only the ledger removes the node. A node without a
				// bound in the ledger (no incumbent when it was solved) is solved
				// again now, and a node that stays open is branched.
				bool branch = true;
				if (!ledger->is_open(bbnode->ledger_id)) branch = false;
				else
				{
					const auto pr = ledger->prune(bbnode->ledger_id);
					if (pr.removed) { ++ledger_removed_by_prune; branch = false; }
					else if (!bbnode->ledger_bound && ledger->U())
					{
						++pop_retries;
						const Exit e = SolveNodeExact(bbnode);
						if (e != Exit::Open) branch = false;
					}
				}
				if (branch)
				{
					z_lb = bbnode->bound; // diagnostic only in exact mode
					BranchNode(bbnode);
					if (tstream.RegisterAttempt()) tstream.WriteRow({STR(rolex.Peek()), STR(log.nodes_closed), STR(log.nodes_open), STR(z_lb), STR(z_ub), STR(spf->formulation->VariableCount())});
				}
			}
			else if (epsilon_bigger(z_ub, bbnode->bound))
			{
				z_lb = bbnode->bound; // Update z_lb here, because we use Best Bound selection.
				BranchNode(bbnode);
				
				// Output to console.
				if (tstream.RegisterAttempt()) tstream.WriteRow({STR(rolex.Peek()), STR(log.nodes_closed), STR(log.nodes_open), STR(z_lb), STR(z_ub), STR(spf->formulation->VariableCount())});
			}
			if (log.status == BCStatus::DidNotStart && !q.empty()) {
				z_lb = std::max(z_lb, std::min(z_ub, q.top()->bound));
				if (on_bound) on_bound(rolex.Peek(), z_lb, "frontier");
			}
			delete bbnode;
		}
		// Only close the gap when the tree was fully explored: a branch
		// abandoned on the deadline (aborted strong branching drops children)
		// can empty the queue without proving optimality.
		if (q.empty() && log.status == BCStatus::DidNotStart && ledger && ledger->certified()) z_lb = z_ub;
		
		tstream.WriteRow({STR(rolex.Peek()), STR(log.nodes_closed), STR(log.nodes_open), STR(z_lb), STR(z_ub), STR(spf->formulation->VariableCount())});

		// On an aborted search the global lower bound is the best
		// open node's bound (q is best-bound-first), not the last popped one —
		// report it so the final incumbent's true gap is known. If the TL hit
		// during root CG there is no valid LB and best_bound stays unset.
		// An interrupted active branch can have children absent from q.
		// Retain its inherited bound rather than advancing to the queue alone.
		if (!q.empty() && log.status == BCStatus::DidNotStart)
			z_lb = std::max(z_lb, std::min(z_ub, q.top()->bound));

		// Delete nodes that were not processed.
		while (!q.empty()) { delete q.top(); q.pop(); }
	}

	// If log was not changed to some error (time limit, memory limit, ...) then
	// the search ended. In exact mode the status is Optimum iff the ledger is
	// certified. Without a ledger the run can never be optimal.
	if (log.status == BCStatus::DidNotStart)
	{
		if (!ledger) Unresolve("ledger_unavailable");
		else if (!ledger->certified()) Unresolve(ledger->U() ? "ledger_open_nodes" : "ledger_no_incumbent");
		else
		{
			log.status = BCStatus::Optimum;
			z_lb = z_ub;
		}
	}
	log.time = rolex.Peek();
	if (on_bound) on_bound(log.time, z_lb,
		log.status == BCStatus::Optimum && z_ub < INFTY ? "search_closed" : "search_terminated");
	
	// Set bounds and solutions found.
	if (z_ub != INFTY) log.best_int_value = z_ub;
	if (z_ub != INFTY) log.best_int_solution = ub;
	if (z_lb != -INFTY) log.best_bound = z_lb;

	// If a solution was found, assign it to the result.
	if (solution != nullptr && z_ub != INFTY)
	{
		solution->routes = spf->InterpretSolution(ub);
		solution->value = log.best_int_value;
	}

	log.final_variable_count = spf->formulation->VariableCount();
	log.final_constraint_count = spf->formulation->ConstraintCount();
	
	return log;
}

double BCP::EstimateBound(Node* node)
{
	spf->SetForbiddenArcs(node->A, node->A_master);
	lp_solver.time_limit = deadline.Remaining();
	auto lp_log = lp_solver.Solve(spf->formulation);
	// An aborted probe must poison the run status: dropping both
	// children of a node silently would let Run() claim optimality later.
	node->rmp_infeasible = lp_log.status == LPStatus::Infeasible;
    if (node->rmp_infeasible) ++strong_rmp_infeasible_count;
	if (lp_log.status == LPStatus::TimeLimitReached) log.status = BCStatus::TimeLimitReached;
	if (lp_log.status == LPStatus::MemoryLimitReached) log.status = BCStatus::MemoryLimitReached;
	// Any other non-optimal outcome drops the child below, so it must stop
	// the run as well.
	if (lp_log.status == LPStatus::Unresolved || lp_log.status == LPStatus::DidNotStart) log.status = BCStatus::Unresolved;
	return lp_log.status == LPStatus::Optimum ? *lp_log.incumbent_value : INFTY;
}

BCP::Exit BCP::SolveNodeExact(Node* node)
{
	spf->SetForbiddenArcs(node->A, node->A_master);
	cg_solver.screen_output = node->index == 0 ? &clog : nullptr;
	cg_solver.time_limit = deadline.Remaining();
	auto cg_log = cg_solver.Solve(spf->formulation, {CGOption::IterationsInformation, CGOption::Basis});
	*log.lp_time += cg_log.lp_time;
	if (cg_log.status == CGStatus::Optimum)
	{
		++cg_optimum_count;
		bool certified = cg_log.iterations.IsSet() && !cg_log.iterations->empty()
			&& cg_log.iterations->back().value("ablation_final_pricing_certified", false);
		if (certified) ++cg_certified_count; else ++cg_unresolved_count;
		node->bound = cg_log.incumbent_value;
		node->opt = *cg_log.incumbent;
	}
	if (node->index == 0 && !root_logged)
	{
		root_logged = true;
		log.root_log = cg_log;
		log.root_variable_count = spf->formulation->VariableCount();
		log.root_constraint_count = spf->formulation->ConstraintCount();
		if (cg_log.status == CGStatus::Optimum) log.root_lp_value = cg_log.incumbent_value;
	}
	// Every exit but Optimum leaves the node open in the ledger.
	if (cg_log.status == CGStatus::TimeLimitReached) { log.status = BCStatus::TimeLimitReached; return Exit::Stop; }
	if (cg_log.status == CGStatus::MemoryLimitReached) { log.status = BCStatus::MemoryLimitReached; return Exit::Stop; }
	if (cg_log.status == CGStatus::Unbounded) fail("Node relaxation can not be unbounded.");
	// Not reachable with the artificial columns (design note, section 12).
	if (cg_log.status == CGStatus::Infeasible) { ++cg_infeasible_count; Unresolve("cg_infeasible"); return Exit::Stop; }
	if (cg_log.status != CGStatus::Optimum) { Unresolve("cg_unresolved"); return Exit::Stop; }
	// A binary64 integral optimum is offered first, so that a cold start has an
	// incumbent for the first close.
	if (node->opt.IsInteger()) AcceptIncumbent(node->opt, node->bound, "BCP");
	Exit e = ExactLoop(node, cg_log);
	if (e == Exit::Open && ledger->U() && node->ledger_bound)
	{
		const auto pr = ledger->prune(node->ledger_id);
		if (pr.removed) { ++ledger_removed_by_prune; e = Exit::Removed; }
	}
	return e;
}

BCP::Exit BCP::ExactLoop(Node* node, CGExecutionLog& cg_log)
{
	namespace xl = kayros::exact_lp;
	using clock = std::chrono::steady_clock;
	const int n = D.NbVertices() - 2;
	const wiring::VertexMap map{n};
	ExactNodeVisit visit;
	visit.ledger_id = node->ledger_id;
	auto finish = [&](Exit e, const string& outcome) {
		visit.outcome = outcome;
		exact_visits.push_back(visit);
		return e;
	};
	auto stop = [&](const string& reason) {
		Unresolve(reason);
		return finish(Exit::Stop, reason);
	};
	auto time_stop = [&]() {
		log.status = BCStatus::TimeLimitReached;
		return finish(Exit::Stop, "time_limit");
	};
	while (true)
	{
		if (!ledger->is_open(node->ledger_id)) return finish(Exit::Removed, "removed");
		if (deadline.Reached()) return time_stop();
		if (MemoryMonitor::Exceeded()) { log.status = BCStatus::MemoryLimitReached; return finish(Exit::Stop, "memory_limit"); }
		if (!cg_log.basis.IsSet()) return stop("no_basis");

		// (i) The basis of the last LP solve of the search, solved in rationals.
		const auto t0 = clock::now();
		const xl::Master M = spf->ExactMaster();
		xl::Basis B(cg_log.basis->columns.begin(), cg_log.basis->columns.end());
		for (int i: cg_log.basis->rows) B.push_back(xl::logical(i));
		const xl::Solution sol = xl::solve_from_basis(M, B, xl::default_pivot_budget);
		const double lp_s = std::chrono::duration<double>(clock::now() - t0).count();
		visit.exact_lp_s += lp_s;
		exact_lp_time += lp_s;
		visit.pivots += sol.pivots;
		if (sol.status != xl::Status::Optimal) return stop(string("exact_lp_") + xl::to_string(sol.reason));
		const wiring::Multipliers mult = wiring::multipliers_of(sol, n, (int)spf->cuts.size(),
			[&](int c) { return spf->CutCustomers(c); });

		// An integral exact primal solution is a candidate. Offered before the
		// close, so that the close compares with the lowest U known.
		if (const auto cols = wiring::integral_columns(sol)) OfferColumns(*cols, nearest_binary64(sol.z), "BCP-exact");
		if (!ledger->U() && node->index == 0 && !freeze_done) FreezeHeuristic();
		// Close needs an incumbent. The node stays open, without bound.
		if (!ledger->U()) return finish(Exit::Open, "no_incumbent");

		// (ii) and (iii) The exact pass of the ledger and its verdict.
		const double remaining = deadline.Remaining().Amount(DurationUnit::Seconds);
		if (remaining <= 0) return time_stop();
		const auto t1 = clock::now();
		const auto close = ledger->close(node->ledger_id, mult.cuts, mult.pi, mult.sigma, {0, remaining});
		const double pass_s = std::chrono::duration<double>(clock::now() - t1).count();
		visit.exact_pass_s += pass_s;
		exact_pass_time += pass_s;
		++visit.rounds;
		if (close.removed) { ++ledger_removed_by_close; return finish(Exit::Removed, "removed"); }
		// A close without bound was refused or its pass did not finish.
		// A pass stopped by the time limit given to it is the time limit of the run.
		if (!close.delta) return deadline.Reached() || pass_s >= remaining ? time_stop() : stop("close_refused: " + close.reason);
		node->ledger_bound = true;
		// (v) Without negative route, the exact multipliers are feasible for every
		// route of the node and their value is below U. The caller queues the
		// node, which is branched when it is popped and prune refuses.
		if (close.negative_count == 0) return finish(Exit::Open, "open");

		// (iv) Every negative route enters the master, else no claim.
		++visit.negative_rounds;
		for (const auto& [customers, rc]: close.negatives)
		{
			const GraphPath path = map.search_path(customers);
			for (int k = 0; k + 1 < (int)path.size(); ++k)
				if (!D.IncludesArc({path[k], path[k+1]})) return stop("route_not_representable");
			const ExactCost expected = wiring::cost_from_reduced(customers, rc, mult);
			switch (admit_route(path, expected))
			{
				case Admission::Added: ++visit.columns_added; break;
				case Admission::Duplicate: return stop("negative_route_already_column");
				case Admission::Rejected: return stop("route_rejected_by_column_evaluation");
				case Admission::CostMismatch: return stop("route_cost_mismatch");
			}
		}
		// The search prices again from the master with the new columns.
		cg_solver.screen_output = nullptr;
		cg_solver.time_limit = deadline.Remaining();
		cg_log = cg_solver.Solve(spf->formulation, {CGOption::IterationsInformation, CGOption::Basis});
		*log.lp_time += cg_log.lp_time;
		if (cg_log.status == CGStatus::TimeLimitReached) return time_stop();
		if (cg_log.status == CGStatus::MemoryLimitReached) { log.status = BCStatus::MemoryLimitReached; return finish(Exit::Stop, "memory_limit"); }
		if (cg_log.status == CGStatus::Infeasible) { ++cg_infeasible_count; return stop("cg_infeasible"); }
		if (cg_log.status != CGStatus::Optimum) return stop("cg_unresolved");
		node->bound = cg_log.incumbent_value;
		node->opt = *cg_log.incumbent;
		if (node->opt.IsInteger()) AcceptIncumbent(node->opt, node->bound, "BCP");
	}
}

void BCP::ProcessNode(Node* node)
{
	node_seq++;
	if (ledger)
	{
		const Exit e = SolveNodeExact(node);
		if (e == Exit::Open) { log.nodes_open++; q.push(node); return; }
		if (e == Exit::Removed) log.nodes_closed++;
		delete node; // removed, or the run stops with the node open in the ledger
		return;
	}
	spf->SetForbiddenArcs(node->A);
	cg_solver.screen_output = node->index == 0 ? &clog : nullptr; // Display only root node CG resolution.
	cg_solver.time_limit = deadline.Remaining();

	// Perform column generation to solve the node.
	auto cg_log = cg_solver.Solve(spf->formulation, {CGOption::IterationsInformation});
	*log.lp_time += cg_log.lp_time;
	
	// Update node.
	if (cg_log.status == CGStatus::Optimum)
	{
        ++cg_optimum_count;
        bool certified = cg_log.iterations.IsSet() && !cg_log.iterations->empty()
            && cg_log.iterations->back().value("ablation_final_pricing_certified", false);
        if (certified) ++cg_certified_count; else ++cg_unresolved_count;
		node->bound = cg_log.incumbent_value;
		node->opt = *cg_log.incumbent;
	}
	
	// Log some information if node is root.
	if (node->index == 0)
	{
		log.root_log = cg_log;
		log.root_variable_count = spf->formulation->VariableCount();
		log.root_constraint_count = spf->formulation->ConstraintCount();
		if (cg_log.status == CGStatus::Optimum) log.root_lp_value = cg_log.incumbent_value;
	}
	
	// Node was not solved to optimality, or is infeasible, then do nothing.
	if (cg_log.status == CGStatus::TimeLimitReached)
	{
		log.status = BCStatus::TimeLimitReached;
		delete node;
	}
	else if (cg_log.status == CGStatus::MemoryLimitReached)
	{
		log.status = BCStatus::MemoryLimitReached;
		delete node;
	}
	else if (cg_log.status == CGStatus::Unbounded)
	{
		fail("Node relaxation can not be unbounded.");
	}
	else if (cg_log.status == CGStatus::Infeasible)
	{
        ++cg_infeasible_count;
		log.nodes_closed++;
		delete node;
	}
	// Node was solved to optimality.
	else if (cg_log.status == CGStatus::Optimum)
	{
		if (node->opt.IsInteger())
		{
			// The incumbent value is the exact cost of the routes, not the LP value.
			AcceptIncumbent(node->opt, node->bound, "BCP");
			log.nodes_closed++;
			delete node;
		}
		else if (epsilon_bigger(node->bound, z_ub))
		{
			// Node is feasible but bound is bigger than UB. Cut branch.
			log.nodes_closed++;
			delete node;
		}
		else
		{
			// Node was feasible then add it to the queue.
			log.nodes_open++;
			q.push(node);
		}
	}
	// Any other outcome (Unresolved, or a status the column generation
	// never reached) leaves the node unsolved. Its default relaxation
	// optimum is empty and would pass IsInteger, so it must give neither
	// an incumbent nor a bound. The run stops without claim.
	else
	{
		log.status = BCStatus::Unresolved;
		delete node;
	}
}

void BCP::BranchNode(Node* node)
{
	Stopwatch rolex_branch(true);
	
	// Calculate z[x_ij] values.
	Matrix<double> x(D.NbVertices(), D.NbVertices(), 0.0);
	for (const auto& y_val: ordered_valuation(node->opt))
	{
		auto& r = spf->RouteOf(y_val.first);
		for (int k = 1; k < (int)r.path.size()-2; ++k) x[r.path[k]][r.path[k+1]] += y_val.second;
	}
	
	// Get STRONG_BRANCH_SIZE most violated x_ij (nearest to 0.5).
	int STRONG_BRANCH_SIZE = 10;
	vector<Arc> x_most; // arcs sorted by the violation.
	for (Arc e: D.Arcs())
		if (epsilon_bigger(x[e.tail][e.head], 0.0) && epsilon_smaller(x[e.tail][e.head], 1.0))
			x_most.push_back(e);
	sort(x_most.begin(), x_most.end(), [&] (Arc e, Arc f) { return fabs(0.5-x[e.tail][e.head]) < fabs(0.5-x[f.tail][f.head]); });
	while (x_most.size() > STRONG_BRANCH_SIZE) x_most.pop_back();

    FirstBranchTrace branch_trace;
    if (branch_trace)
    {
        Matrix<double> raw_x(D.NbVertices(), D.NbVertices(), 0.0);
        for (const auto& y_val : node->opt)
        {
            const auto& route = spf->RouteOf(y_val.first);
            for (int k = 1; k < (int) route.path.size() - 2; ++k)
                raw_x[route.path[k]][route.path[k + 1]] += y_val.second;
        }
        vector<Arc> legacy_candidates;
        for (Arc arc : D.Arcs())
            if (epsilon_bigger(raw_x[arc.tail][arc.head], 0.0)
                    && epsilon_smaller(raw_x[arc.tail][arc.head], 1.0))
                legacy_candidates.push_back(arc);
        sort(legacy_candidates.begin(), legacy_candidates.end(), [&](Arc a, Arc b) {
            return fabs(0.5 - raw_x[a.tail][a.head]) < fabs(0.5 - raw_x[b.tail][b.head]);
        });
        while (legacy_candidates.size() > STRONG_BRANCH_SIZE) legacy_candidates.pop_back();
        branch_trace.Begin(node->index, node->bound, x_most, x, legacy_candidates, raw_x, node->opt);
    }
	
	// Calculate all candidate estimate bounds and keep the best one.
	vector<Node> best_candidate;
    int selected_tail = -1, selected_head = -1;
    double selected_estimate = -INFTY;
    int selected_infeasible = 0;
    if (x_most.empty()) ++unbranched_fractional_count;
	// In exact mode the node stays open in the ledger, so the run has no claim.
	if (x_most.empty() && ledger) { Unresolve("no_branching_candidate"); *log.branching_time += rolex_branch.Pause(); return; }
	if (!x_most.empty())
	{
		double best_estimate = -INFTY;
		
		for (Arc e: x_most)
		{
			if (deadline.Reached()) { log.status = BCStatus::TimeLimitReached; break; }
			vector<Arc> A = node->A; // infeasible arcs.
			
			// Left node (x_e = 0).
			A.push_back(e);
			Node left{node_seq+1, INFTY, A};
			left.A_master = node->A_master;
			left.bound = EstimateBound(&left);
			
			// Right node (x_e = 1).
			A.pop_back();
			for (Vertex j: D.Successors(e.tail)) if (j != e.head) A.push_back({e.tail, j});
			for (Vertex i: D.Predecessors(e.head)) if (i != e.tail) A.push_back({i, e.head});
			Node right{node_seq+2, INFTY, A};
			right.A_master = node->A_master;
			right.bound = EstimateBound(&right);
			
			// If the candidate is the best so far, keep it.
			if (min(left.bound, right.bound) > best_estimate)
			{
				best_estimate = min(left.bound, right.bound);
                selected_tail = e.tail;
                selected_head = e.head;
                selected_estimate = best_estimate;
				best_candidate = {};
                selected_infeasible = int(left.rmp_infeasible) + int(right.rmp_infeasible);
				if (left.bound < INFTY) best_candidate.push_back(left);
				if (right.bound < INFTY) best_candidate.push_back(right);
			}
		}
	}
	
    branch_trace.Finish(selected_tail, selected_head, selected_estimate);
    if (log.status == BCStatus::DidNotStart) selected_rmp_infeasible_count += selected_infeasible;
	*log.branching_time += rolex_branch.Pause();
	if (ledger)
	{
		// Exact mode. The probes above are untrusted and never reach the ledger.
		// The ledger branches once on the selected arc, and both children are
		// always solved, whatever the probes said.
		if (log.status != BCStatus::DidNotStart || selected_tail < 0) return;
		const auto b = ledger->branch(node->ledger_id, selected_tail, selected_head);
		if (!b.accepted) { Unresolve("ledger_branch_refused: " + b.reason); return; }
		++ledger_branches;
		const Arc e{selected_tail, selected_head};
		vector<Node> children(2);
		children[0] = Node{node_seq+1, node->bound, node->A};
		children[0].A.push_back(e);
		children[1] = Node{node_seq+2, node->bound, node->A};
		for (Vertex j: D.Successors(e.tail)) if (j != e.head) children[1].A.push_back({e.tail, j});
		for (Vertex i: D.Predecessors(e.head)) if (i != e.tail) children[1].A.push_back({i, e.head});
		children[0].ledger_id = b.left;
		children[1].ledger_id = b.right;
		// The search must forbid at least the arcs of the ledger. Arcs of D that
		// it had not forbidden are added to its own set. Arcs absent from D only
		// fix columns of the master (warm-start columns may use them).
		const wiring::VertexMap map{D.NbVertices() - 2};
		for (Node& c: children)
		{
			const auto F = ledger->forbidden(c.ledger_id);
			if (!F) { Unresolve("ledger_child_unknown"); return; }
			const auto [in_d, outside] = wiring::split_forbidden(*F, map, D);
			set<pair<int, int>> have;
			for (Arc a: c.A) have.insert({a.tail, a.head});
			for (Arc a: in_d)
				if (have.insert({a.tail, a.head}).second) { c.A.push_back(a); ++ledger_arcs_added_to_search; }
			c.A_master.clear();
			for (Arc a: outside) { c.A_master.push_back(a); ++ledger_arcs_master_only; }
		}
		for (Node& c: children)
		{
			if (log.status != BCStatus::DidNotStart) break;
			ProcessNode(new Node(c));
		}
		return;
	}
	// Process new children to the queue.
	for (Node& candidate: best_candidate)
	{
		if (log.status == BCStatus::TimeLimitReached || log.status == BCStatus::MemoryLimitReached || log.status == BCStatus::Unresolved) break;
		ProcessNode(new Node(candidate));
	}
}

void BCP::FreezeHeuristic()
{
	if (deadline.Reached()) return;
	freeze_done = true;
	BCSolver bc_solver;
	bc_solver.time_limit = deadline.Remaining(); // residual budget, not the full TL.
	auto bc_log = bc_solver.Solve(spf->formulation, {BCOption::BestIntSolution});
	// A TL-truncated MILP may still carry a feasible incumbent — take it.
	bool has_incumbent = (bc_log.status == BCStatus::Optimum || bc_log.status == BCStatus::TimeLimitReached)
		&& bc_log.best_int_value.IsSet() && bc_log.best_int_solution.IsSet();
	if (has_incumbent) AcceptIncumbent(*bc_log.best_int_solution, *bc_log.best_int_value, "BCP-freezeH");
}

bool BCP::SeparateCuts(const Valuation& z)
{
	// Parse basis variables.
	vector<VertexSet> z_visited; // z_visited[i] = vertices visited by basis variable i.
	vector<double> z_values; // z_values[i] = value of basis variable i.
	for (const auto& y_val: ordered_valuation(z))
	{
		auto route = spf->RouteOf(y_val.first);
		z_visited.push_back(create_bitset<MAX_N>(route.path));
		z_values.push_back(y_val.second);
	}
	
	// Brute force enumeration of all cuts, check the most violated.
	double best_violation = 0.0;
	SubsetRowCut best;
	for (Vertex i = 1; i < D.NbVertices()-1; ++i)
	{
		if (deadline.Reached()) return false; // abort separation; CG's own TL check terminates the run.
		for (Vertex j = i + 1; j < D.NbVertices() - 1; ++j)
		{
			for (Vertex k = j + 1; k < D.NbVertices() - 1; ++k)
			{
				double violation = -1.0;
				for (int r = 0; r < z_visited.size(); ++r)
					if ((int)z_visited[r].test(i) + (int)z_visited[r].test(j) + (int)z_visited[r].test(k) >= 2)
						violation += z_values[r];
				if (epsilon_bigger(violation, best_violation))
				{
					best_violation = violation;
					best = create_bitset<MAX_N>({i,j,k});
				}
			}
		}
	}
	
	// Add cut if found.
	if (epsilon_bigger(best_violation, 0.1)) spf->AddCut(best);
	return epsilon_bigger(best_violation, 0.1);
}
} // namespace
