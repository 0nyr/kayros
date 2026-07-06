//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "bcp/bcp.h"

#include <climits>

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
			int cuts_added = 0;
			while (this->spf->cuts.size() < cut_limit)
			{
				if (deadline.Reached()) break; // (M5.2)
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
			if (cuts_added > 0) clog << "\tCuts: " << cuts_added << endl;
		}
	};
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
	
	// Start algorithm. (M5.2) One absolute deadline for every component below.
	deadline = Deadline::In(time_limit);
	rolex.Resume();
	
	// Create root node and solve.
	clog << "Processing root node." << endl;
	Node* root = new Node{0, INFTY, {}};
	ProcessNode(root);
	log.root_time = rolex.Peek();

	if (!q.empty())
	{
		clog << "Solving BC with existing columns to get an UB." << endl;
		FreezeHeuristic();
		
		clog << "Branching." << endl;
		
		TableStream tstream(&clog, 1.0);
		tstream.AddColumn("time", 10).AddColumn("#closed", 10).AddColumn("#open", 10).AddColumn("LB", 10).AddColumn("UB", 10).AddColumn("#cols", 10);
		tstream.WriteHeader();
		
		while (!q.empty())
		{
			if (log.status == BCStatus::TimeLimitReached || log.status == BCStatus::MemoryLimitReached) break;
			if (deadline.Reached()) { log.status = BCStatus::TimeLimitReached; break; } // (M5.2)
			if (log.nodes_closed >= node_limit) { log.status = BCStatus::NodeLimitReached; break; }
			
			// Pop node.
			Node* bbnode = q.top();
			q.pop();
			log.nodes_open--;
			log.nodes_closed++;
			
			if (epsilon_bigger(z_ub, bbnode->bound))
			{
				z_lb = bbnode->bound; // Update z_lb here, because we use Best Bound selection.
				BranchNode(bbnode);
				
				// Output to console.
				if (tstream.RegisterAttempt()) tstream.WriteRow({STR(rolex.Peek()), STR(log.nodes_closed), STR(log.nodes_open), STR(z_lb), STR(z_ub), STR(spf->formulation->VariableCount())});
			}
			delete bbnode;
		}
		// (M5.2) Only close the gap when the tree was fully explored: a branch
		// abandoned on the deadline (aborted strong branching drops children)
		// can empty the queue without proving optimality.
		if (q.empty() && log.status == BCStatus::DidNotStart) z_lb = z_ub;
		
		tstream.WriteRow({STR(rolex.Peek()), STR(log.nodes_closed), STR(log.nodes_open), STR(z_lb), STR(z_ub), STR(spf->formulation->VariableCount())});
		
		// Delete nodes that were not processed.
		while (!q.empty()) { delete q.top(); q.pop(); }
	}

	// If log was not changed to some error (time limit, memory limit, ...) then optimum was found.
	if (log.status == BCStatus::DidNotStart)
	{
		log.status = BCStatus::Optimum;
		z_lb = z_ub;
	}
	log.time = rolex.Peek();
	
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
	spf->SetForbiddenArcs(node->A);
	lp_solver.time_limit = deadline.Remaining(); // (M5.2)
	auto lp_log = lp_solver.Solve(spf->formulation);
	// (M5.2) An aborted probe must poison the run status: dropping both
	// children of a node silently would let Run() claim optimality later.
	if (lp_log.status == LPStatus::TimeLimitReached) log.status = BCStatus::TimeLimitReached;
	return lp_log.status == LPStatus::Optimum ? *lp_log.incumbent_value : INFTY;
}

void BCP::ProcessNode(Node* node)
{
	node_seq++;
	spf->SetForbiddenArcs(node->A);
	cg_solver.screen_output = node->index == 0 ? &clog : nullptr; // Display only root node CG resolution.
	cg_solver.time_limit = deadline.Remaining(); // (M5.2)

	// Perform column generation to solve the node.
	auto cg_log = cg_solver.Solve(spf->formulation, {CGOption::IterationsInformation});
	*log.lp_time += cg_log.lp_time;
	
	// Update node.
	if (cg_log.status == CGStatus::Optimum)
	{
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
		log.nodes_closed++;
		delete node;
	}
	// Node was solved to optimality.
	else
	{
		if (node->opt.IsInteger())
		{
			if (z_ub > node->bound) // Found a new optimum.
			{
				z_ub = node->bound;
				ub = node->opt;
				Duration tsol = rolex.Peek();
				clog << "New UB: " << z_ub << " at node " << node->index << " - time: " << tsol << endl;
				if (on_incumbent) on_incumbent(tsol, goc::VRPSolution(z_ub, spf->InterpretSolution(ub)), "BCP");
			}
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
}

void BCP::BranchNode(Node* node)
{
	Stopwatch rolex_branch(true);
	
	// Calculate z[x_ij] values.
	Matrix<double> x(D.NbVertices(), D.NbVertices(), 0.0);
	for (auto& y_val: node->opt)
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
	
	// Calculate all candidate estimate bounds and keep the best one.
	vector<Node> best_candidate;
	if (!x_most.empty())
	{
		double best_estimate = -INFTY;
		
		for (Arc e: x_most)
		{
			if (deadline.Reached()) { log.status = BCStatus::TimeLimitReached; break; } // (M5.2)
			vector<Arc> A = node->A; // infeasible arcs.
			
			// Left node (x_e = 0).
			A.push_back(e);
			Node left{node_seq+1, INFTY, A};
			left.bound = EstimateBound(&left);
			
			// Right node (x_e = 1).
			A.pop_back();
			for (Vertex j: D.Successors(e.tail)) if (j != e.head) A.push_back({e.tail, j});
			for (Vertex i: D.Predecessors(e.head)) if (i != e.tail) A.push_back({i, e.head});
			Node right{node_seq+2, INFTY, A};
			right.bound = EstimateBound(&right);
			
			// If the candidate is the best so far, keep it.
			if (min(left.bound, right.bound) > best_estimate)
			{
				best_estimate = min(left.bound, right.bound);
				best_candidate = {};
				if (left.bound < INFTY) best_candidate.push_back(left);
				if (right.bound < INFTY) best_candidate.push_back(right);
			}
		}
	}
	
	*log.branching_time += rolex_branch.Pause();
	// Process new children to the queue.
	for (Node& candidate: best_candidate)
	{
		if (log.status == BCStatus::TimeLimitReached || log.status == BCStatus::MemoryLimitReached) break; // (M5.2)
		ProcessNode(new Node(candidate));
	}
}

void BCP::FreezeHeuristic()
{
	if (deadline.Reached()) return; // (M5.2)
	BCSolver bc_solver;
	bc_solver.time_limit = deadline.Remaining(); // (M5.2) residual budget, not the full TL.
	auto bc_log = bc_solver.Solve(spf->formulation, {BCOption::BestIntSolution});
	// (M5.2) A TL-truncated MILP may still carry a feasible incumbent — take it.
	bool has_incumbent = (bc_log.status == BCStatus::Optimum || bc_log.status == BCStatus::TimeLimitReached)
		&& bc_log.best_int_value.IsSet() && bc_log.best_int_solution.IsSet();
	if (has_incumbent && bc_log.best_int_value < z_ub)
	{
		z_ub = bc_log.best_int_value;
		ub = *bc_log.best_int_solution;
		Duration tsol = rolex.Peek();
		clog << "New UB: " << z_ub << " in freeze heuristic - time: " << tsol << endl;
		if (on_incumbent) on_incumbent(tsol, goc::VRPSolution(z_ub, spf->InterpretSolution(ub)), "BCP-freezeH");
	}
}

bool BCP::SeparateCuts(const Valuation& z)
{
	// Parse basis variables.
	vector<VertexSet> z_visited; // z_visited[i] = vertices visited by basis variable i.
	vector<double> z_values; // z_values[i] = value of basis variable i.
	for (auto& y_val: z)
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
		if (deadline.Reached()) return false; // (M5.2) abort separation; CG's own TL check terminates the run.
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