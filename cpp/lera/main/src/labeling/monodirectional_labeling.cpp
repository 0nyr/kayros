//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "labeling/monodirectional_labeling.h"

#include <climits>

#include "labeling/pwl_domination_function.h"

using namespace std;
using namespace goc;
using namespace nyr;

namespace solver
{
namespace
{
// Definition of Alpha from Section 5.2.
double alpha(Label* l, bool partial)
{
	if (partial) return l->min_cost;
	else return -(l->rw.right-l->duration(l->rw.right))-l->p-l->cut_cost;
}

// Definition of Beta from Section 5.2.
double beta(Label* l, bool partial)
{
	if (partial) return max(img(l->duration))-l->p-l->cut_cost;
	else return -(l->rw.right-l->duration(l->rw.right))-l->p-l->cut_cost;
}
}

MonodirectionalLabeling::MonodirectionalLabeling(
	const VRPInstance& vrp,
	const std::optional<TDNGRoutesParams>& ng_params
): 
	vrp_(vrp), 
	correcting(false),
	// Create neignborhoods if ng_params are given.
	ng(ng_params ? std::make_optional<TDNGNeighborhoods>(vrp, ng_params->partitioned_horizon, ng_params->nb_neighbors_to_keep) : std::nullopt)
{
	cross = true;
	process_limit = INT_MAX;
	time_limit = 2.0_hr;
	partial = limited_extension = lazy_extension = unreachable_strengthened = sort_by_cost = true;
	elementary_check_relaxation = cost_check_relaxation = ng_routes_relaxation = false;
	processed_count = 0;
	
	t_m = vrp.T;
	U = vector<DemandLevel>(vrp.D.NbVertices());
	
	// no-label is a label that represents the empty path.
	no_label.parent = nullptr;
	no_label.p = no_label.q = no_label.min_cost = 0.0;
	no_label.duration = vrp.tau[vrp.o][vrp.o];
	no_label.rw = dom(no_label.duration);
	no_label.length = 0;
	no_label.S = no_label.U = {};
	no_label.v = vrp.o;
}

MonodirectionalLabeling::~MonodirectionalLabeling()
{
	Clean();
}

void MonodirectionalLabeling::SetProblem(const PricingProblem& pricing_problem)
{
	// Set cut resources in null label.
	no_label.cut_cost = 0.0;
	no_label.cut_nz = {};
	no_label.cut_visited = vector<int>(pricing_problem.S.size(), 0);
	
	vrp_.D.AddArcs(pp_.A); // Add previously forbidden arcs.
	pp_ = pricing_problem;
	vrp_.D.RemoveArcs(pp_.A); // Remove current pricing problem forbidden arcs.
	Clean();
}

vector<Label*> MonodirectionalLabeling::Run(
	LBQueue* q, 
	MLBExecutionLog* log
) {
	// Use rolex to measure whole run time, and rolex2 to measure steps time.
	Stopwatch rolex(true), rolex2(false);
	vector<Label*> P; // Processed labels.
	while (!q->empty())
	{
		if (P.size() >= process_limit) { log->status = MLBStatus::ProcessLimitReached; break; }
		if (rolex.Peek() >= time_limit) { log->status = MLBStatus::TimeLimitReached; break; }
		
		// If label crossed t_m, but should not, then stop processing queue.
		// This extensions will be used for last-edge merge strategy.
		if (!cross && epsilon_bigger(q->top().makespan, t_m)) break;
		
		// Pop label.
		rolex2.Reset().Resume();
		LazyLabel ll = q->top();
		q->pop();
		*log->queuing_time += rolex2.Pause();
		
		// Turn lazy label into complete label.
		Label* l = ll.extension;
		if (lazy_extension)
		{
			rolex2.Reset().Resume();
			l = ExtensionStep(ll);
			*log->extension_time += rolex2.Pause();
		}
		if (!l) continue; // Label could not be extended.
		log->extended_count++;
		
		// Check domination.
		rolex2.Reset().Resume();
		bool is_dominated = DominationStep(l);
		*log->domination_time += rolex2.Pause();
		if (is_dominated) *log->positive_domination_time += rolex2.Pause();
		if (!is_dominated) *log->negative_domination_time += rolex2.Pause();
		if (is_dominated)
		{
			log->dominated_count++;
			delete l;
			continue;
		} // Label is dominated, ignore.
		
		// Correct existing labels.
		if (correcting)
		{
			rolex2.Reset().Resume();
			*log->corrected_count += CorrectionStep(l);
			*log->correction_time += rolex2.Pause();
		}
		
		// If min(rw(l)) > t_m, then l should not be extended and ll should have been preserved in the queue with the
		// new makespan.
		if (!cross && epsilon_bigger(min(l->rw), t_m))
		{
			q->push(LazyLabel(l->parent, l->v, min(l->rw)));
			delete l;
			continue;
		}
		
		// Otherwise, label l should be processed and extended.
		// Process label.
		rolex2.Reset().Resume();
		ProcessStep(l);
		
		log->processed_count++;
		*log->process_time += rolex2.Pause();
		P.push_back(l);
		stretch_to_size(*log->count_by_length, l->length+1, 0);
		log->count_by_length->at(l->length)++;
		processed_count++;
		
		// Get feasible extensions.
		if (epsilon_smaller_equal(min(l->rw), t_m))
		{
			rolex2.Reset().Resume();
			auto extensions = EnumerationStep(l);
			*log->enumeration_time += rolex2.Pause();
			*log->enumerated_count += extensions.size();
			
			// Add feasible extensions to the queue.
			rolex2.Reset().Resume();
			for (LazyLabel& ll: extensions) q->push(ll);
			*log->queuing_time += rolex2.Pause();
		}
	}
	
	if (q->empty()) log->status = MLBStatus::Finished;
	*log->time += rolex.Pause();
	
	return P;
}

LazyLabel MonodirectionalLabeling::Init() const
{
	LazyLabel ll = {(Label*) &no_label, vrp_.o, vrp_.tw[vrp_.o].left};
	if (!lazy_extension) ll.extension = ExtensionStep(ll);
	return ll;
}

Label* MonodirectionalLabeling::ExtensionStep(const LazyLabel& ll) const
{
	if (correcting && ll.parent->duration.Empty()) return nullptr;
	auto& l = ll.parent;
	Vertex v = ll.v;
	Vertex u = l->v;
	
	// If correcting and now reaching vertex v is infeasible, return nullptr.
	if (correcting && vrp_.ArrivalTime({u, v}, l->rw.left) == INFTY) return nullptr;
	
	// Check if depot triangle inequality holds.
	// If max(rw(lv)) < a_v and tau_u0v(max(rw(lv))) <= a_v - max(rw(lv)) then ignore label.
	if (epsilon_smaller(l->rw.right, vrp_.tw[v].left) && vrp_.D.IncludesArc({u, vrp_.d}) && vrp_.D.IncludesArc({vrp_.o, v}))
	{
		TimeUnit tau_u0v = vrp_.TravelTime({u, vrp_.d}, max(l->rw)) + vrp_.PreTravelTime({vrp_.o, v}, vrp_.tw[v].left);
		if (epsilon_smaller(tau_u0v, vrp_.tw[v].left - l->rw.right)) return nullptr;
	}
	
	auto lv = new Label();
	lv->parent = l;
	lv->v = v;
	lv->q = l->q + vrp_.q[v];
	lv->p = l->p + pp_.P[v];
	lv->length = l->length + 1;
	// If max(rw(l)) < min(img(dep_uv)) then no matter when we depart we reach v before its time window.
	// Otherwise, we can do the classic extension D_lv(t) = D_l(\dep_uv(t)) + \tau_uv(\dep_uv(t)).
	lv->duration = epsilon_smaller(max(l->rw), min(img(vrp_.dep[u][v])))
		? PWLFunction::ConstantFunction(l->duration(max(l->rw)) + min(vrp_.tw[v]) - max(l->rw), {min(vrp_.tw[v]), min(vrp_.tw[v])})
		: (l->duration + vrp_.tau[u][v]).Compose(vrp_.dep[u][v]);
	if (limited_extension && !cross) lv->duration.RestrictDomain({0.0, t_m});
	if (lv->duration.Empty()) { delete lv; return nullptr; } // If no duration pieces exist, then the label is dominated.
	lv->rw = dom(lv->duration);
	
	if (ng_routes_relaxation && ng.has_value())
	{
		const VertexSet& ng_v = ng->neighbors(v, lv->rw.left);
		lv->S = unite(intersection(l->S, ng_v), {v}); // S := (S(parent) ∩ N(v, t)) ∪ {v}
	}
	else
		lv->S = unite(l->S, {v}); // Full elementary route (no NG-relaxation)
	lv->U = unite(lv->S, unreachable_strengthened ? vrp_.Unreachable(v, lv->rw.left) : vrp_.WeakUnreachable(v, lv->rw.left));
	
	// Extend cut resources.
	lv->cut_cost = l->cut_cost;
	lv->cut_visited = l->cut_visited;
	for (int i = 0; i < pp_.S.size(); ++i)
	{
		if (pp_.S[i].test(lv->v))
		{
			lv->cut_visited[i]++;
			if (lv->cut_visited[i] == 2) lv->cut_cost += pp_.sigma[i]; // Visited 2 vertices of the cut.
		}
		if (lv->cut_visited[i] == 1)
		{
			lv->cut_nz.push_back(i); // Add to cut_nz cuts with exactly 1 visited vertex of the cut.
		}
	}
	lv->min_cost = min(img(lv->duration)) - lv->p - lv->cut_cost;
	return lv;
}

bool MonodirectionalLabeling::DominationStep(Label* l) const
{
	// If full route, then it is dominated if the reduced cost is bigger than or equal to zero.
	if (l->v == vrp_.d) return epsilon_bigger_equal(l->min_cost, 0.0);
	
	// Create function Delta which will be dominated.
	PWLDominationFunction Delta = l->duration;
	double l_beta = beta(l, partial);
	
	for (auto& demand_entry : U[l->v])
	{
		if (epsilon_bigger(demand_entry.first, l->q)) break;
		for (auto& m: demand_entry.second)
		{
			// We know that q(m) <= q(l), v(m) = v(l).
			if (sort_by_cost && epsilon_bigger(alpha(m, partial), l_beta)) break;
			if (!elementary_check_relaxation && !is_subset(m->U, l->U)) continue;
			
			if (!cost_check_relaxation)
			{
				// theta = p(l) + cut_cost(l) - p(m) - cut_cost(m) - \sum {sigma(i) : cut_visited[i](m) == 1 && cut_visited[i](l) != 1 }.
				double theta = l->p + l->cut_cost - m->p - m->cut_cost;
				for (int i: m->cut_nz) if (l->cut_visited[i] != 1) theta -= pp_.sigma[i];
				if (!partial && !Delta.IsAlwaysDominated(m->duration, theta)) continue;
				else if (partial && !Delta.DominatePieces(m->duration, theta)) continue;
			}
			
			return true;
		}
	}
	l->duration = (PWLFunction) Delta;
	l->rw = l->duration.Domain();
	l->min_cost = min(img(l->duration)) - l->p - l->cut_cost;
	return false;
}

int MonodirectionalLabeling::CorrectionStep(Label* m)
{
	int removed = 0;
	for (auto it_d = U[m->v].rbegin(); it_d != U[m->v].rend(); ++it_d)
	{
		auto& demand_entry = *it_d;
		if (epsilon_smaller(demand_entry.first, m->q)) break;
		for (int j = 0; j < demand_entry.second.size(); ++j)
		{
			Label* l = demand_entry.second[j];
			if (!elementary_check_relaxation && !is_subset(m->U, l->U)) continue;
			if (!cost_check_relaxation)
			{
				// theta = p(l) + cut_cost(l) - p(m) - cut_cost(m) - \sum {sigma(i) : cut_visited[i](m) == 1 && cut_visited[i](l) != 1 }.
				double theta = l->p + l->cut_cost - m->p - m->cut_cost;
				for (int i: m->cut_nz) if (l->cut_visited[i] != 1) theta -= pp_.sigma[i];
				PWLDominationFunction Delta(l->duration);
				if (partial)
				{
					Delta.DominatePieces(m->duration, theta);
					l->duration = (PWLFunction) Delta;
					l->rw = l->duration.Domain();
					l->min_cost = min(img(l->duration)) - l->p - l->cut_cost;
				}
				if ((!partial && Delta.IsAlwaysDominated(m->duration, theta)) || (partial && l->duration.Empty()))
				{
					// If l is fully dominated, remove.
					demand_entry.second.erase(demand_entry.second.begin()+j);
					--j;
					++removed;
				}
			}
		}
	}
	return removed;
}

void MonodirectionalLabeling::ProcessStep(Label* l)
{
	// (If domination structure) insert label l into the dominance structure U.
	if (sort_by_cost) insert_sorted(U[l->v].Insert(floor(l->q), {}), l, [&](Label* l1, Label* l2) { return alpha(l1, partial) < alpha(l2, partial); });
	else U[l->v].Insert(floor(l->q), {}).push_back(l); // else, just push it to the end of the list.
}

vector<LazyLabel> MonodirectionalLabeling::EnumerationStep(Label* l) const
{
	vector<LazyLabel> E;
	if (l->v == vrp_.d) return E; // End depot has no extensions.
	for (Vertex v: vrp_.D.Successors(l->v))
	{
		if (l->U.test(v)) continue;
		if (epsilon_bigger(l->q + vrp_.q[v], vrp_.Q)) continue;
		if (epsilon_bigger(min(l->rw), max(dom(vrp_.arr[l->v][v])))) continue;
		double makespan = vrp_.arr[l->v][v](max(min(l->rw), min(dom(vrp_.arr[l->v][v]))));
		LazyLabel ll{l, v, cross ? l->rw.left : makespan};
		if (!lazy_extension)
		{
			ll.extension = ExtensionStep(ll);
			if (!ll.extension) continue;
			ll.makespan = ll.extension->rw.left;
		}
		E.push_back(ll);
	}
	return E;
}

void MonodirectionalLabeling::Clean()
{
	processed_count = 0;
	for (auto& entry: U)
		for (auto& entry_2: entry)
			for (Label* m: entry_2.second)
				delete m;
	U = vector<DemandLevel>(vrp_.D.NbVertices());
}
} // solver