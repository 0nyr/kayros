//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "labeling/monodirectional_labeling.h"

#include <climits>
#include <algorithm>
#include <stdexcept>

#include "labeling/pwl_domination_function.h"
#include "labeling/trace.h"

using namespace std;
using namespace goc;
using namespace nyr;

namespace solver
{
extern bool trace_domination_detail; // pwl_domination_function.cpp
namespace
{
// kayros (M5.7): a label's rw and dom(duration) are maintained separately and
// can disagree by epsilon (goc's piece arithmetic is epsilon-tolerant, and the
// reverse-side normalizer can nudge reverse-arrival domain boundaries), and
// Value() throws outside the domain. Clamp boundary evaluations into the
// domain. This is Lera's epsilon search arithmetic (bounds/filters), never
// certification arithmetic (stage A reprices every column checker-exactly).
// Kept as a guard: instrumentation on 2026-08-05 never saw it fire, but
// nothing enforces rw within dom(duration) and the alternative to clamping is
// a thrown exception, not a better value.
double duration_at(const Label* l, double t)
{
	t = std::max(min(dom(l->duration)), std::min(t, max(dom(l->duration))));
	// M5.9 (21/n): durations are set-valued where departure-choice sets exist
	// (stacked pieces at one abscissa); the label semantics is the MINIMUM.
	// Value returns an arbitrary representative there, which over-priced
	// merges and discarded optimal columns (the Ari-B10 trace).
	return l->duration.MinValueAt(t);
}

// kayros (M5.7): when an entire label duration function lives inside a
// bridged sliver of the reverse-side normalizer (TW-tight labels on stepwise
// ATFs), goc's epsilon piece iteration can emit pieces with slightly
// out-of-order boundaries, leaving an inverted incremental domain_ (left >
// right by ~1e-3) that later evaluations reject. Rebuild with monotone
// non-overlapping boundaries; dust-inverted fragments are dropped. Search
// arithmetic only, and a no-op on well-formed functions (fast path below).
// M5.9 label tracing (dev tool, zero cost when unset): KAYROS_TRACE_PATH is a
// comma-separated vertex sequence (e.g. "0,8,9,7,13,3,1,16"); every label whose
// path is a PREFIX of it gets its fate printed to stderr (extension, domination
// verdict incl. the dominator, enumeration filters, queue events). Used to find
// where cold pricing kills a witness column's label chain. M13.0: the trace
// namespace moved to labeling/trace.h so the bidirectional merge shares it.

// M13.0: does the function carry any vertical (value-jump or choice) piece?
// Cheap linear scan; one of the conditions selecting the exact step-arc
// extension arithmetic. M13.3: NOT a sufficient one. Jump-free FORWARD
// functions carry CHOICE verticals routinely (Inverse of a departure plateau),
// so the decisive gate is the solve being step-carrying
// (goc::step_exact_arithmetic()); see the M13.3 note at the extension below.
static bool has_vertical(const PWLFunction& f)
{
	for (int k = 0; k < f.PieceCount(); ++k)
		if (f.Piece(k).is_vertical()) return true;
	return false;
}

PWLFunction normalize_pwl(const PWLFunction& f)
{
	// Fast path: boundaries already monotone (the norm) — return untouched so
	// well-formed instances keep bit-identical label arithmetic.
	bool ok = true;
	double hi = -INFTY;
	for (int k = 0; k < f.PieceCount(); ++k)
	{
		const Interval& d = f.Piece(k).domain;
		if (d.left < hi || d.right < d.left) { ok = false; break; }
		hi = d.right;
	}
	if (ok) return f;
	PWLFunction g;
	hi = -INFTY;
	for (int k = 0; k < f.PieceCount(); ++k)
	{
		const LinearFunction& p = f.Piece(k);
		double l = std::max(p.domain.left, hi);
		double r = p.domain.right;
		if (r < l) continue;
		g.AddPiece(LinearFunction({l, p.Value(l)}, {r, p.Value(r)}));
		hi = r;
	}
	return g;
}

// Definition of Alpha from Section 5.2.
double alpha(Label* l, bool partial)
{
	if (partial) return l->min_cost;
	else return -(l->rw.right-duration_at(l, l->rw.right))-l->p-l->cut_cost;
}

// Definition of Beta from Section 5.2.
double beta(Label* l, bool partial)
{
	if (partial) return max(img(l->duration))-l->p-l->cut_cost;
	else return -(l->rw.right-duration_at(l, l->rw.right))-l->p-l->cut_cost;
}
}

MonodirectionalLabeling::MonodirectionalLabeling(
	const VRPInstance& vrp,
	const std::optional<TDNGRoutesParams>& ng_params
): 
	vrp_(vrp), 
	correcting(true),
	// Create neignborhoods if ng_params are given.
	ng(ng_params ? std::make_optional<TDNGNeighborhoods>(vrp, ng_params->partitioned_horizon, ng_params->nb_neighbors_to_keep) : std::nullopt)
{
	cross = true;
	process_limit = INT_MAX;
	time_limit = 2.0_hr;
	partial = limited_extension = lazy_extension = unreachable_strengthened = sort_by_cost = true;
	elementary_check_relaxation = cost_check_relaxation = ng_routes_relaxation = false;
	theta_relaxation = false;
	packed_filter = 0;
	processed_count = 0;
	
	t_m = vrp.T;
	U = vector<DemandLevel>(vrp.D.NbVertices());
	packed_ = vector<PackedLevel>(vrp.D.NbVertices());
	
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
	if (correcting && !lazy_extension)
		throw std::invalid_argument("Label correction requires lazy extension");

	// Use rolex to measure whole run time, and rolex2 to measure steps time.
	Stopwatch rolex(true), rolex2(false);
	vector<Label*> P; // Processed labels.
	while (!q->empty())
	{
		if (P.size() >= process_limit) { log->status = MLBStatus::ProcessLimitReached; break; }
		if (rolex.Peek() >= time_limit) { log->status = MLBStatus::TimeLimitReached; break; }
		// kayros (M13.2): the label containers are where the full-horizon
		// memory pathology accumulates: unwind with an honest verdict
		// instead of being OOM-killed. Sticky: outer loops see it too.
		if (MemoryMonitor::Exceeded()) { log->status = MLBStatus::MemoryLimitReached; break; }
		
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
		
		// If min(rw(l)) > t_m, then l should not be extended and ll should have been preserved in the queue with the
		// new makespan.
		if (!cross && epsilon_bigger(min(l->rw), t_m))
		{
			q->push(LazyLabel(l->parent, l->v, min(l->rw)));
			delete l;
			continue;
		}

		// Only a label retained for processing may correct stored labels.
		if (correcting)
		{
			rolex2.Reset().Resume();
			*log->corrected_count += CorrectionStep(l);
			*log->correction_time += rolex2.Pause();
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
	if (correcting && !lazy_extension)
		throw std::invalid_argument("Label correction requires lazy extension");

	LazyLabel ll = {(Label*) &no_label, vrp_.o, vrp_.tw[vrp_.o].left};
	if (!lazy_extension) ll.extension = ExtensionStep(ll);
	return ll;
}

Label* MonodirectionalLabeling::ExtensionStep(const LazyLabel& ll) const
{
	if (correcting && ll.parent->duration.Empty())
	{
		if (trace::on() && trace::prefix_len(ll.parent) > 0)
			fprintf(stderr, "TRC EXT-FAIL path=%s+%d reason=parent-duration-empty(correcting)\n",
				trace::path_str(ll.parent).c_str(), ll.v);
		return nullptr;
	}
	auto& l = ll.parent;
	Vertex v = ll.v;
	Vertex u = l->v;
	
	// If correcting and now reaching vertex v is infeasible, return nullptr.
	if (correcting && vrp_.ArrivalTime({u, v}, l->rw.left) == INFTY)
	{
		if (trace::on() && trace::prefix_len(l) > 0 && trace::prefix_len(l) < (int) trace::target().size()
			&& trace::target()[trace::prefix_len(l)] == v)
			fprintf(stderr, "TRC EXT-FAIL path=%s+%d reason=arrival-INFTY(correcting) rw_left=%.3f\n",
				trace::path_str(l).c_str(), v, l->rw.left);
		return nullptr;
	}
	
	// M5.9 trace: does the extension complete a prefix of the target?
	int tr_parent = trace::prefix_len(l);
	bool tr = tr_parent > 0 && tr_parent < (int) trace::target().size() && trace::target()[tr_parent] == v;

	// Check if depot triangle inequality holds.
	// If max(rw(lv)) < a_v and tau_u0v(max(rw(lv))) <= a_v - max(rw(lv)) then ignore label.
	if (epsilon_smaller(l->rw.right, vrp_.tw[v].left) && vrp_.D.IncludesArc({u, vrp_.d}) && vrp_.D.IncludesArc({vrp_.o, v}))
	{
		TimeUnit tau_u0v = vrp_.TravelTime({u, vrp_.d}, max(l->rw)) + vrp_.PreTravelTime({vrp_.o, v}, vrp_.tw[v].left);
		if (epsilon_smaller(tau_u0v, vrp_.tw[v].left - l->rw.right))
		{
			if (tr) fprintf(stderr, "TRC EXT-FAIL path=%s+%d reason=depot-triangle\n", trace::path_str(l).c_str(), v);
			return nullptr;
		}
	}
	
	auto lv = new Label();
	lv->parent = l;
	lv->v = v;
	lv->q = l->q + vrp_.q[v];
	lv->p = l->p + pp_.P[v];
	lv->length = l->length + 1;
	// If max(rw(l)) < min(img(dep_uv)) then no matter when we depart we reach v before its time window.
	// Otherwise, we can do the classic extension D_lv(t) = D_l(\dep_uv(t)) + \tau_uv(\dep_uv(t)).
	// M13.0: on step-carrying arcs (any vertical in dep, tau or the label's own
	// duration, believed at the time to be true only on the exact value-jump
	// path, jump-free arcs carrying none; the M13.3 note below refutes that
	// premise and supplies the real gate) the composite (D + tau) o dep is
	// UNSOUND: a reversed waiting span appears
	// as a plateau of dep at value tau0 while D + tau carries the same-arc
	// vertical at tau0, and the plateau rule fills the whole span with one
	// constant read from the vertical, erasing position-dependent mandatory
	// waiting (Rifki-2 dominator 21,4,2,10,3 claimed ~1959 where the
	// checker-exact suffix duration is ~12703: under-estimated dominators then
	// erase witness labels at the Exact level). The elapsed-time identity
	// D_lv(t) = D_l(dep(t)) + t - dep(t) = ((D - Id) o dep)(t) + t is
	// algebraically equal wherever arr(dep(t)) = t, charges waiting spans
	// exactly (slope-1 fill resolves the plateau/vertical correspondence for
	// BOTH vertical kinds, which invert to indistinguishable dep plateaus) and
	// over-estimates only unattained jump-gap abscissae (sound side). It also
	// keeps tau out of the composed outer entirely.
	// M13.3: gated on the solve being step-carrying. The M13.0 premise that
	// only step-carrying arcs carry verticals is FALSE. CHOICE verticals
	// (Inverse of a departure plateau) and set-valued label durations are
	// ubiquitous on jump-free instances too, so on those the predicate fired
	// on most extensions and replaced the audited v1.0.0 extension arithmetic
	// with the elapsed-time identity, whose jump-gap over-estimation is sound
	// only against the exact-path semantics: cold solves certified optima
	// strictly above checker-valid solutions (Vu2020 n=59 Vu-A5-pA-d90-w40,
	// 2764.380 against the true 2760.110). Jump-free solves keep the legacy
	// composite; step-carrying solves keep M13.0 unchanged.
	const bool step_arc = goc::step_exact_arithmetic()
		&& (has_vertical(vrp_.dep[u][v]) || has_vertical(vrp_.tau[u][v])
			|| has_vertical(l->duration));
	if (epsilon_smaller(max(l->rw), min(img(vrp_.dep[u][v]))))
	{
		// Departure-before-window branch: the label's minimal duration at its
		// last ready time plus the forced wait. MinValueAt on the step path
		// (durations are set-valued at choice abscissae, min semantics, 21/n);
		// Value on the legacy path for bit-identity.
		double d_at = step_arc ? l->duration.MinValueAt(max(l->rw)) : l->duration(max(l->rw));
		lv->duration = PWLFunction::ConstantFunction(d_at + min(vrp_.tw[v]) - max(l->rw), {min(vrp_.tw[v]), min(vrp_.tw[v])});
	}
	else if (step_arc)
	{
		PWLFunction comp = (l->duration - PWLFunction::IdentityFunction(l->duration.Domain())).Compose(vrp_.dep[u][v]);
		lv->duration = comp.Empty() ? comp : comp + PWLFunction::IdentityFunction(comp.Domain());
	}
	else
		lv->duration = (l->duration + vrp_.tau[u][v]).Compose(vrp_.dep[u][v]);
	if (limited_extension && !cross) lv->duration.RestrictDomain({0.0, t_m});
	lv->duration = normalize_pwl(lv->duration); // kayros (M5.7): heal epsilon-dust piece misorder.
	if (lv->duration.Empty())
	{
		if (tr) fprintf(stderr, "TRC EXT-FAIL path=%s+%d reason=empty-duration\n", trace::path_str(l).c_str(), v);
		delete lv; return nullptr; // If no duration pieces exist, then the label is dominated.
	}
	lv->rw = dom(lv->duration);
	
	if (theta_relaxation)
		lv->S = unite(intersection(l->S, theta_local.empty() ? theta : theta_local[v]), {v}); // (ms61) S := (S(parent) ∩ theta) ∪ {v}
	else if (ng_routes_relaxation && ng.has_value())
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
	if (tr) fprintf(stderr, "TRC EXT path=%s rw=[%.3f,%.3f] dur_img=[%.3f,%.3f] min_cost=%.6f q=%.1f\n",
		trace::path_str(lv).c_str(), lv->rw.left, lv->rw.right,
		min(img(lv->duration)), max(img(lv->duration)), lv->min_cost, lv->q);
	// M13.0: full piece dump of the witness label's duration function AND of
	// its parent's CURRENT (possibly domination-shrunk) duration at extension
	// time (dev tool; KAYROS_TRACE_DUMP=1 with the path trace on).
	if (tr && std::getenv("KAYROS_TRACE_DUMP"))
	{
		auto dump = [&](const char* tag, const Label* lab) {
			for (int k = 0; k < lab->duration.PieceCount(); ++k)
			{
				const LinearFunction& p = lab->duration.Piece(k);
				fprintf(stderr, "TRC %s path=%s piece=%d dom=[%.6f,%.6f] img=[%.6f,%.6f] %s att=%.6f\n",
					tag, trace::path_str(lab).c_str(), k, p.domain.left, p.domain.right,
					p.image.left, p.image.right,
					p.is_vertical() ? (p.is_choice_vertical() ? "C" : "J") : "-",
					p.is_vertical() ? p.intercept : p.Value(p.domain.left));
			}
		};
		dump("EXT-PARENT", l);
		dump("EXT-DUMP", lv);
	}
	return lv;
}

bool MonodirectionalLabeling::DominationStep(Label* l) const
{
	// If full route, then it is dominated if the reduced cost is bigger than or equal to zero.
	if (l->v == vrp_.d) return epsilon_bigger_equal(l->min_cost, 0.0);
	
	// Create function Delta which will be dominated.
	PWLDominationFunction Delta = l->duration;
	double l_beta = beta(l, partial);
	trace_domination_detail = trace::on() && trace::prefix_len(l) > 0;
	
	auto packed_it = packed_[l->v].begin();
	for (auto& demand_entry : U[l->v])
	{
		if (epsilon_bigger(demand_entry.first, l->q)) break;
		const vector<PackedEntry>* packed = nullptr;
		if (packed_filter) { packed = &*packed_it; ++packed_it; }
		const BoundLevel& bucket = demand_entry.second;
		if (packed && packed->size() != bucket.size())
			throw std::logic_error("packed dominance filter out of sync with its bucket");
		for (size_t k = 0; k < bucket.size(); ++k)
		{
			Label* m = bucket[k];
			// We know that q(m) <= q(l), v(m) = v(l).
			if (packed)
			{
				const PackedEntry& e = (*packed)[k];
				if (packed_filter >= 2 && (e.alpha != alpha(m, partial) || e.U != m->U))
					throw std::logic_error("packed dominance filter entry differs from its label");
				if (sort_by_cost && epsilon_bigger(e.alpha, l_beta)) break;
				if (!elementary_check_relaxation && !is_subset(e.U, l->U)) continue;
			}
			else
			{
				if (sort_by_cost && epsilon_bigger(alpha(m, partial), l_beta)) break;
				if (!elementary_check_relaxation && !is_subset(m->U, l->U)) continue;
			}
			
			if (!cost_check_relaxation)
			{
				// theta = p(l) + cut_cost(l) - p(m) - cut_cost(m) - \sum {sigma(i) : cut_visited[i](m) == 1 && cut_visited[i](l) != 1 }.
				double theta = l->p + l->cut_cost - m->p - m->cut_cost;
				for (int i: m->cut_nz) if (l->cut_visited[i] != 1) theta -= pp_.sigma[i];
				if (!partial && !Delta.IsAlwaysDominated(m->duration, theta)) continue;
				else if (partial && !Delta.DominatePieces(m->duration, theta)) continue;
			}
			
			trace_domination_detail = false;
			if (trace::prefix_len(l) > 0)
			{
				std::string lu, mu;
				for (int b = 0; b < (int) vrp_.D.NbVertices(); ++b)
				{
					if (l->U.test(b)) lu += std::to_string(b) + ",";
					if (m->U.test(b)) mu += std::to_string(b) + ",";
				}
				fprintf(stderr, "TRC DOMINATED lvl=%s path=%s by=%s theta=%.6f m_dur_img=[%.3f,%.3f] lU={%s} mU={%s} l_rw=[%.3f,%.3f] m_rw=[%.3f,%.3f]\n",
					(elementary_check_relaxation ? "HE" : cost_check_relaxation ? "HC" : "EX"),
					trace::path_str(l).c_str(), trace::path_str(m).c_str(),
					l->p + l->cut_cost - m->p - m->cut_cost,
					min(img(m->duration)), max(img(m->duration)),
					lu.c_str(), mu.c_str(), l->rw.left, l->rw.right, m->rw.left, m->rw.right);
			}
			return true;
		}
	}
	trace_domination_detail = false;
	if (trace::prefix_len(l) > 0)
		fprintf(stderr, "TRC SURVIVED lvl=%s path=%s rw=[%.3f,%.3f] dur_img=[%.3f,%.3f] min_cost=%.6f\n",
			(elementary_check_relaxation ? "HE" : cost_check_relaxation ? "HC" : "EX"),
			trace::path_str(l).c_str(), min(dom((PWLFunction) Delta)), max(dom((PWLFunction) Delta)),
			min(img((PWLFunction) Delta)), max(img((PWLFunction) Delta)),
			min(img((PWLFunction) Delta)) - l->p - l->cut_cost);
	l->duration = normalize_pwl((PWLFunction) Delta); // kayros (M5.7)
	l->rw = l->duration.Domain();
	l->min_cost = min(img(l->duration)) - l->p - l->cut_cost;
	return false;
}

int MonodirectionalLabeling::CorrectionStep(Label* m)
{
	if (cost_check_relaxation) return 0;
	int removed = 0;
	auto packed_it = packed_[m->v].rbegin();
	for (auto it_d = U[m->v].rbegin(); it_d != U[m->v].rend(); ++it_d)
	{
		auto& demand_entry = *it_d;
		if (epsilon_smaller(demand_entry.first, m->q)) break;
		vector<PackedEntry>* packed = nullptr;
		if (packed_filter) { packed = &*packed_it; ++packed_it; }
		if (packed && packed->size() != demand_entry.second.size())
			throw std::logic_error("packed dominance filter out of sync with its bucket");
		bool changed_cost = false;
		for (int j = 0; j < demand_entry.second.size(); ++j)
		{
			if (packed)
			{
				// Same test as below, read from the packed copy.
				if (!elementary_check_relaxation && !is_subset(m->U, (*packed)[j].U)) continue;
			}
			Label* l = demand_entry.second[j];
			if (packed_filter >= 2 && packed && (*packed)[j].U != l->U)
				throw std::logic_error("packed dominance filter entry differs from its label");
			if (!elementary_check_relaxation && !is_subset(m->U, l->U)) continue;
			if (!cost_check_relaxation)
			{
				// theta = p(l) + cut_cost(l) - p(m) - cut_cost(m) - \sum {sigma(i) : cut_visited[i](m) == 1 && cut_visited[i](l) != 1 }.
				double theta = l->p + l->cut_cost - m->p - m->cut_cost;
				for (int i: m->cut_nz) if (l->cut_visited[i] != 1) theta -= pp_.sigma[i];
				PWLDominationFunction Delta(l->duration);
				if (partial)
				{
					const double previous_cost = l->min_cost;
					Delta.DominatePieces(m->duration, theta);
					l->duration = normalize_pwl((PWLFunction) Delta); // kayros (M5.7)
					l->rw = l->duration.Domain();
					l->min_cost = min(img(l->duration)) - l->p - l->cut_cost;
					changed_cost |= l->min_cost != previous_cost;
				}
				if ((!partial && Delta.IsAlwaysDominated(m->duration, theta)) || (partial && l->duration.Empty()))
				{
					// If l is fully dominated, remove.
					retired_labels_.push_back(l);
					demand_entry.second.erase(demand_entry.second.begin()+j);
					if (packed) packed->erase(packed->begin()+j);
					--j;
					++removed;
				}
			}
		}
		if (changed_cost)
		{
			// Subsequent labels consult U before this batch returns.
			if (sort_by_cost)
				std::stable_sort(demand_entry.second.begin(), demand_entry.second.end(),
					[&](Label* a, Label* b) { return alpha(a, partial) < alpha(b, partial); });
			corrected_merge_buckets_.emplace_back(m->v, demand_entry.first);
			// Sort keys changed and the bucket may have been reordered.
			if (packed) RebuildPacked(*packed, demand_entry.second);
		}
	}
	return removed;
}

void MonodirectionalLabeling::ProcessStep(Label* l)
{
	// (If domination structure) insert label l into the dominance structure U.
	if (packed_filter)
	{
		// (ms61) Same insertion as below, mirrored at the same index in the
		// packed copy.
		const CapacityUnit key = floor(l->q);
		BoundLevel& bucket = U[l->v].Insert(key, {});
		// Mirror VectorMap::Insert: a new demand key opens an empty packed
		// bucket at the same sorted position.
		size_t key_index = 0;
		for (auto& entry: U[l->v]) { if (entry.first == key) break; ++key_index; }
		PackedLevel& level = packed_[l->v];
		if (level.size() < (size_t) (U[l->v].end() - U[l->v].begin())) level.insert(level.begin() + key_index, vector<PackedEntry>());
		vector<PackedEntry>& packed = level[key_index];
		if (packed.size() != bucket.size())
			throw std::logic_error("packed dominance filter out of sync with its bucket");
		size_t position = bucket.size();
		if (sort_by_cost)
			position = std::upper_bound(bucket.begin(), bucket.end(), l,
				[&](Label* l1, Label* l2) { return alpha(l1, partial) < alpha(l2, partial); }) - bucket.begin();
		bucket.insert(bucket.begin() + position, l);
		packed.insert(packed.begin() + position, PackedEntry{alpha(l, partial), l->U});
		return;
	}
	if (sort_by_cost) insert_sorted(U[l->v].Insert(floor(l->q), {}), l, [&](Label* l1, Label* l2) { return alpha(l1, partial) < alpha(l2, partial); });
	else U[l->v].Insert(floor(l->q), {}).push_back(l); // else, just push it to the end of the list.
}

vector<LazyLabel> MonodirectionalLabeling::EnumerationStep(Label* l) const
{
	vector<LazyLabel> E;
	// M5.9 trace: which vertex must extend this prefix, if any.
	int tl = trace::prefix_len(l);
	Vertex trace_next = (tl > 0 && tl < (int) trace::target().size()) ? trace::target()[tl] : -1;
	if (l->v == vrp_.d) return E; // End depot has no extensions.
	for (Vertex v: vrp_.D.Successors(l->v))
	{
		bool tr = trace_next >= 0 && v == trace_next;
		if (l->U.test(v)) { if (tr) fprintf(stderr, "TRC ENUM-SKIP path=%s v=%d reason=unreachable-set\n", trace::path_str(l).c_str(), v); continue; }
		if (epsilon_bigger(l->q + vrp_.q[v], vrp_.Q)) { if (tr) fprintf(stderr, "TRC ENUM-SKIP path=%s v=%d reason=capacity\n", trace::path_str(l).c_str(), v); continue; }
		if (epsilon_bigger(min(l->rw), max(dom(vrp_.arr[l->v][v])))) { if (tr) fprintf(stderr, "TRC ENUM-SKIP path=%s v=%d reason=rw-beyond-arr-dom rw_min=%.3f arr_dom_max=%.3f\n", trace::path_str(l).c_str(), v, min(l->rw), max(dom(vrp_.arr[l->v][v]))); continue; }
		double makespan = vrp_.arr[l->v][v](max(min(l->rw), min(dom(vrp_.arr[l->v][v]))));
		LazyLabel ll{l, v, cross ? l->rw.left : makespan};
		if (!lazy_extension)
		{
			ll.extension = ExtensionStep(ll);
			if (!ll.extension) { if (tr) fprintf(stderr, "TRC ENUM-SKIP path=%s v=%d reason=eager-extension-failed\n", trace::path_str(l).c_str(), v); continue; }
			ll.makespan = ll.extension->rw.left;
		}
		if (tr) fprintf(stderr, "TRC ENUM-PUSH path=%s v=%d makespan=%.3f\n", trace::path_str(l).c_str(), v, ll.makespan);
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
	for (Label* label: retired_labels_) delete label;
	retired_labels_.clear();
	corrected_merge_buckets_.clear();
	U = vector<DemandLevel>(vrp_.D.NbVertices());
	packed_ = vector<PackedLevel>(vrp_.D.NbVertices());
}

void MonodirectionalLabeling::RebuildPacked(vector<PackedEntry>& packed, const BoundLevel& labels) const
{
	packed.resize(labels.size());
	for (size_t k = 0; k < labels.size(); ++k) packed[k] = PackedEntry{alpha(labels[k], partial), labels[k]->U};
}
} // solver
