//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "labeling/bidirectional_labeling.h"

#include "bcp/pricing_problem.h"

#include <climits>
#include <cstdlib>

using namespace std;
using namespace goc;
using namespace nyr;

namespace solver
{
namespace
{
// kayros (M5.7): bridge upward value jumps of a monotone PWL function with
// steep segments so that Inverse() yields gap-free departure functions.
// Interior plateaus of the forward arrival function (Rifki-style stepwise
// travel times encode tau down-steps as slope -1 arrival segments) become
// value jumps in dep and hence in the reverse arrival below; inverting a
// jump produces an interior domain gap and DepartureTime()/Value() then
// fails. The bridged function never exceeds the original (the steep segment
// stays below the following piece), so reverse arrivals are under-estimated
// on a measure-delta sliver per jump: pricing/bounds stay valid lower bounds
// and no wrong optimum can be certified (costs are checker-exact via the
// stage-A repricing in the kayros bridge).
PWLFunction continuize_value_jumps(const PWLFunction& f)
{
	const double delta = 1e-3; // >> goc EPS (1e-6), dust vs any horizon
	if (f.PieceCount() <= 1) return f;
	// Piecewise breakpoints; value jumps appear as duplicate-x pairs.
	vector<Point2D> bp;
	for (int k = 0; k < f.PieceCount(); ++k)
	{
		const LinearFunction& p = f.Piece(k);
		Point2D l(p.domain.left, p.Value(p.domain.left));
		Point2D r(p.domain.right, p.Value(p.domain.right));
		if (bp.empty() || bp.back().x != l.x || bp.back().y != l.y) bp.push_back(l);
		if (r.x != l.x || r.y != l.y) bp.push_back(r);
	}
	PWLFunction g;
	size_t k = 0;
	while (k < bp.size())
	{
		size_t j = k;
		while (j + 1 < bp.size() && bp[j + 1].x == bp[k].x) ++j;
		Point2D lo(bp[k].x, bp[k].y), hi(bp[j].x, bp[j].y);
		Point2D cur = lo;
		if (hi.y > lo.y && j + 1 < bp.size())
		{
			// Steep bridge from the lower value onto the following segment.
			const Point2D& nxt = bp[j + 1];
			double d = min(delta, (nxt.x - hi.x) / 2.0);
			double t = d / (nxt.x - hi.x);
			Point2D mid(hi.x + d, hi.y + t * (nxt.y - hi.y));
			g.AddPiece(LinearFunction(cur, mid));
			cur = mid;
		}
		if (j + 1 < bp.size())
		{
			const Point2D& nxt = bp[j + 1];
			// Continue to the next breakpoint's lower value.
			size_t j2 = j + 1;
			g.AddPiece(LinearFunction(cur, Point2D(nxt.x, bp[j2].y)));
		}
		k = j + 1;
	}
	return g;
}

// Reverses a VRP instance.
// o' := d
// d' := o
// D' := reverse(D)
// tw'(v) := [T-b(v), T-a(v)]
// arr'_vu(t) := T-dep_uv(T-t)
VRPInstance reverse_instance(const VRPInstance& vrp)
{
	VRPInstance r = vrp; // default shallow copy.
	swap(r.o, r.d);
	r.D = vrp.D.Reverse();
	for (Vertex v: r.D.Vertices()) r.tw[v] = {vrp.T - vrp.tw[v].right, vrp.T - vrp.tw[v].left};
	for (Vertex u: vrp.D.Vertices())
	{
		for (Vertex v: vrp.D.Successors(u))
		{
			// Compute reverse travel functions. M5.9 (design memo 12.2): the
			// reflection T - dep(T - t) is built by the exact graph transforms
			// FlipTime + FlipValue (one IEEE subtraction per coordinate,
			// platform-stable, jumps keep their attained endpoints), not by
			// Compose(T - Id) + arithmetic. The waiting-time prefix (a plateau
			// at the earliest reverse arrival, covering [min tw, min dom)) is a
			// direct graph edit equivalent to the old Min(Constant, .) on a
			// non-decreasing function, avoiding Max/Intersection entirely.
			r.arr[v][u] = vrp.dep[u][v].FlipTime(vrp.T).FlipValue(vrp.T);
			if (epsilon_smaller(min(r.tw[v]), min(dom(r.arr[v][u]))))
			{
				PWLFunction padded;
				padded.AddPiece(LinearFunction(
					Point2D(min(r.tw[v]), min(img(r.arr[v][u]))),
					Point2D(min(dom(r.arr[v][u])), min(img(r.arr[v][u])))));
				for (auto& p: r.arr[v][u].Pieces()) padded.AddPiece(p);
				r.arr[v][u] = padded;
			}
			// M5.9: the reverse mollifier is load-bearing and unconditional.
			// Time-reversal turns a left-continuous forward step function into a
			// right-continuous reverse one; the labeling assumes uniform
			// left-continuity, so exact reverse verticals misprice at jumps. The
			// mollified continuous path is sound (validated by the randomized
			// differential fuzzer). continuize_value_jumps is a no-op on
			// jump-free arcs.
			if (!std::getenv("KAYROS_STEP_EXACT")) // M5.9 exact-jump path (13.2 tagged verticals)
				r.arr[v][u] = continuize_value_jumps(r.arr[v][u]); // kayros (M5.7): see above.
			r.tau[v][u] = r.arr[v][u] - PWLFunction::IdentityFunction({0.0, vrp.T});
			r.dep[v][u] = r.arr[v][u].Inverse();
			r.pretau[v][u] = PWLFunction::IdentityFunction(dom(r.dep[v][u])) - r.dep[v][u];
		}
	}
	// Add travel functions for (i, i) (for boundary reasons).
	for (Vertex u: r.D.Vertices())
	{
		r.tau[u][u] = r.pretau[u][u] = PWLFunction::ConstantFunction(0.0, r.tw[u]);
		r.dep[u][u] = r.arr[u][u] = PWLFunction::IdentityFunction(r.tw[u]);
	}
	// Set LDT.
	for (Vertex i: r.D.Vertices())
	{
		vector<TimeUnit> LDT_i = compute_latest_departure_time(r.D, i, r.tw[i].right, [&] (Vertex u, Vertex v, double tf) { return r.DepartureTime({u,v}, tf); });
		for (Vertex k: r.D.Vertices()) r.LDT[k][i] = LDT_i[k];
	}
	return r;
}

PricingProblem reverse_pricing_problem(const PricingProblem& pp)
{
	PricingProblem rpp = pp;
	rpp.A.clear();
	for (Arc e: pp.A) rpp.A.push_back(e.Reverse());
	return rpp;
}
}

BidirectionalLabeling::BidirectionalLabeling(
	const VRPInstance& vrp,
	const std::optional<TDNGRoutesParams>& ng_routes_params
): 
	vrp_(vrp), 
	ng_params_(ng_routes_params),
	lbl_{
		MonodirectionalLabeling(vrp_, ng_routes_params), // Forward labeling.
		MonodirectionalLabeling(reverse_instance(vrp_), ng_routes_params) // Backward labeling.
	}
{
	solution_limit = INT_MAX;
	time_limit = Duration::Max();
	screen_output = nullptr;
	closing_state = true;
	merge_start = 0;
	lbl_[0].process_limit = lbl_[1].process_limit = 10;
	lbl_[0].cross = false, lbl_[1].cross = true;
	partial = limited_extension = lazy_extension = unreachable_strengthened = sort_by_cost = true;
	elementary_check_relaxation = cost_check_relaxation = ng_routes_relaxation = false;
	correcting = false;
	// M5.9: `symmetric` was NEVER initialized (upstream set it from the
	// experiment JSON, e.g. bpc_lera.json: false; the vendoring dropped that
	// wiring), so line ~172 read an indeterminate value: per-binary-
	// deterministic garbage that can differ across toolchains. Prime suspect
	// for the 2026-07-10 cross-platform certification divergence. Default =
	// upstream Lera BPC config (false: asymmetric, t_m = T). Env toggle for
	// the divergence experiment; TODO remove after the experiment concludes.
	symmetric = std::getenv("KAYROS_LBL_SYMMETRIC") != nullptr;
}

BLBExecutionLog BidirectionalLabeling::Run(
    const PricingProblem& pricing_problem, 
    std::vector<Route>* R,
    LabelingLevel level
) {
	// Clean solution pool.
	S.clear();
	M[0] = M[1] = vector<MonodirectionalLabeling::DemandLevel>(vrp_.D.NbVertices());
	
	// Set pricing problem.
	vrp_.D.AddArcs(pp_.A); // Add previously forbidden arcs.
	pp_ = pricing_problem;
	vrp_.D.RemoveArcs(pp_.A); // Remove pricing problem forbidden arcs.
	
	// Init forward and backward labeling.
	lbl_[0].SetProblem(pp_);
	lbl_[1].SetProblem(reverse_pricing_problem(pp_));
	lbl_[0].t_m = lbl_[1].t_m = symmetric ? vrp_.T / 2 : vrp_.T;
	
	// Determine level flags.
	setup_labeling_level_flag(level);

	// Set flags based on the level
	lbl_[0].partial = lbl_[1].partial = partial;
	lbl_[0].elementary_check_relaxation = lbl_[1].elementary_check_relaxation = elementary_check_relaxation;
	lbl_[0].cost_check_relaxation = lbl_[1].cost_check_relaxation = cost_check_relaxation;
	lbl_[0].limited_extension = lbl_[1].limited_extension = limited_extension;
	lbl_[0].lazy_extension = lbl_[1].lazy_extension = lazy_extension;
	lbl_[0].sort_by_cost = lbl_[1].sort_by_cost = sort_by_cost;
	lbl_[0].unreachable_strengthened = lbl_[1].unreachable_strengthened = unreachable_strengthened;
	lbl_[0].correcting = lbl_[1].correcting = correcting;
	
	BLBExecutionLog log(true);
	Stopwatch rolex(false), merge_rolex(false);
	run_deadline_ = Deadline::In(time_limit); // (kayros M5.2)
	
	// Init queues with initial labels.
	LBQueue q[2];
	q[0].push(lbl_[0].Init());
	q[1].push(lbl_[1].Init());
	
	// Index monodirectional labeling logs by direction.
	MLBExecutionLog* mlb_log[2] { &*log.forward_log, &*log.backward_log };
	
	// Initialize output.
	TableStream tstream(screen_output, 2.0);
	tstream.AddColumn("time", 10).AddColumn("fw-time", 10).AddColumn("bw-time", 10).AddColumn("fw-proc", 10).
		AddColumn("bw-proc", 10).AddColumn("#sol", 6).AddColumn("fw-t_m", 8).AddColumn("bw-t_m", 8).
		AddColumn("#q-f", 10).AddColumn("#q-b", 10);
	tstream.WriteHeader();
	
	rolex.Resume();
	
	// While there are labels to extend, do it.
	bool processed = true;
	while (processed)
	{
		processed = false;
		// For each direction of the labeling (0=Forward, 1=Backward).
		for (int d: {0, 1})
		{
			int od = (d+1)%2; // opposite direction.
			
			if (q[d].empty()) continue;
			if (rolex.Peek() >= time_limit) { log.status = BLBStatus::TimeLimitReached; break; } // Check if TLim is reached.
			if (S.size() >= solution_limit) { log.status = BLBStatus::SolutionLimitReached; break; } // Check if SLim is reached.
			lbl_[d].time_limit = time_limit - rolex.Peek(); // Set time limit.
			auto P = lbl_[d].Run(&q[d], mlb_log[d]);
			
			// If iterative-merge is enabled, then add the labels to the structure.
			if (!closing_state)
				for (Label* l: P)
					insert_sorted(M[d][l->v].Insert(floor(l->q), {}), l, [] (Label* l, Label* m) { return l->min_cost < m->min_cost; });
			
			// If iterative-merge is enabled, then try to merge.
			if (!closing_state && log.forward_log->processed_count >= merge_start)
			{
				merge_rolex.Reset().Resume();
				for (Label* l: P)
				{
					if (run_deadline_.Reached()) break; // (kayros M5.2)
					IterativeMerge(l, M[od]);
				}
				*log.merge_time += merge_rolex.Pause();
			}
			
			// Check if any full route was generated.
			for (Label* l: P)
				if (d == 0 && l->v == vrp_.d && epsilon_smaller(l->min_cost, 0.0))
					AddSolution(l->Path(), min(img(l->duration)));
			
			// Update t_m.
			if (q[d].empty()) lbl_[d].t_m = vrp_.T - lbl_[od].t_m; // If d has no more labels in the queue, the middle is t_m
			else lbl_[od].t_m = min(lbl_[od].t_m, max(vrp_.T-lbl_[d].t_m, vrp_.T-q[d].top().makespan));
			
			// Check if any label was processed.
			processed |= !P.empty();
		}
		
		// Output to screen.
		if (tstream.RegisterAttempt() || !processed)
		{
			tstream.WriteRow({STR(rolex.Peek()), STR(mlb_log[0]->time), STR(mlb_log[1]->time),
					 STR(mlb_log[0]->processed_count), STR(mlb_log[1]->processed_count), STR(S.size()),
					 STR(lbl_[0].t_m), STR(vrp_.T-lbl_[1].t_m), STR(q[0].size()), STR(q[1].size())});
		}
	}
	
	// Last-edge merge.
	if (S.size() < solution_limit && rolex.Peek() < time_limit)
	{
		merge_rolex.Reset().Resume();
		LastArcMerge(q[0], lbl_[1].U);
		*log.merge_time += merge_rolex.Pause();
		// (kayros M5.2) A deadline-truncated merge must not report Finished:
		// "Finished" is the caller's proof that pricing was exhaustive.
		if (run_deadline_.Reached()) log.status = BLBStatus::TimeLimitReached;
	}
	
	if (S.size() >= solution_limit) log.status = BLBStatus::SolutionLimitReached;
	else if (log.status == BLBStatus::DidNotStart) log.status = BLBStatus::Finished;
	*log.time += rolex.Pause();
	
	// Add solutions from the pool to the return vector R.
	for (auto& V_r: S)
	{
		// (kayros M5.2) Repricing the pool (one DP per route) is post-deadline
		// work on a doomed run once the TL fired — cut the tail. The routes
		// lost here would never be used: the caller terminates on the TL.
		if (run_deadline_.Reached()) break;
		Route& r = V_r.second;
		// Compute r actual duration (nyr::RouteDuration -> goc::Route).
		auto best = vrp_.BestDurationRoute(r.path);
		R->push_back(Route(best.path, best.t0, best.value));
	}
	
	return log;
}

void BidirectionalLabeling::IterativeMerge(Label* l, const MonodirectionalLabeling::DominanceStructure& L)
{
	TimeUnit T = vrp_.T;
	for (auto& demand_entry : L[l->v])
	{
		if (run_deadline_.Reached()) break; // (kayros M5.2)
		if (S.size() >= solution_limit) break; // Do not exceed solution limit.
		if (epsilon_bigger(demand_entry.first+l->q-vrp_.q[l->v], vrp_.Q)) break;
		for (auto& m: demand_entry.second)
		{
			if (S.size() >= solution_limit) break; // Do not exceed solution limit.
			if (epsilon_bigger_equal(m->min_cost+l->min_cost+pp_.P[l->v] + l->cut_cost - l->parent->cut_cost, 0.0)) break;
			Merge(l, m);
		}
	}
}

void BidirectionalLabeling::LastArcMerge(LBQueue& qf, const MonodirectionalLabeling::DominanceStructure& Lb)
{
	TimeUnit T = vrp_.T;
	
	// Create M_ijq structure.
	Matrix<VectorMap<CapacityUnit, vector<Label*>>> M(vrp_.D.NbVertices(), vrp_.D.NbVertices());
	for (Vertex v: vrp_.D.Vertices())
		for (auto& entry: Lb[v])
			for (auto& m: entry.second)
				insert_sorted(M[m->v][m->parent->v].Insert(entry.first, {}), m, [] (Label* m1, Label* m2) { return m1->min_cost < m2->min_cost; });
	
	while (!qf.empty())
	{
		if (run_deadline_.Reached()) break; // (kayros M5.2)
		LazyLabel ll = qf.top();
		qf.pop();
		Label* l = ll.parent;
		if (S.size() >= solution_limit) continue;
		
		for (auto& entry: M[ll.parent->v][ll.v])
		{
			if (epsilon_bigger(entry.first + l->q - vrp_.q[l->v], vrp_.Q)) break;
			if (S.size() >= solution_limit) break; // Do not exceed solution limit.
			for (Label* m: entry.second)
			{
				if (S.size() >= solution_limit) break; // Do not exceed solution limit.
				if (epsilon_bigger_equal(m->min_cost+l->min_cost+pp_.P[l->v] + l->cut_cost - l->parent->cut_cost, 0.0)) break;
				Merge(l, m);
			}
		}
	}
}

void BidirectionalLabeling::Merge(Label* l, Label* m)
{
	TimeUnit T = vrp_.T;
	
	if (epsilon_bigger(min(l->rw), T-min(m->rw))) return;
	if (intersection(l->S, m->S) != create_bitset<MAX_N>({l->v})) return;
	
	Route r;
	// kayros (M5.7): rw and dom(duration) can disagree by mollifier dust
	// (continuize_value_jumps nudges reverse-arrival domain boundaries by
	// <= 1e-3, beyond goc EPS); clamp boundary evaluations into the domain.
	// Search arithmetic only — columns are repriced checker-exactly (stage A).
	auto duration_at = [](const Label* x, double t) {
		t = std::max(min(dom(x->duration)), std::min(t, max(dom(x->duration))));
		return x->duration.Value(t);
	};
	// Merge l and m duration functions lm_d(t) = l_d(t) + m_d(T-t).
	if (epsilon_bigger_equal(T-max(m->rw), max(l->rw)))
	{
		r.duration = duration_at(l, max(l->rw)) + duration_at(m, max(m->rw)) + (T-max(m->rw)) - max(l->rw);
	}
	else
	{
		// M5.9 (design memo 12.2): m->duration(T - t) via the exact graph
		// reflection FlipTime, not Compose(T - Id). One IEEE subtraction per
		// breakpoint (platform-stable, no epsilon/PreValue arithmetic), and
		// value jumps keep their attained endpoints through the reflection.
		PWLFunction lm_duration = l->duration + m->duration.FlipTime(T);
		if (lm_duration.Empty()) return;
		r.duration = min(img(lm_duration));
	}
	
	double merge_cut_cost = 0.0;
	for (int i = 0; i < pp_.S.size(); ++i) if (l->parent->cut_visited[i]+m->cut_visited[i] >= 2) merge_cut_cost += pp_.sigma[i];
	double merge_cost = r.duration - l->p - m->p + pp_.P[l->v] - merge_cut_cost;
	if (epsilon_bigger_equal(merge_cost, 0.0)) return;
	
	// Merge l and m paths.
	r.path = l->Path();
	for (Label* x = m->parent; x->parent != nullptr; x = x->parent) r.path.push_back(x->v);
	if (r.path[0] != vrp_.o) r.path = reverse(r.path);
	
	// We have a negative reduced cost route r.
	AddSolution(r.path, r.duration);
}

void BidirectionalLabeling::AddSolution(const goc::GraphPath& p, double min_duration)
{
	VertexSet V = create_bitset<MAX_N>(p);
	if (!includes_key(S, V)) S[V] = Route({}, 0.0, INFTY);
	if (S[V].duration > min_duration) S[V] = Route(p, 0.0, min_duration);
}

void BidirectionalLabeling::setup_labeling_level_flag(
    LabelingLevel level
) {
    cost_check_relaxation = false;
    elementary_check_relaxation = false;
    ng_routes_relaxation = false;

    switch (level)
    {
        case LabelingLevel::HeuristicCost:
            cost_check_relaxation = true;
            break;
        case LabelingLevel::HeuristicElementarity:
            elementary_check_relaxation = true;
            break;
        case LabelingLevel::HeuristicNG:
            ng_routes_relaxation = true;
            break;
        case LabelingLevel::Exact:
            // No relaxation flags are set.
            break;
    }
}
} // namespace