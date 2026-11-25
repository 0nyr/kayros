//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "labeling/bidirectional_labeling.h"

#include "bcp/pricing_problem.h"

#include <climits>
#include <algorithm>
#include <cstdlib>
#include <stdexcept>
#include <string>

#include "labeling/trace.h"

using namespace std;
using namespace goc;
using namespace nyr;

namespace solver
{
namespace
{
// Build gap-free reverse departure functions for jump-free solves. Reflection can yield verticals from departure plateaus and can also bring differently rounded pieces to the same abscissa. Represent coincident breakpoints by a steep bridge of width at most 1e-3 below the following piece.
// The Vu2020 reverse-side regression tests exercise this path. See NOTICE item 9 for the comparison with unmodified reverse functions.
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
			// Compute reverse travel functions. The
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
			// The reverse-side normalizer is load-bearing and unconditional here.
			// Time-reversal turns a left-continuous forward step function into a
			// right-continuous reverse one; the labeling assumes uniform
			// left-continuity, so exact reverse verticals misprice at jumps. It is
			// NOT a no-op on jump-free arcs either: it flattens the CHOICE
			// verticals that Inverse/FlipTime leave in the reflected arrival, which
			// is the invariant the legacy (non-exact) arithmetic requires. Only the
			// exact value-jump path skips it, and that path carries verticals on
			// purpose. See the helper's header before touching this.
			if (!goc::step_exact_arithmetic()) // Step-carrying input keeps its tagged verticals.
				r.arr[v][u] = continuize_value_jumps(r.arr[v][u]); // Kayros: see above.
			r.tau[v][u] = r.arr[v][u] - PWLFunction::IdentityFunction({0.0, vrp.T});
			try { r.dep[v][u] = r.arr[v][u].Inverse(); }
			catch (const goc::BareJumpError& e)
			{
				// See VRPInstance from_json: never bridge, name the arc.
				throw std::domain_error("reverse arrival function of arc (" + std::to_string(u) + ", " + std::to_string(v) + "): " + e.what());
			}
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
	packed_filter = 0;
	merge_prefilter = false;
	merge_prefilter_hits = 0;
	heuristic_cut_blind = false;
	last_arc_merge_iterate = false;
	near_threshold = 0.0;
	near_limit = 0;
	near_pool_cap = 200000;
	last_negative_count = last_near_count = 0;
	time_limit = Duration::Max();
	screen_output = nullptr;
	closing_state = true;
	merge_start = 0;
	lbl_[0].process_limit = lbl_[1].process_limit = 10;
	lbl_[0].cross = false, lbl_[1].cross = true;
	partial = limited_extension = lazy_extension = unreachable_strengthened = sort_by_cost = true;
	elementary_check_relaxation = cost_check_relaxation = ng_routes_relaxation = false;
	correcting = true;
	step_pool_ = false;
	if (goc::step_exact_arithmetic())
	{
		for (Vertex u: vrp_.D.Vertices())
		{
			for (Vertex v: vrp_.D.Successors(u))
			{
				for (int k = 0; k < vrp_.dep[u][v].PieceCount(); ++k)
					if (vrp_.dep[u][v].Piece(k).is_vertical()) { step_pool_ = true; break; }
				if (step_pool_) break;
			}
			if (step_pool_) break;
		}
	}
	if (trace::merge_on())
	{
		fprintf(stderr, "TRC CTOR step_pool=%d\n", (int) step_pool_);
		// Dump arr/dep pieces of every arc along the merge target (dev tool).
		const auto& t = trace::merge_target();
		for (size_t k = 0; k + 1 < t.size(); ++k)
		{
			Vertex u = t[k], v = t[k + 1];
			if (!vrp_.D.IncludesArc({u, v})) continue;
			auto dump = [&](const char* name, const PWLFunction& f) {
				for (int p = 0; p < f.PieceCount(); ++p)
				{
					const LinearFunction& pc = f.Piece(p);
					fprintf(stderr, "TRC ARC %s[%d][%d] piece=%d dom=[%.6f,%.6f] img=[%.6f,%.6f] %s att=%.6f\n",
						name, (int) u, (int) v, p, pc.domain.left, pc.domain.right,
						pc.image.left, pc.image.right,
						pc.is_vertical() ? (pc.is_choice_vertical() ? "C" : "J") : "-",
						pc.is_vertical() ? pc.intercept : pc.Value(pc.domain.left));
				}
			};
			dump("arr", vrp_.arr[u][v]);
			dump("dep", vrp_.dep[u][v]);
		}
	}
}

BLBExecutionLog BidirectionalLabeling::Run(
    const PricingProblem& pricing_problem, 
    std::vector<Route>* R,
    LabelingLevel level
) {
	if (correcting && !lazy_extension)
		throw std::invalid_argument("Label correction requires lazy extension");

	// Clean solution pool.
	S.clear();
	S_paths.clear(); // Path-keyed pool (step-carrying instances)
	S_rc.clear();
	last_pool_durations.clear();
	last_empty_negative_routes = last_empty_near_routes = 0;
	negative_count_ = near_count_ = 0;
	near_eff_ = (!step_pool_ && near_threshold > 0.0 && near_limit > 0) ? near_threshold : 0.0;
	last_negative_count = last_near_count = 0;
	M[0] = M[1] = vector<MonodirectionalLabeling::DemandLevel>(vrp_.D.NbVertices());
	
	// Set pricing problem.
	vrp_.D.AddArcs(pp_.A); // Add previously forbidden arcs.
	pp_ = pricing_problem;
	vrp_.D.RemoveArcs(pp_.A); // Remove pricing problem forbidden arcs.
	
	// Init forward and backward labeling.
	lbl_[0].SetProblem(pp_);
	lbl_[1].SetProblem(reverse_pricing_problem(pp_));
	lbl_[0].t_m = lbl_[1].t_m = vrp_.T;
	
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
	lbl_[0].packed_filter = lbl_[1].packed_filter = packed_filter;
	lbl_[0].cut_blind_dominance = lbl_[1].cut_blind_dominance =
		heuristic_cut_blind && level != LabelingLevel::Exact && (cost_check_relaxation || elementary_check_relaxation);
	
	BLBExecutionLog log(true);
	Stopwatch rolex(false), merge_rolex(false);
	run_deadline_ = Deadline::In(time_limit);
	
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
			if (pool_size() >= solution_limit) { log.status = BLBStatus::SolutionLimitReached; break; } // Check if SLim is reached.
			// Kayros: sticky memory watermark; also catches a
			// monodirectional Run below that broke on MemoryLimitReached.
			if (MemoryMonitor::Exceeded()) { log.status = BLBStatus::MemoryLimitReached; break; }
			lbl_[d].time_limit = time_limit - rolex.Peek(); // Set time limit.
			auto P = lbl_[d].Run(&q[d], mlb_log[d]);
			if (correcting)
			{
				merge_rolex.Reset().Resume();
				RefreshCorrectedMergeLabels(d);
				*log.merge_time += merge_rolex.Pause();
			}
			
			// If iterative-merge is enabled, then add the labels to the structure.
			if (!closing_state)
				for (Label* l: P)
				{
					if (correcting && l->duration.Empty()) continue;
					insert_sorted(M[d][l->v].Insert(floor(l->q), {}), l, [] (Label* l, Label* m) { return l->min_cost < m->min_cost; });
				}
			
			// If iterative-merge is enabled, then try to merge.
			if (!closing_state && log.forward_log->processed_count >= merge_start)
			{
				merge_rolex.Reset().Resume();
				for (Label* l: P)
				{
					if (run_deadline_.Reached()) break;
					IterativeMerge(l, M[od]);
				}
				*log.merge_time += merge_rolex.Pause();
			}
			
			// Check if any full route was generated.
			for (Label* l: P)
				if (d == 0 && l->v == vrp_.d && epsilon_smaller(l->min_cost, 0.0))
					AddSolution(l->Path(), min(img(l->duration)), l->min_cost);
			
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
					 STR(mlb_log[0]->processed_count), STR(mlb_log[1]->processed_count), STR(pool_size()),
					 STR(lbl_[0].t_m), STR(vrp_.T-lbl_[1].t_m), STR(q[0].size()), STR(q[1].size())});
		}
	}
	
	// Last-edge merge.
	// Kayros: a memory-tripped run must not start the merge tail.
	if (pool_size() < solution_limit && rolex.Peek() < time_limit && log.status != BLBStatus::MemoryLimitReached)
	{
		merge_rolex.Reset().Resume();
		LastArcMerge(q[0], lbl_[1].U);
		*log.merge_time += merge_rolex.Pause();
		// A deadline-truncated merge must not report Finished:
		// "Finished" is the caller's proof that pricing was exhaustive.
		if (run_deadline_.Reached()) log.status = BLBStatus::TimeLimitReached;
	}
	
	// Kayros: MemoryLimitReached takes precedence: "Finished" or a
	// full pool would let the caller read a truncated pricing as exhaustive.
	if (log.status != BLBStatus::MemoryLimitReached)
	{
		if (pool_size() >= solution_limit) log.status = BLBStatus::SolutionLimitReached;
		else if (log.status == BLBStatus::DidNotStart) log.status = BLBStatus::Finished;
	}
	*log.time += rolex.Pause();
	
	// Add solutions from the pool to the return vector R.
	// Repricing the pool (one DP per route) is post-deadline
	// work on a doomed run once the TL fired — cut the tail. The routes
	// lost here would never be used: the caller terminates on the TL.
	auto reprice_into_R = [&](const Route& r, bool negative) {
		// The pool can contain an unresolved INFTY placeholder without a path.
		if (r.path.size() < 3) return;
		// Compute r actual duration (nyr::RouteDuration -> goc::Route).
		auto best = vrp_.BestDurationRoute(r.path);
		if (best.path.empty()) {
			if (negative) ++last_empty_negative_routes;
			else ++last_empty_near_routes;
			return;
		}
		// Trace: fate of the merge target through the pool repricing.
		if (trace::merge_on() && r.path == trace::merge_target())
			fprintf(stderr, "TRC POOL-EXIT path_len=%zu pool_dur=%.6f best_dur=%.6f best_empty=%d\n",
				r.path.size(), r.duration, best.value, (int) best.path.empty());
		R->push_back(Route(best.path, best.t0, best.value));
		last_pool_durations.push_back(r.duration);
	};
	if (step_pool_)
	{
		for (auto& P_r: S_paths)
		{
			if (run_deadline_.Reached()) break;
			reprice_into_R(P_r.second, true);
		}
	}
	else if (near_eff_ > 0.0)
	{
		// Negative routes first (pool iteration order), then the best
		// near-zero ones by labeling reduced cost.
		vector<pair<double, const Route*>> near;
		for (auto& V_r: S)
		{
			if (run_deadline_.Reached()) break;
			if (epsilon_smaller(S_rc[V_r.first], 0.0)) reprice_into_R(V_r.second, true);
			else near.push_back({S_rc[V_r.first], &V_r.second});
		}
		last_negative_count = (int) R->size();
		if ((int) near.size() > near_limit)
		{
			std::nth_element(near.begin(), near.begin() + near_limit, near.end(),
				[](const pair<double, const Route*>& a, const pair<double, const Route*>& b) { return a.first < b.first; });
			near.resize(near_limit);
		}
		std::sort(near.begin(), near.end(),
			[](const pair<double, const Route*>& a, const pair<double, const Route*>& b) {
				return a.first < b.first || (a.first == b.first && a.second->path < b.second->path); });
		for (auto& n: near)
		{
			if (run_deadline_.Reached()) break;
			reprice_into_R(*n.second, false);
		}
		last_near_count = (int) R->size() - last_negative_count;
	}
	else
	{
		for (auto& V_r: S)
		{
			if (run_deadline_.Reached()) break;
			reprice_into_R(V_r.second, true);
		}
	}
	if (near_eff_ <= 0.0) last_negative_count = (int) R->size();
	return log;
}

void BidirectionalLabeling::RefreshCorrectedMergeLabels(int direction)
{
	auto& changed = lbl_[direction].corrected_merge_buckets_;
	if (!closing_state)
	{
		std::sort(changed.begin(), changed.end());
		changed.erase(std::unique(changed.begin(), changed.end()), changed.end());
		for (const auto& [vertex, demand] : changed)
			for (auto& entry : M[direction][vertex])
			{
				if (entry.first < demand) continue;
				if (entry.first > demand) break;
				auto& labels = entry.second;
				labels.erase(std::remove_if(labels.begin(), labels.end(),
					[](Label* label) { return label->duration.Empty(); }), labels.end());
				std::stable_sort(labels.begin(), labels.end(),
					[](Label* a, Label* b) { return a->min_cost < b->min_cost; });
				break;
			}
	}
	changed.clear();
}

void BidirectionalLabeling::IterativeMerge(Label* l, const MonodirectionalLabeling::DominanceStructure& L)
{
	if (correcting && l->duration.Empty()) return;
	TimeUnit T = vrp_.T;
	for (auto& demand_entry : L[l->v])
	{
		if (run_deadline_.Reached()) break;
		if (pool_size() >= solution_limit) break; // Do not exceed solution limit.
		if (epsilon_bigger(demand_entry.first+l->q-vrp_.q[l->v], vrp_.Q)) break;
		for (auto& m: demand_entry.second)
		{
			if (pool_size() >= solution_limit) break; // Do not exceed solution limit.
			if (epsilon_bigger_equal(m->min_cost+l->min_cost+pp_.P[l->v] + l->cut_cost - l->parent->cut_cost, near_eff_)) break;
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
	
	// See last_arc_merge_iterate. The pool never shrinks, so once the
	// solution limit is reached nothing further can be merged.
	struct QueueStorage : LBQueue
	{
		static const std::vector<LazyLabel>& of(const LBQueue& q) { return q.*&QueueStorage::c; }
	};
	const std::vector<LazyLabel>* storage = last_arc_merge_iterate ? &QueueStorage::of(qf) : nullptr;
	size_t storage_index = 0;
	while (storage ? storage_index < storage->size() : !qf.empty())
	{
		if (run_deadline_.Reached()) break;
		LazyLabel ll = storage ? (*storage)[storage_index++] : qf.top();
		if (!storage) qf.pop();
		Label* l = ll.parent;
		if (correcting && l->duration.Empty()) continue;
		if (pool_size() >= solution_limit) { if (storage) break; continue; }
		
		// Trace: is this pop the merge target's frontier arc?
		bool tr_arc = trace::merge_fwd_prefix(l);
		if (tr_arc)
			fprintf(stderr, "TRC LAM-POP l=%s v=%d l_min_cost=%.6f\n",
				trace::path_str(l).c_str(), (int) ll.v, l->min_cost);
		for (auto& entry: M[ll.parent->v][ll.v])
		{
			// Trace: report the witness backward label's standing in this
			// pool even when the sorted cost break would stop before reaching it.
			if (tr_arc)
				for (Label* m: entry.second)
					if (trace::merge_bwd_suffix(m))
						fprintf(stderr, "TRC LAM-WITNESS l=%s m=%s q_entry=%.1f m_min_cost=%.6f l_min_cost=%.6f P=%.6f sum=%.6f q_break=%d\n",
							trace::path_str(l).c_str(), trace::path_str(m).c_str(),
							entry.first, m->min_cost, l->min_cost, pp_.P[l->v],
							m->min_cost+l->min_cost+pp_.P[l->v] + l->cut_cost - l->parent->cut_cost,
							(int) epsilon_bigger(entry.first + l->q - vrp_.q[l->v], vrp_.Q));
			if (epsilon_bigger(entry.first + l->q - vrp_.q[l->v], vrp_.Q))
			{
				if (tr_arc) fprintf(stderr, "TRC LAM-QBREAK l=%s v=%d q_entry=%.1f\n",
					trace::path_str(l).c_str(), (int) ll.v, entry.first);
				break;
			}
			if (pool_size() >= solution_limit) break; // Do not exceed solution limit.
			for (Label* m: entry.second)
			{
				if (pool_size() >= solution_limit) break; // Do not exceed solution limit.
				if (epsilon_bigger_equal(m->min_cost+l->min_cost+pp_.P[l->v] + l->cut_cost - l->parent->cut_cost, near_eff_))
				{
					if (tr_arc && trace::merge_bwd_suffix(m))
						fprintf(stderr, "TRC LAM-CBREAK l=%s m=%s m_min_cost=%.6f l_min_cost=%.6f P=%.6f sum=%.6f\n",
							trace::path_str(l).c_str(), trace::path_str(m).c_str(),
							m->min_cost, l->min_cost, pp_.P[l->v],
							m->min_cost+l->min_cost+pp_.P[l->v] + l->cut_cost - l->parent->cut_cost);
					break;
				}
				Merge(l, m);
			}
		}
	}
}

void BidirectionalLabeling::Merge(Label* l, Label* m)
{
	if (correcting && (l->duration.Empty() || m->duration.Empty())) return;
	TimeUnit T = vrp_.T;

	// Trace: full arithmetic of the witness pair's merge attempt.
	bool tr = trace::merge_matches(l, m);
	if (epsilon_bigger(min(l->rw), T-min(m->rw)))
	{
		if (tr) fprintf(stderr, "TRC MERGE-RET l=%s m=%s reason=rw-disjoint l_rw_min=%.6f T-m_rw_min=%.6f\n",
			trace::path_str(l).c_str(), trace::path_str(m).c_str(), min(l->rw), T-min(m->rw));
		return;
	}
	if (intersection(l->S, m->S) != create_bitset<MAX_N>({l->v}))
	{
		if (tr) fprintf(stderr, "TRC MERGE-RET l=%s m=%s reason=S-overlap\n",
			trace::path_str(l).c_str(), trace::path_str(m).c_str());
		return;
	}
	
	// Cut penalty of the merged route, needed by the prefilter below
	// and by the cost test at the end.
	double merge_cut_cost = 0.0;
	for (int i = 0; i < pp_.S.size(); ++i) if (l->parent->cut_visited[i]+m->cut_visited[i] >= 2) merge_cut_cost += pp_.sigma[i];
	if (merge_prefilter && !tr)
	{
		// min(img(duration)) of each label, recovered from its min_cost. The
		// merged duration is at least their sum in both branches below (the
		// waiting term is non-negative, and the minimum of a sum is at least
		// the sum of the minima). The 1e-5 margin covers the rounding of
		// these recoveries and the epsilon of the waiting test.
		const double duration_bound = (l->min_cost + l->p + l->cut_cost) + (m->min_cost + m->p + m->cut_cost);
		const double cost_bound = duration_bound - l->p - m->p + pp_.P[l->v] - merge_cut_cost;
		if (epsilon_bigger_equal(cost_bound - 1e-5, near_eff_)) { ++merge_prefilter_hits; return; }
	}

	Route r;
	// The label window and its duration domain can differ at a rounded boundary. Clamp the evaluation to the domain before taking the minimum over departure choices. Master columns are priced separately with the binary64 reference fold.
	auto duration_at = [](const Label* x, double t) {
		t = std::max(min(dom(x->duration)), std::min(t, max(dom(x->duration))));
		// Minimum over the departure-choice set (see
		// monodirectional duration_at).
		return x->duration.MinValueAt(t);
	};
	// Merge l and m duration functions lm_d(t) = l_d(t) + m_d(T-t).
	if (epsilon_bigger_equal(T-max(m->rw), max(l->rw)))
	{
		r.duration = duration_at(l, max(l->rw)) + duration_at(m, max(m->rw)) + (T-max(m->rw)) - max(l->rw);
		if (tr) fprintf(stderr, "TRC MERGE-WAIT l=%s m=%s l_at=%.6f m_at=%.6f wait=%.6f dur=%.6f\n",
			trace::path_str(l).c_str(), trace::path_str(m).c_str(),
			duration_at(l, max(l->rw)), duration_at(m, max(m->rw)),
			(T-max(m->rw)) - max(l->rw), r.duration);
	}
	else
	{
		// M->duration(T - t) via the exact graph
		// reflection FlipTime, not Compose(T - Id). One IEEE subtraction per
		// breakpoint (platform-stable, no epsilon/PreValue arithmetic), and
		// value jumps keep their attained endpoints through the reflection.
		PWLFunction lm_duration = l->duration + m->duration.FlipTime(T);
		if (lm_duration.Empty())
		{
			if (tr) fprintf(stderr, "TRC MERGE-RET l=%s m=%s reason=empty-overlap l_rw=[%.6f,%.6f] m_flip_rw=[%.6f,%.6f]\n",
				trace::path_str(l).c_str(), trace::path_str(m).c_str(),
				min(l->rw), max(l->rw), T-max(m->rw), T-min(m->rw));
			return;
		}
		r.duration = min(img(lm_duration));
		if (tr) fprintf(stderr, "TRC MERGE-OVERLAP l=%s m=%s dur=%.6f l_rw=[%.6f,%.6f] m_flip_rw=[%.6f,%.6f]\n",
			trace::path_str(l).c_str(), trace::path_str(m).c_str(), r.duration,
			min(l->rw), max(l->rw), T-max(m->rw), T-min(m->rw));
	}

	double merge_cost = r.duration - l->p - m->p + pp_.P[l->v] - merge_cut_cost;
	if (tr) fprintf(stderr, "TRC MERGE-COST l=%s m=%s dur=%.6f merge_cost=%.6f -> %s\n",
		trace::path_str(l).c_str(), trace::path_str(m).c_str(), r.duration, merge_cost,
		epsilon_bigger_equal(merge_cost, 0.0) ? "REJECT" : "ADD");
	if (epsilon_bigger_equal(merge_cost, near_eff_)) return;
	
	// Merge l and m paths.
	r.path = l->Path();
	for (Label* x = m->parent; x->parent != nullptr; x = x->parent) r.path.push_back(x->v);
	if (r.path[0] != vrp_.o) r.path = reverse(r.path);
	
	// We have a negative reduced cost route r.
	AddSolution(r.path, r.duration, merge_cost);
}

void BidirectionalLabeling::AddSolution(const goc::GraphPath& p, double min_duration, double reduced_cost)
{
	// Trace: pool decisions for orderings of the merge target's vertex set.
	if (trace::merge_on()
		&& create_bitset<MAX_N>(p) == create_bitset<MAX_N>(trace::merge_target()))
	{
		std::string ps;
		for (auto v: p) ps += (ps.empty() ? "" : ",") + std::to_string(v);
		fprintf(stderr, "TRC POOL-ADD cand=%s dur=%.6f pool=%s\n",
			ps.c_str(), min_duration, step_pool_ ? "path" : "set");
	}
	if (step_pool_)
	{
		// Path-keyed pool (see the header comment): every distinct
		// ordering survives to the binary64 reference pool-exit repricing; the
		// bound only dedups re-merges of the SAME path.
		if (!includes_key(S_paths, p)) S_paths[p] = Route({}, 0.0, INFTY);
		if (S_paths[p].duration > min_duration) S_paths[p] = Route(p, 0.0, min_duration);
		return;
	}
	VertexSet V = create_bitset<MAX_N>(p);
	if (near_eff_ > 0.0)
	{
		// Same keying and same best-duration replacement rule as the
		// legacy pool; additionally track each entry's labeling reduced cost
		// so that the solution limit keeps counting negative routes only.
		const bool negative = epsilon_smaller(reduced_cost, 0.0);
		auto it = S.find(V);
		if (it == S.end())
		{
			if (!negative && (int) near_count_ >= near_pool_cap) return; // memory guard.
			S[V] = Route(p, 0.0, min_duration);
			S_rc[V] = reduced_cost;
			if (negative) ++negative_count_; else ++near_count_;
			return;
		}
		if (it->second.duration > min_duration)
		{
			double& rc = S_rc[V];
			const bool was_negative = epsilon_smaller(rc, 0.0);
			it->second = Route(p, 0.0, min_duration);
			if (reduced_cost < rc) rc = reduced_cost;
			if (!was_negative && epsilon_smaller(rc, 0.0)) { ++negative_count_; --near_count_; }
		}
		return;
	}
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
