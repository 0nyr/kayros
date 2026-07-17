//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef NETWORKS2019_BIDIRECTIONAL_LABELING_H
#define NETWORKS2019_BIDIRECTIONAL_LABELING_H

#include <map>
#include <vector>
#include <tuple>

#include <goc/goc.h>
#include <nyr/nyr.h>

#include "bcp/pricing_problem.h"
#include "label.h"
#include "lazy_label.h"
#include "monodirectional_labeling.h"
#include "labeling_level.h"

namespace solver
{

class BidirectionalLabeling
{
public:
	int solution_limit; // Maximum number of solutions to obtain.
	goc::Duration time_limit; // Maximum execution time.
	std::ostream* screen_output; // Output log to this stream (if nullptr, then no output is available).
	bool closing_state; // true if Closing state (last-edge merge), false if Opening state (iterative merge).
	int merge_start; // after <merge_start> forward labels have been processed, start iterative-merge.
	bool partial; // Indicates if partial domination should be used.
	bool limited_extension; // Indicates if limited extension should be applied.
	bool lazy_extension; // Indicates if lazy extension is used.
	bool unreachable_strengthened; // Indicates if the strengthened version of unreachable vertices is used.
	bool sort_by_cost; // Indicate if the last level sorting by cost strategy is used.
	bool correcting; // Indicates if the correcting step is executed.
	bool symmetric; // Indicates if symmetric bidirectional labeling should be applied (or asymmetric if false).
	
	// Resolution level-specific parameters.
	bool elementary_check_relaxation; // Indicates if dominance S(M) \subseteq S(L) should be ignored (heuristically).
	bool cost_check_relaxation; // Indicates if dominance c_M(t) <= c_L(t) should be ignored (heuristically).
	bool ng_routes_relaxation; // Indicates if NG-routes relaxation is be used.

	BidirectionalLabeling(
		const nyr::VRPInstance& vrp,
		const std::optional<TDNGRoutesParams>& ng_routes_params
	);
	
	// Runs the bidirectional labeling algorithm and leaves the negative reduced cost routes on the parameter R.
	// Returns: the execution information log.
	goc::BLBExecutionLog Run(
		const PricingProblem& pricing_problem, 
		std::vector<goc::Route>* R, 
		LabelingLevel level
	);
	
private:
	// Sets the labeling level flags based on the given level.
	void setup_labeling_level_flag(
		LabelingLevel level
	);

	// Attempts to merge label l against all the labels in the opposite direction dominance structure.
	// 	w: 	l will be merged with all labels m in L such that v(m) == v(l) and v(parent(m)) == w.
	// 		if w == -1, then the check v(parent(m)) == w is ignored.
	void IterativeMerge(Label* l, const MonodirectionalLabeling::DominanceStructure& L);
	
	void LastArcMerge(LBQueue& qf, const MonodirectionalLabeling::DominanceStructure& Lb);
	
	void Merge(Label* l, Label* m);

	// Adds a solution to the pool S if it is the best yet found with those visited vertices.
	void AddSolution(const goc::GraphPath& p, double min_duration);

	// (kayros M5.2) Deadline of the current Run(), derived from time_limit at
	// entry; consulted inside IterativeMerge/LastArcMerge so merge loops are
	// interruptible (they had no time checks and could overshoot unboundedly).
	goc::Deadline run_deadline_;

	nyr::VRPInstance vrp_;

	// NG-route stuff
	const std::optional<TDNGRoutesParams>& ng_params_; // Parameters for the NG-routes.

	PricingProblem pp_; // Pricing problem provided in the Run method to be used in the labeling algorithm.
	MonodirectionalLabeling lbl_[2]; // lbl_[0] = forward, lbl_[1] = backward.
	MonodirectionalLabeling::DominanceStructure M[2]; // Processed labels are stored in M[v][q] sorted by min_cost(l).
	
	// Pool of negative reduced cost solutions found (indexed by their visited vertices).
	// We only keep the best solution for each set of visited vertices.
	std::unordered_map<nyr::VertexSet, goc::Route> S;
	// M13.0: on step-carrying instances (any vertical in a forward tau, i.e.
	// the exact value-jump path) the pool is instead keyed by PATH. The
	// set-key kept one ordering per set ranked by the merge-time duration
	// BOUND, but exact-path bounds carry search dust, so a truly-cheaper
	// ordering could be shadowed by a bound-cheaper one forever (Rifki-17
	// n=50: witness ordering checker-duration 2957 shadowed by a 2975
	// ordering with a lower bound; the pool exit reprices only the stored
	// path and the master never saw the witness column: certified
	// 21331 > 21319). Path-keying lets every ordering reach the checker-exact
	// repricing; the bridge dedups by path fingerprint anyway. Jump-free and
	// mollified instances carry no verticals and keep the legacy pool
	// (container, order and size semantics bit-identical).
	bool step_pool_;
	std::map<goc::GraphPath, goc::Route> S_paths;
	size_t pool_size() const { return step_pool_ ? S_paths.size() : S.size(); }
};
} // namespace

#endif //NETWORKS2019_BIDIRECTIONAL_LABELING_H
