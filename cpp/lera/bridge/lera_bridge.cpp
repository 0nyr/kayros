// kayros-owned bridge to the vendored Lera-Romero BPC (NOT vendored code).
#include "lera_bridge.h"

#include <optional>
#include <vector>

#include <goc/goc.h>
#include <nyr/nyr.h>

#include "bcp/bcp.h"
#include "bcp/pricing_problem.h"
#include "bcp/spf.h"
#include "labeling/bidirectional_labeling.h"
#include "labeling/labeling_level.h"
#include "preprocess/preprocess_capacity.h"
#include "preprocess/preprocess_service_waiting.h"
#include "preprocess/preprocess_time_windows.h"
#include "preprocess/preprocess_triangle_depot.h"

using namespace std;
using namespace goc;
using namespace solver;

namespace kayros::lera {

std::string solve_duration_json(const std::string& payload, const SolveParams& params) {
    nlohmann::json inst = nlohmann::json::parse(payload);

    // Lera's shared preprocessing. travel_times are already in the payload
    // (derived from MAMUT ATF sidecars), so the per-benchmark travel-time
    // construction steps are skipped.
    clog << "Preprocessing (MAMUT bridge)..." << endl;
    preprocess_capacity(inst);
    preprocess_service_waiting(inst);
    preprocess_time_windows(inst);
    preprocess_triangle_depot(inst);

    nyr::VRPInstance vrp = inst.get<nyr::VRPInstance>();

    // Create SPF and add initial routes (o, i, d).
    SPF spf(vrp.D.NbVertices());
    for (Vertex i : exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) {
        auto r = vrp.BestDurationRoute({vrp.o, i, vrp.d});
        spf.AddRoute(Route(r.path, r.t0, r.value));
    }

    BCP bcp(vrp.D, &spf);
    bcp.time_limit = Duration(params.time_limit_s, DurationUnit::Seconds);
    bcp.cut_limit = params.cut_limit;
    bcp.node_limit = params.node_limit;

    // Warm start (M5.3): reprice each heuristic route under Lera arithmetic
    // (never the checker value — a tighter-by-dust UB could prune the true
    // optimum) and add it as an initial column. Routes that fail Lera
    // repricing (boundary dust or preprocessing) are skipped; the UB is only
    // set when the added routes still partition the customers exactly.
    nlohmann::json warm_log;
    if (!params.initial_routes.empty()) {
        vector<int> cover(vrp.D.NbVertices(), 0);
        double ub_value = 0.0;
        int added = 0;
        bool valid_ub = true;
        for (const auto& customers : params.initial_routes) {
            GraphPath path;
            path.push_back(vrp.o);
            for (int c : customers) path.push_back(c);
            path.push_back(vrp.d);
            auto r = vrp.BestDurationRoute(path);
            if (r.value == INFTY) { valid_ub = false; continue; }
            spf.AddRoute(Route(r.path, r.t0, r.value));
            ++added;
            ub_value += r.value;
            for (int c : customers) ++cover[c];
        }
        for (Vertex v : exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) valid_ub &= cover[v] == 1;
        if (valid_ub && added > 0) bcp.SetInitialIncumbent(ub_value);
        warm_log["routes_given"] = params.initial_routes.size();
        warm_log["routes_added"] = added;
        if (valid_ub && added > 0) warm_log["ub"] = ub_value;
        clog << "Warm start: " << added << "/" << params.initial_routes.size()
             << " columns added" << (valid_ub && added > 0 ? ", UB set" : ", no UB") << endl;
    }

    nlohmann::json incumbents = nlohmann::json::array();
    bcp.on_incumbent = [&](Duration elapsed, const VRPSolution& sol, const std::string& origin) {
        nlohmann::json entry = {
            {"time", elapsed.Amount(DurationUnit::Seconds)},
            {"value", sol.value},
            {"origin", origin},
            {"routes", sol.routes},
        };
        if (params.on_incumbent) params.on_incumbent(entry.dump());
        incumbents.push_back(std::move(entry));
    };

    // Labeling pricing with the 3-level heuristic ladder of Lera's original
    // driver (relax cost -> relax elementarity -> exact); NG-routes off.
    const std::optional<TDNGRoutesParams> ng_params = std::nullopt;
    BidirectionalLabeling lbl(vrp, ng_params);
    lbl.solution_limit = params.solution_limit;
    lbl.closing_state = false;

    const vector<LabelingLevel> levels = {
        LabelingLevel::HeuristicCost, LabelingLevel::HeuristicElementarity, LabelingLevel::Exact};
    const vector<string> level_names = {"Heuristic Cost", "Heuristic Elementarity", "Exact"};
    size_t heuristic_level = 0;
    bcp.pricing_solver = [&](const PricingProblem& pricing_problem, int /*node_number*/,
                             Duration /*tlimit*/, CGExecutionLog* cg_execution_log) {
        vector<Route> R;
        while (heuristic_level < levels.size()) {
            // (M5.2) Residual budget from the BCP's single absolute deadline,
            // not from a per-call stopwatch chain.
            lbl.time_limit = bcp.deadline.Remaining();
            auto lbl_log = lbl.Run(pricing_problem, &R, levels[heuristic_level]);

            // Add iteration log.
            cg_execution_log->iterations->push_back(lbl_log);
            cg_execution_log->iterations->back()["iteration_name"] = level_names[heuristic_level];

            // Update merge_start and closing_state.
            lbl.closing_state |=
                heuristic_level == levels.size() - 1 && lbl_log.status == BLBStatus::Finished;
            lbl.merge_start = (lbl.merge_start + lbl_log.forward_log->processed_count) / 2;

            if (!R.empty()) break;
            ++heuristic_level;
        }
        // Add negative reduced cost routes. (M5.2) The adds are deadline-
        // checked per route: a full pool is thousands of ~1 ms formulation
        // inserts, the dominant non-interruptible tail otherwise. Once the
        // deadline fired the CG status is stamped so a truncated pricing pass
        // can never masquerade as "no new columns = node optimal".
        for (auto& r : R) {
            if (bcp.deadline.Reached()) break;
            spf.AddRoute(r);
        }
        if (bcp.deadline.Reached())
            cg_execution_log->status = CGStatus::TimeLimitReached;
        if (heuristic_level >= levels.size()) {
            heuristic_level = 0;
            lbl.closing_state = false;
            lbl.merge_start = 0;
        }
    };

    VRPSolution solution(INFTY, {});
    auto log = bcp.Run(&solution);

    nlohmann::json result;
    result["exact_log"] = log;
    result["incumbents"] = incumbents;
    if (!warm_log.is_null()) result["warm_start"] = warm_log;
    if (solution.value != INFTY) {
        result["value"] = solution.value;
        result["routes"] = solution.routes;
    }
    return result.dump();
}

}  // namespace kayros::lera
