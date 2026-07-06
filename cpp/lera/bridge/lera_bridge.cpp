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

    nlohmann::json incumbents = nlohmann::json::array();
    bcp.on_incumbent = [&](Duration elapsed, const VRPSolution& sol, const std::string& origin) {
        incumbents.push_back({
            {"time", elapsed.Amount(DurationUnit::Seconds)},
            {"value", sol.value},
            {"origin", origin},
            {"routes", sol.routes},
        });
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
                             Duration tlimit, CGExecutionLog* cg_execution_log) {
        Stopwatch iteration_rolex(true);
        vector<Route> R;
        while (heuristic_level < levels.size()) {
            lbl.time_limit = tlimit - iteration_rolex.Peek();
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
        // Add negative reduced cost routes.
        for (auto& r : R) spf.AddRoute(r);
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
    if (solution.value != INFTY) {
        result["value"] = solution.value;
        result["routes"] = solution.routes;
    }
    return result.dump();
}

}  // namespace kayros::lera
