// kayros-owned bridge to the vendored Lera-Romero BPC (NOT vendored code).
#include "goc/linear_programming/solver/ablation_settings.h"
#include "lera_bridge.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <cstdlib>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <goc/goc.h>
#include <nyr/nyr.h>

#include "bcp/bcp.h"
#include "bcp/exact_cost.h"
#include "bcp/pricing_problem.h"
#include "bcp/spf.h"
#include "core/exact_route_pricer.h"
#include "core/instance.h"
#include "exact_columns.h"
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

namespace {

// Build the instance of the exact column evaluation from the original MAMUT arrival functions, windows and service times. Preprocessing shifts the windows and folds service into the travel times.
kayros::Instance build_checker_instance(const nlohmann::json& raw) {
    kayros::Instance ci;
    ci.num_customers = raw.at("num_customers").get<int32_t>();
    ci.num_vehicles = raw.at("num_vehicles").get<int32_t>();
    ci.vehicle_capacity = raw.at("vehicle_capacity").get<int64_t>();
    ci.horizon_start = raw.at("horizon")[0].get<double>();
    ci.horizon_end = raw.at("horizon")[1].get<double>();
    ci.has_time_windows = raw.at("has_time_windows").get<bool>();
    ci.demands = raw.at("demands").get<vector<int64_t>>();
    ci.service_times = raw.at("service_times").get<vector<double>>();
    if (ci.has_time_windows) {
        ci.tw_earliest = raw.at("tw_earliest").get<vector<double>>();
        ci.tw_latest = raw.at("tw_latest").get<vector<double>>();
    }
    const int64_t nv = ci.num_vertices();
    vector<vector<double>> xs(nv * nv), ys(nv * nv);
    for (const auto& e : raw.at("atfs")) {
        const int64_t a = e[0].get<int64_t>() * nv + e[1].get<int64_t>();
        xs[a] = e[2].get<vector<double>>();
        ys[a] = e[3].get<vector<double>>();
    }
    ci.atf_offset.assign(nv * nv + 1, 0);
    for (int64_t a = 0; a < nv * nv; ++a)
        ci.atf_offset[a + 1] = ci.atf_offset[a] + (int64_t)xs[a].size();
    ci.atf_xs.reserve(ci.atf_offset.back());
    ci.atf_ys.reserve(ci.atf_offset.back());
    for (int64_t a = 0; a < nv * nv; ++a) {
        ci.atf_xs.insert(ci.atf_xs.end(), xs[a].begin(), xs[a].end());
        ci.atf_ys.insert(ci.atf_ys.end(), ys[a].begin(), ys[a].end());
    }
    return ci;
}

}  // namespace

std::string solve_exact_json(const std::string& payload, const SolveParams& params) {
    const goc::BackendSettingsScope backend_scope;
    const goc::StepExactArithmeticScope step_mode(params.stepwise);

    nlohmann::json inst = nlohmann::json::parse(payload);

    // Preserve the original binary64 data before preprocessing mutates the payload. Every column of the master is evaluated on it by the exact header.
    const kayros::Instance checker_inst = build_checker_instance(inst.at("mamut_raw"));
    kayros::ExactRoutePricer pricer(checker_inst);
    inst.erase("mamut_raw");

    // Lera's shared preprocessing. travel_times are already in the payload
    // (derived from MAMUT ATF sidecars), so the per-benchmark travel-time
    // construction steps are skipped.
    clog << "Preprocessing (MAMUT bridge)..." << endl;
    preprocess_capacity(inst);
    preprocess_service_waiting(inst);
    preprocess_time_windows(inst);
    preprocess_triangle_depot(inst);

    nyr::VRPInstance vrp = inst.get<nyr::VRPInstance>();

    // Initialize the set-partitioning formulation with one singleton column per customer, at its exact cost. A singleton that the exact evaluation rejects does not enter. The artificial column of its customer covers it.
    // A duplicate path cannot improve the restricted master.
    unordered_set<string> column_keys;
    auto path_key = [](const GraphPath& p) {
        string k;
        for (goc::Vertex v : p) { k += to_string(v); k += ','; }
        return k;
    };

    SPF spf(vrp.D.NbVertices());
    int infeasible_singletons = 0;
    for (Vertex i : exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) {
        const GraphPath seed_path{vrp.o, i, vrp.d};
        const auto col = exact_column(pricer, checker_inst, seed_path);
        if (!col.feasible) { ++infeasible_singletons; continue; }
        column_keys.insert(path_key(seed_path));
        spf.AddRoute(Route(seed_path, col.departure, 0.0), col.cost);
    }

    BCP bcp(vrp.D, &spf);
    bcp.time_limit = Duration(params.time_limit, DurationUnit::Seconds);
    bcp.cut_limit = params.cut_limit;
    bcp.node_limit = params.node_limit;

    // Each native solve thread enforces its own limit against process RSS.
    const MemoryMonitor memory_scope((int64_t)(params.memory_limit_mb * 1048576.0));

    // Evaluate each warm-start path exactly, as every master column. Set the upper bound to the exact cost only if the feasible paths still partition the customers.
    nlohmann::json warm_log;
    if (!params.initial_routes.empty()) {
        vector<int> cover(vrp.D.NbVertices(), 0);
        ExactCost ub_value = 0;
        int added = 0;
        bool valid_ub = true;
        for (const auto& customers : params.initial_routes) {
            GraphPath path;
            path.push_back(vrp.o);
            for (int c : customers) path.push_back(c);
            path.push_back(vrp.d);
            const auto col = exact_column(pricer, checker_inst, path);
            if (!col.feasible) { valid_ub = false; continue; }
            column_keys.insert(path_key(path));
            spf.AddRoute(Route(path, col.departure, 0.0), col.cost);
            ++added;
            ub_value += col.cost;
            for (int c : customers) ++cover[c];
        }
        for (Vertex v : exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) valid_ub &= cover[v] == 1;
        if (valid_ub && added > 0) bcp.SetInitialIncumbent(ub_value);
        warm_log["routes_given"] = params.initial_routes.size();
        warm_log["routes_added"] = added;
        if (valid_ub && added > 0) {
            warm_log["ub"] = nearest_binary64(ub_value);
            warm_log["ub_exact"] = ub_value.get_str();
        }
        clog << "Warm start: " << added << "/" << params.initial_routes.size()
             << " columns added" << (valid_ub && added > 0 ? ", UB set" : ", no UB") << endl;
    }

    // 1 artificial column per customer, of cost M at least the cost of any solution, so that every restricted master has an LP optimum. The exact evaluation never accepts an integer solution that selects one. The master has no fleet row, so the exact evaluation of an incumbent enforces the fleet limit.
    const ExactCost artificial = artificial_cost(checker_inst);
    for (Vertex i : exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) spf.AddArtificial(i, artificial);
    bcp.fleet_limit = checker_inst.num_vehicles;

    int exact_calls = 0, exact_finished = 0;
    bool last_exact_finished = false, ladder_certified = false;
    int duplicate_columns = 0;
    auto closure_census = [&]() { return nlohmann::json{
        {"cg_optimum", bcp.cg_optimum_count}, {"cg_certified_final_pricing", bcp.cg_certified_count},
        {"cg_unresolved_final_pricing", bcp.cg_unresolved_count}, {"cg_infeasible_unverified", bcp.cg_infeasible_count},
        {"strong_rmp_infeasible_unverified", bcp.strong_rmp_infeasible_count},
        {"selected_rmp_infeasible_unverified", bcp.selected_rmp_infeasible_count},
        {"unbranched_fractional_unverified", bcp.unbranched_fractional_count}, {"duplicate_columns", duplicate_columns}}; };
    nlohmann::json bound_events = nlohmann::json::array();
    bcp.on_bound = [&](Duration elapsed, double bound, const std::string& source) {
        nlohmann::json entry = {{"event", source == "search_closed" ? "search_closed" : (source == "search_terminated" ? "search_terminated" : "global_bound")},
            {"time", elapsed.Amount(DurationUnit::Seconds)}, {"source", source},
            {"global_lb", bound > -INFTY && bound < INFTY ? nlohmann::json(bound) : nlohmann::json(nullptr)},
            {"exact_pricing", {{"calls", exact_calls}, {"finished", exact_finished}}}};
        entry["closure_census"] = closure_census();
        if (params.on_event) params.on_event(entry.dump());
        bound_events.push_back(std::move(entry));
    };
    nlohmann::json incumbents = nlohmann::json::array();
    bcp.on_incumbent = [&](Duration elapsed, const VRPSolution& sol, const std::string& origin) {
        nlohmann::json entry = {
            {"time", elapsed.Amount(DurationUnit::Seconds)},
            {"value", sol.value},
            {"exact_value", bcp.ExactIncumbentValue()->get_str()},
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

    constexpr double near_rel = 0.10;
    constexpr int near_limit = 3000;
    lbl.packed_filter = 1;
    lbl.merge_prefilter = true;
    lbl.last_arc_merge_iterate = true;
    const int verify_mask = params.verify_final_mask;
    if (verify_mask < 0 || verify_mask > 127)
        throw std::invalid_argument("verify_final_mask must be in [0, 127]");
    const bool verify_final = verify_mask != 0;
    int verify_final_runs = 0, verify_final_catches = 0;
    long long verify_final_columns = 0;
    long long near_columns_added = 0, near_columns_discarded = 0;
    const vector<LabelingLevel> levels = {
        LabelingLevel::HeuristicCost, LabelingLevel::HeuristicElementarity, LabelingLevel::Exact};
    const vector<string> level_names = {"Heuristic Cost", "Heuristic Elementarity", "Exact"};

    // Compare the labeling reduced cost with the reduced cost of the master cost (the nearest binary64 value of the exact cost) at the same customer and subset-row duals. A phantom has negative labeling reduced cost and positive master reduced cost above tolerance. Count it even when its path is already in the master.
    // The duration counters compare merge-time labeling and BestDurationRoute with the master cost. The separate duplicate and verification counters count master reduced costs below minus the integrity tolerance.
    const double integrity_eps = 1e-6;
    const PricingProblem* integrity_pp = nullptr;
    const std::vector<double>* integrity_pool = nullptr;
    bool integrity_verifying = false;
    long long phantom_columns = 0, phantom_duplicates = 0, labeling_below_checker = 0, route_duration_mismatches = 0, route_duration_below = 0;
    long long verify_negative_columns = 0, negative_duplicates = 0;
    double max_phantom_reduced_cost = 0.0, max_labeling_gap = 0.0, max_route_duration_gap = 0.0;
    auto checker_reduced_cost = [&](const GraphPath& path, double duration, const PricingProblem& pp) {
        double rc = duration;
        for (size_t k = 1; k + 1 < path.size(); ++k) rc -= pp.P[path[k]];
        for (size_t i = 0; i < pp.S.size(); ++i) {
            int visited = 0;
            for (size_t k = 1; k + 1 < path.size(); ++k) if (pp.S[i].test(path[k])) ++visited;
            if (visited >= 2) rc -= pp.sigma[i];
        }
        return rc;
    };

    long long empty_negative_routes = 0, empty_near_routes = 0;
    size_t heuristic_level = 0;
    int last_negative = 0; // leading negative routes in the last R.
    auto run_one_level = [&](const PricingProblem& pp, CGExecutionLog* cg_execution_log) {
        vector<Route> R;
        // Residual budget from the BCP's single absolute deadline,
        // not from a per-call stopwatch chain.
        lbl.time_limit = bcp.deadline.Remaining();
        if (heuristic_level == levels.size() - 1) ++exact_calls;
        // Near-zero collection only at the Exact level, scaled by the
        // mean positive customer dual so one setting serves every instance.
        lbl.near_threshold = 0.0;
        lbl.near_limit = near_limit;
        if (heuristic_level == levels.size() - 1) {
            double dual_sum = 0.0;
            int dual_count = 0;
            for (Vertex v : exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) {
                if (pp.P[v] > 0.0) dual_sum += pp.P[v];
                ++dual_count;
            }
            if (dual_count > 0) lbl.near_threshold = near_rel * dual_sum / dual_count;
        }
        lbl.heuristic_cut_blind = levels[heuristic_level] == LabelingLevel::HeuristicElementarity;
        integrity_pp = &pp;
        integrity_pool = &lbl.last_pool_durations;
        auto lbl_log = lbl.Run(pp, &R, levels[heuristic_level]);
        empty_negative_routes += lbl.last_empty_negative_routes;
        empty_near_routes += lbl.last_empty_near_routes;
        last_negative = lbl.last_negative_count;
        last_exact_finished = heuristic_level == levels.size() - 1 && lbl_log.status == BLBStatus::Finished;
        if (last_exact_finished) ++exact_finished;

        // Add iteration log.
        cg_execution_log->iterations->push_back(lbl_log);
        cg_execution_log->iterations->back()["iteration_name"] = level_names[heuristic_level];
        cg_execution_log->iterations->back()["pricing_negative_routes"] = lbl.last_negative_count;
        cg_execution_log->iterations->back()["pricing_near_routes"] = lbl.last_near_count;
        cg_execution_log->iterations->back()["pricing_near_threshold"] = lbl.near_threshold;
        cg_execution_log->iterations->back()["pricing_extended_labels"] = *lbl_log.forward_log->extended_count + *lbl_log.backward_log->extended_count;
        cg_execution_log->iterations->back()["pricing_bcp_elapsed_s"] =
            (bcp.time_limit - bcp.deadline.Remaining()).Amount(DurationUnit::Seconds);

        // Update merge_start and closing_state.
        lbl.closing_state |=
            heuristic_level == levels.size() - 1 && lbl_log.status == BLBStatus::Finished;
        lbl.merge_start = (lbl.merge_start + lbl_log.forward_log->processed_count) / 2;
        return R;
    };

    int checker_infeasible_count = 0; // Rejected by the exact column evaluation.


    // Both pricing passes deduplicate paths and evaluate each new column exactly. Poll time and memory per route so a large pool can be interrupted during insertion.
    auto add_columns = [&](const vector<Route>& R, size_t begin, size_t end, bool priced_negative) {
        int added = 0;
        for (size_t route_index = begin; route_index < end && route_index < R.size(); ++route_index) {
            const auto& r = R[route_index];
            if (bcp.deadline.Reached()) break;
            if (MemoryMonitor::Exceeded()) break;
            if (!column_keys.insert(path_key(r.path)).second) {
                ++duplicate_columns;
                if (priced_negative && integrity_pp) {
                    const auto dup = exact_column(pricer, checker_inst, r.path);
                    if (dup.feasible) {
                        const double rc = checker_reduced_cost(r.path, nearest_binary64(dup.cost), *integrity_pp);
                        if (rc > integrity_eps) { ++phantom_duplicates; max_phantom_reduced_cost = std::max(max_phantom_reduced_cost, rc); }
                        else if (rc < -integrity_eps) ++negative_duplicates;
                    }
                }
                continue;
            }
            const auto col = exact_column(pricer, checker_inst, r.path);
            if (!col.feasible)
            {
                // Skip the column and count it in pricing_integrity. A rejected negative-priced route leaves the closure diagnostic incomplete.
                ++checker_infeasible_count;
                clog << "Skipped a priced route that the exact evaluation rejects (path "
                     << STR(r.path) << ")" << endl;
                continue;
            }
            spf.AddRoute(Route(r.path, col.departure, 0.0), col.cost);
            ++added;
            if (priced_negative && integrity_pp) {
                const double duration = nearest_binary64(col.cost);
                const double rc = checker_reduced_cost(r.path, duration, *integrity_pp);
                if (rc > integrity_eps) { ++phantom_columns; max_phantom_reduced_cost = std::max(max_phantom_reduced_cost, rc); }
                if (integrity_verifying && rc < -integrity_eps) ++verify_negative_columns;
                const double route_gap = std::abs(r.duration - duration);
                if (route_gap > integrity_eps) { ++route_duration_mismatches; max_route_duration_gap = std::max(max_route_duration_gap, route_gap); }
                if (r.duration < duration - integrity_eps) ++route_duration_below;
                if (integrity_pool && route_index < integrity_pool->size()) {
                    const double gap = duration - (*integrity_pool)[route_index];
                    if (gap > integrity_eps) { ++labeling_below_checker; max_labeling_gap = std::max(max_labeling_gap, gap); }
                }
            }
        }
        return added;
    };

    // Ladder escalation is driven by NEW columns, not by a
    // non-empty pricing pool. A heuristic level can keep re-pricing an
    // EXISTING column (its labeling-arithmetic reduced cost slightly negative
    // while the master cost is not), the fingerprint dedup then
    // adds 0, and the old ladder broke without escalating — so the BCP read
    // "no new columns" as node-optimal WITHOUT the Exact level ever running:
    // unsound termination (the Rifki-14 k=15 reproducer certified 5606 with
    // pricing logs showing zero Exact iterations). Now the ladder climbs until
    // some level contributes new columns or the Exact level finishes empty.
    auto run_ladder_and_add = [&](const PricingProblem& pp, CGExecutionLog* lg) {
        int added = 0;
        ladder_certified = false;
        while (!bcp.deadline.Reached()) {
            if (MemoryMonitor::Exceeded()) break;
            const bool at_exact = heuristic_level + 1 >= levels.size();
            const vector<Route> R = run_one_level(pp, lg);
            const size_t negative = std::min<size_t>(R.size(), (size_t) std::max(last_negative, 0));
            added = add_columns(R, 0, negative, true);
            if (added > 0) {
                // Near-zero routes ride along with a productive pass
                // only. A pass without a new negative column discards them,
                // so "no new column" keeps its meaning for termination.
                if (negative < R.size()) near_columns_added += add_columns(R, negative, R.size(), false);
                break;
            }
            near_columns_discarded += (long long) (R.size() - negative);
            bool covered = last_exact_finished && lbl.last_empty_negative_routes == 0;
            if (at_exact && verify_final && last_exact_finished && !bcp.deadline.Reached()) {
                // Re-run the empty Exact pass with the masked features changed.
                const bool s_corr = lbl.correcting, s_closing = lbl.closing_state, s_sort = lbl.sort_by_cost,
                    s_unreach = lbl.unreachable_strengthened, s_partial = lbl.partial, s_limited = lbl.limited_extension,
                    s_prefilter = lbl.merge_prefilter;
                const int s_merge_start = lbl.merge_start, s_packed = lbl.packed_filter;
                if (verify_mask & 1) lbl.correcting = false;
                if (verify_mask & 2) { lbl.closing_state = false; lbl.merge_start = 0; }
                if (verify_mask & 4) lbl.sort_by_cost = false;
                if (verify_mask & 8) lbl.unreachable_strengthened = false;
                if (verify_mask & 16) { lbl.partial = false; lbl.correcting = false; }
                if (verify_mask & 32) lbl.limited_extension = false;
                if (verify_mask & 64) { lbl.packed_filter = 0; lbl.merge_prefilter = false; }
                ++verify_final_runs;
                const vector<Route> R2 = run_one_level(pp, lg);
                covered = covered && last_exact_finished && lbl.last_empty_negative_routes == 0;
                lbl.correcting = s_corr; lbl.closing_state = s_closing; lbl.sort_by_cost = s_sort;
                lbl.unreachable_strengthened = s_unreach; lbl.partial = s_partial; lbl.limited_extension = s_limited;
                lbl.merge_prefilter = s_prefilter; lbl.merge_start = s_merge_start; lbl.packed_filter = s_packed;
                const size_t negative2 = std::min<size_t>(R2.size(), (size_t) std::max(last_negative, 0));
                integrity_verifying = true;
                const int added2 = add_columns(R2, 0, negative2, true);
                integrity_verifying = false;
                if (added2 > 0) {
                    ++verify_final_catches;
                    verify_final_columns += added2;
                    clog << "\tVERIFY_FINAL: Exact pass missed " << added2 << " column(s)" << endl;
                    for (size_t k = 0; k < negative2; ++k) clog << "\tVERIFY_FINAL route " << STR(R2[k].path) << " duration " << R2[k].duration << endl;
                    added = added2;
                    break;
                }
            }
            if (at_exact) {
                // The final labeling level added no column. Record whether search limits allowed the pass to finish. Numerical pricing discrepancies are retained in pricing_integrity.
                ladder_certified = covered && !bcp.deadline.Reached() && !MemoryMonitor::Exceeded();
                heuristic_level = 0;
                lbl.closing_state = false;
                lbl.merge_start = 0;
                break;
            }
            ++heuristic_level;
        }
        return added;
    };

    bcp.pricing_solver = [&](const PricingProblem& pricing_problem, int /*node_number*/,
                             Duration /*tlimit*/, CGExecutionLog* cg_execution_log) {
        run_ladder_and_add(pricing_problem, cg_execution_log);
        // A deadline hit must never masquerade as "no new columns =
        // node optimal".
        if (bcp.deadline.Reached())
            cg_execution_log->status = CGStatus::TimeLimitReached;
        // A
        // memory-truncated pricing pass must never read as node-optimal.
        // Set last so memory takes precedence; colgen exits with this status
        // when no new columns landed, and ProcessNode maps it to
        // BCStatus::MemoryLimitReached (the existing CG->BC branch).
        if (MemoryMonitor::Exceeded())
            cg_execution_log->status = CGStatus::MemoryLimitReached;
        if (cg_execution_log->iterations.IsSet() && !cg_execution_log->iterations->empty())
            cg_execution_log->iterations->back()["ablation_final_pricing_certified"] =
                ladder_certified && !bcp.deadline.Reached() && !MemoryMonitor::Exceeded();
    };

    VRPSolution solution(INFTY, {});
    auto log = bcp.Run(&solution);

    nlohmann::json result;
    result["effective_backend_settings"] = goc::ablation_backend_settings();
    result["effective_algorithm_settings"] = {{"cut_limit", params.cut_limit}, {"cut_violation_threshold", 0.1}, {"algorithm_epsilon", 1e-6}, {"solution_limit", params.solution_limit}, {"node_limit", params.node_limit}, {"ng_routes", false}};
    result["effective_algorithm_settings"]["label_correction"] = lbl.correcting;
    result["effective_algorithm_settings"]["lazy_extension"] = lbl.lazy_extension;
    result["effective_algorithm_settings"]["pricing_options"] = {
        {"near_rel", near_rel}, {"near_limit", near_limit}, {"heuristic_cut_blind", true},
        {"last_arc_merge_iterate", lbl.last_arc_merge_iterate}, {"merge_prefilter", lbl.merge_prefilter},
        {"packed_filter", lbl.packed_filter != 0}, {"verify_final_mask", verify_mask}};
    result["pricing_counters"] = {
        {"near_columns_added", near_columns_added}, {"near_columns_discarded", near_columns_discarded},
        {"merge_prefilter_hits", lbl.merge_prefilter_hits}, {"verify_final_runs", verify_final_runs},
        {"verify_final_catches", verify_final_catches}, {"verify_final_columns", verify_final_columns}};
    result["exact_log"] = log;
    result["bound_events"] = bound_events;
    result["closure_census"] = closure_census();
    result["exact_pricing"] = {{"calls", exact_calls}, {"finished", exact_finished}};
    result["checker_infeasible_columns"] = checker_infeasible_count;
    // The exact master: cost M of the artificial columns, singletons that the exact evaluation rejects, and the incumbent candidates of the BCP (see BCP::AcceptIncumbent).
    result["exact_master"] = {
        {"artificial_cost", nearest_binary64(artificial)}, {"artificial_columns", vrp.D.NbVertices() - 2},
        {"infeasible_singletons", infeasible_singletons},
        {"artificial_candidates", bcp.artificial_candidate_count},
        {"rejected_candidates", bcp.rejected_candidate_count},
        {"repriced_candidates", bcp.repriced_candidate_count}};
    // See the comment at the declaration of the counters.
    result["pricing_integrity"] = {
        {"epsilon", integrity_eps},
        {"empty_negative_priced_routes", empty_negative_routes}, {"empty_near_priced_routes", empty_near_routes},
        {"phantom_columns", phantom_columns}, {"phantom_duplicates", phantom_duplicates},
        {"max_phantom_reduced_cost", max_phantom_reduced_cost},
        {"labeling_below_checker_columns", labeling_below_checker}, {"max_labeling_below_checker", max_labeling_gap},
        {"route_duration_mismatches", route_duration_mismatches}, {"route_duration_below_checker", route_duration_below},
        {"max_route_duration_gap", max_route_duration_gap},
        {"verify_final_negative_columns", verify_negative_columns},
        {"negative_duplicates", negative_duplicates}};
    // The conditions under which every node closure of this solve was
    // backed by a finished Exact pricing pass (and, with a VERIFY_FINAL mask,
    // by a finished verification pass) that found no new column. They restate
    // closure_census as booleans with stable names:
    //  - status_optimum: the search ended with status Optimum.
    //  - all_closures_certified: cg_unresolved_final_pricing == 0.
    //  - no_unpriced_closure: no node was closed without pricing, i.e.
    //    cg_infeasible_unverified, selected_rmp_infeasible_unverified and
    //    unbranched_fractional_unverified are all 0.
    //  - no_checker_infeasible_column: checker_infeasible_columns == 0.
    //  - no_empty_negative_priced_route: all negative-priced paths were reconstructed.
    //  - no_rejected_candidate: the exact evaluation rejected no integer solution of the master (fleet limit or coverage), so no node closed on a solution that was not one.
    // all_hold records coverage of the native closure checks. A certified lower bound under the exact evaluation profile also needs a verified pricing argument.
    {
        const bool status_optimum = log.status == BCStatus::Optimum;
        const bool all_certified = bcp.cg_unresolved_count == 0;
        const bool no_unpriced = bcp.cg_infeasible_count == 0 && bcp.selected_rmp_infeasible_count == 0
            && bcp.unbranched_fractional_count == 0;
        const bool no_infeasible = checker_infeasible_count == 0;
        const bool no_rejected = bcp.rejected_candidate_count == 0;
        result["closure_conditions"] = {
            {"status_optimum", status_optimum}, {"all_closures_certified", all_certified},
            {"no_unpriced_closure", no_unpriced}, {"no_checker_infeasible_column", no_infeasible},
            {"no_empty_negative_priced_route", empty_negative_routes == 0},
            {"no_rejected_candidate", no_rejected},
            {"all_hold", status_optimum && all_certified && no_unpriced && no_infeasible && empty_negative_routes == 0 && no_rejected},
            {"verify_final_mask", verify_mask}};
    }
    // Record the memory guard diagnostics.
    result["memory"] = {
        {"limit_mb", params.memory_limit_mb},
        {"peak_rss_mb", (double)MemoryMonitor::PeakRSSBytes() / 1048576.0},
        {"limit_reached", MemoryMonitor::Exceeded()},
    };
    result["incumbents"] = incumbents;
    if (!warm_log.is_null()) result["warm_start"] = warm_log;
    if (solution.value != INFTY) {
        // The value is the nearest binary64 value of the exact cost of the incumbent, which exact_value gives as a fraction. The public API independently checks the routes with the library checker.
        result["value"] = solution.value;
        result["exact_value"] = bcp.ExactIncumbentValue()->get_str();
        result["routes"] = solution.routes;
    }
    return result.dump();
}

}  // namespace kayros::lera
