// kayros-owned bridge to the vendored Lera-Romero BPC (NOT vendored code).
#include "lera_bridge.h"

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <goc/goc.h>
#include <nyr/nyr.h>

#include "bcp/bcp.h"
#include "bcp/pricing_problem.h"
#include "bcp/spf.h"
#include "core/instance.h"
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

// Checker-exact certification instance (M5.6 stage A) built from the payload's
// mamut_raw section: untouched MAMUT ATF breakpoints, TWs and service times
// (Lera's preprocessing folds service/waiting into travel times and shifts
// TWs, and the tau pieces store y-x whose re-addition is not bit-exact — the
// certification arithmetic must run on the verbatim data).
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

// Lera path (o=0, customers..., d=n+1) -> checker evaluation of the customers.
kayros::RouteEval checker_eval(const kayros::Instance& ci, const goc::GraphPath& path) {
    vector<int32_t> customers;
    for (goc::Vertex v : path)
        if (v != 0 && v != ci.num_customers + 1) customers.push_back(v);
    return kayros::evaluate_route(ci, customers.data(), (int64_t)customers.size());
}

}  // namespace

std::string solve_duration_json(const std::string& payload, const SolveParams& params) {
    nlohmann::json inst = nlohmann::json::parse(payload);

    // Checker-exact certification instance (M5.6 stage A), grabbed before
    // Lera's preprocessing mutates the payload.
    const kayros::Instance checker_inst = build_checker_instance(inst.at("mamut_raw"));
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

    // Create SPF and add initial routes (o, i, d) with checker-exact costs
    // (M5.6 stage A: every master objective coefficient is the checker value
    // of its route, so LP bounds and incumbents live in checker arithmetic).
    // A checker-infeasible single-customer route keeps Lera's INFTY cost: it
    // stays as an artificial column guaranteeing RMP feasibility and can
    // never enter a finite-value solution.
    // Column fingerprints (per path): duplicates must never re-enter the SPF —
    // under smoothed duals (stabilization below) existing master columns can
    // price negative again, and re-adding them makes no LP progress.
    unordered_set<string> column_keys;
    auto path_key = [](const GraphPath& p) {
        string k;
        for (goc::Vertex v : p) { k += to_string(v); k += ','; }
        return k;
    };

    SPF spf(vrp.D.NbVertices());
    for (Vertex i : exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) {
        const GraphPath seed_path{vrp.o, i, vrp.d};
        auto r = vrp.BestDurationRoute(seed_path);
        auto ev = checker_eval(checker_inst, seed_path);
        column_keys.insert(path_key(seed_path));
        spf.AddRoute(ev.feasible ? Route(seed_path, ev.departure, ev.duration)
                                 : Route(r.path, r.t0, r.value));
    }

    BCP bcp(vrp.D, &spf);
    bcp.time_limit = Duration(params.time_limit_s, DurationUnit::Seconds);
    bcp.cut_limit = params.cut_limit;
    bcp.node_limit = params.node_limit;

    // Warm start (M5.3): reprice each heuristic route with the checker-exact
    // fold (M5.6 stage A: the master's arithmetic) and add it as an initial
    // column. Checker-infeasible routes are skipped; the UB is only set when
    // the added routes still partition the customers exactly.
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
            auto ev = checker_eval(checker_inst, path);
            if (!ev.feasible) { valid_ub = false; continue; }
            column_keys.insert(path_key(path));
            spf.AddRoute(Route(path, ev.departure, ev.duration));
            ++added;
            ub_value += ev.duration;
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
    auto run_ladder = [&](const PricingProblem& pp, CGExecutionLog* cg_execution_log) {
        vector<Route> R;
        while (heuristic_level < levels.size()) {
            // (M5.2) Residual budget from the BCP's single absolute deadline,
            // not from a per-call stopwatch chain.
            lbl.time_limit = bcp.deadline.Remaining();
            auto lbl_log = lbl.Run(pp, &R, levels[heuristic_level]);

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
        if (heuristic_level >= levels.size()) {
            heuristic_level = 0;
            lbl.closing_state = false;
            lbl.merge_start = 0;
        }
        return R;
    };

    // Column adds, shared by both pricing passes below. Checker-exact costs
    // (M5.6 stage A): a route the labeling prices but the checker rejects
    // would break certification integrity — fail loudly, never drop silently
    // (dropping could let CG claim optimality with improving columns
    // outstanding). Pricing emits only reduced costs < -1e-6 while the
    // Lera-vs-checker cost dust is ~1e-11, so every priced column is still
    // strictly improving under checker costs. Duplicate columns are skipped
    // by path fingerprint: under SMOOTHED duals existing master columns can
    // price negative again, and re-adding them makes no LP progress (the
    // no-new-columns outcome is the misprice signal instead). (M5.2) The
    // adds are deadline-checked per route: a full pool is thousands of ~1 ms
    // formulation inserts, the dominant non-interruptible tail otherwise.
    auto add_columns = [&](const vector<Route>& R) {
        int added = 0;
        for (const auto& r : R) {
            if (bcp.deadline.Reached()) break;
            // Lera's solution pool holds an empty-path INFTY placeholder for
            // merge candidates whose duration never resolved (Route({}, 0,
            // INFTY) in AddSolution); pool repricing can emit it. It is not a
            // column (no customers, INFTY value, never improving), so it is
            // skipped rather than certified — the loud-fail below is for real
            // routes the checker rejects.
            if (r.path.size() < 3) continue;
            if (!column_keys.insert(path_key(r.path)).second) continue;
            auto ev = checker_eval(checker_inst, r.path);
            if (!ev.feasible)
                fail("M5.6: labeling priced a checker-infeasible route (path " + STR(r.path) + ")");
            spf.AddRoute(Route(r.path, ev.departure, ev.duration));
            ++added;
        }
        return added;
    };

    // Dual stabilization (M5.1b residual), Neame's rule: pricing runs on the
    // exponential moving average pi_s = alpha*pi_s_prev + (1-alpha)*pi, which
    // tracks the dual trajectory while damping the oscillations HiGHS's
    // alternative optima feed to the labeling (the diagnosed dual-trajectory
    // sensitivity). A Wentges center-on-misprice-only variant was tried first
    // and froze the center at the first iteration's duals (C103 12 s -> TL).
    // Correctness is untouched by any center/alpha policy: CG termination is
    // only ever decided on TRUE duals — a smoothed pass that adds no NEW
    // column triggers an immediate true-duals pass (misprice), which resets
    // the center to pi. New cuts enter the center with dual 0.
    vector<double> center_P, center_sigma;
    bool center_set = false;
    int misprice_count = 0;
    const double alpha = params.stab_alpha;
    bcp.pricing_solver = [&](const PricingProblem& pricing_problem, int /*node_number*/,
                             Duration /*tlimit*/, CGExecutionLog* cg_execution_log) {
        if (alpha > 0.0 && center_set) {
            PricingProblem pps = pricing_problem;  // A and S are structural.
            for (size_t i = 0; i < pps.P.size(); ++i)
                pps.P[i] = alpha * center_P[i] + (1.0 - alpha) * pricing_problem.P[i];
            center_sigma.resize(pricing_problem.sigma.size(), 0.0);
            for (size_t i = 0; i < pps.sigma.size(); ++i)
                pps.sigma[i] = alpha * center_sigma[i] + (1.0 - alpha) * pricing_problem.sigma[i];
            int added = add_columns(run_ladder(pps, cg_execution_log));
            if (added == 0 && !bcp.deadline.Reached()) {
                ++misprice_count;
                add_columns(run_ladder(pricing_problem, cg_execution_log));
                center_P = pricing_problem.P;
                center_sigma = pricing_problem.sigma;
            } else {
                // Neame EMA: the center is the point we just priced at.
                center_P = std::move(pps.P);
                center_sigma = std::move(pps.sigma);
            }
        } else {
            add_columns(run_ladder(pricing_problem, cg_execution_log));
            if (!center_set) {
                center_P = pricing_problem.P;
                center_sigma = pricing_problem.sigma;
                center_set = true;
            }
        }
        // (M5.2) A deadline hit must never masquerade as "no new columns =
        // node optimal".
        if (bcp.deadline.Reached())
            cg_execution_log->status = CGStatus::TimeLimitReached;
    };

    VRPSolution solution(INFTY, {});
    auto log = bcp.Run(&solution);

    nlohmann::json result;
    result["exact_log"] = log;
    result["incumbents"] = incumbents;
    if (!warm_log.is_null()) result["warm_start"] = warm_log;
    if (alpha > 0.0) result["stabilization"] = {{"alpha", alpha}, {"misprices", misprice_count}};
    if (solution.value != INFTY) {
        // (M5.6 stage A) Column costs are checker-exact, but the LP solver's
        // reported objective sums them in its own order: re-sum the route
        // costs in the checker's canonical route order (sorted by first
        // customer, compute_solution_cost convention, left fold) so the
        // reported value is bit-identical to the checker sum.
        if (!solution.routes.empty()) {
            vector<const Route*> ordered;
            for (const auto& r : solution.routes) ordered.push_back(&r);
            sort(ordered.begin(), ordered.end(),
                 [](const Route* a, const Route* b) { return a->path[1] < b->path[1]; });
            double v = 0.0;
            for (const Route* r : ordered) v += r->duration;
            result["value"] = v;
        } else {
            result["value"] = solution.value;
        }
        result["routes"] = solution.routes;
    }
    return result.dump();
}

}  // namespace kayros::lera
