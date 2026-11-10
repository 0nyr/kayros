// kayros-owned bridge to the vendored Lera-Romero BPC (NOT vendored code).
#include "goc/linear_programming/solver/ablation_settings.h"
#include "lera_bridge.h"

#include <algorithm>
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
    goc::ablation_backend_settings() = nlohmann::json::object();
    // M13.3: pin the PWL vertical arithmetic for the whole solve, from the
    // same KAYROS_STEP_EXACT flag the Python bridge sets per instance and the
    // labeling already reads. Step-carrying instances get the M13.0 exact
    // tagged-vertical rules; jump-free instances get the audited kayros 1.0.0
    // rules bit-identically (the M13.0 rules over-certify on them; see
    // goc/math/pwl_function.h). Scoped so the process-global default (M13.0,
    // the goc unit-test semantics) is restored on return.
    const goc::StepExactArithmeticScope step_mode(std::getenv("KAYROS_STEP_EXACT") != nullptr);

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

    // kayros (M13.2): arm the process-global RSS watermark for this solve
    // (resets any sticky tripped state from a previous solve in-process).
    MemoryMonitor::SetLimitBytes((int64_t)(params.memory_limit_mb * 1048576.0));

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

    // (ms61) Column-generation control switches, all inert by default (private
    // development candidates, read from the environment so that one binary
    // serves every arm of a comparison). None of them touches the label
    // arithmetic or the termination rule: a node is only ever declared
    // optimal after an Exact pricing pass on true duals that finishes without
    // a new negative column, followed by a cut round that finds no violated
    // cut.
    //  - DROPBACK_LEVEL (-1 off, 0 or 1): after an Exact pass that contributed
    //    at least DROPBACK_MIN_COLS new columns, the next pass restarts at
    //    that heuristic level instead of staying at Exact.
    //  - EARLY_CUTS (0 off, 1, 2): at the root, separate subset-row cuts on
    //    the current restricted-master solution before paying for an Exact
    //    pass. Mode 1 does so before every Exact pass. Mode 2 does so only
    //    when the previous Exact pass of the current cycle was productive and
    //    Finished (pool not saturated), i.e. when the master is close to
    //    converged. Mode 1 can add cuts that the converged master would never
    //    violate (seen on Dabia2013 n=25 C207: 25 cuts against 0).
    //  - NEAR_REL (>= 0) and NEAR_LIMIT: an Exact pass that found at least one
    //    new negative column also contributes up to NEAR_LIMIT routes whose
    //    labeling reduced cost lies in [0, NEAR_REL * mean customer dual).
    //  - CORRECTION (-1 keep default, 0 off, 1 on): label correction step.
    //  - DSSR (0 off, 1, 2): the Exact level runs as a decremental
    //    state-space relaxation. Elementarity is enforced on a critical
    //    vertex set theta only, which starts empty, persists for the whole
    //    solve and grows with the vertices repeated by the most negative
    //    non-elementary routes. Only elementary routes become columns. A pass
    //    proves the absence of negative elementary routes only when it
    //    Finished without any negative route of the relaxed space, repeated
    //    vertices or not. Mode 1 grows theta when a pass yields no new
    //    column, mode 2 after every pass that met a non-elementary route.
    auto env_int = [](const char* name, int fallback) {
        const char* v = std::getenv(name);
        return v && *v ? std::atoi(v) : fallback;
    };
    auto env_double = [](const char* name, double fallback) {
        const char* v = std::getenv(name);
        return v && *v ? std::atof(v) : fallback;
    };
    const int dropback_level = env_int("KAYROS_BPC_DROPBACK_LEVEL", -1);
    const int dropback_min_cols = env_int("KAYROS_BPC_DROPBACK_MIN_COLS", 1);
    const int early_cuts = env_int("KAYROS_BPC_EARLY_CUTS", 0);
    if (early_cuts < 0 || early_cuts > 2)
        throw std::invalid_argument("KAYROS_BPC_EARLY_CUTS must be 0, 1 or 2");
    bool exact_productive_since_cuts = false;
    const double near_rel = env_double("KAYROS_BPC_NEAR_REL", 0.0);
    const int near_limit = env_int("KAYROS_BPC_NEAR_LIMIT", 3000);
    const int correction_switch = env_int("KAYROS_BPC_CORRECTION", -1);
    if (dropback_level < -1 || dropback_level > 1)
        throw std::invalid_argument("KAYROS_BPC_DROPBACK_LEVEL must be -1, 0 or 1");
    if (near_rel < 0.0 || near_limit < 0)
        throw std::invalid_argument("KAYROS_BPC_NEAR_REL and KAYROS_BPC_NEAR_LIMIT must be non-negative");
    if (correction_switch >= 0) lbl.correcting = correction_switch != 0;
    //  - PACKED (0 off, 1 on, 2 on with self-check): packed dominance filter,
    //    a pure execution change (same labels, same order, same verdicts).
    const int packed_filter = env_int("KAYROS_BPC_PACKED", 0);
    if (packed_filter < 0 || packed_filter > 2) throw std::invalid_argument("KAYROS_BPC_PACKED must be 0, 1 or 2");
    lbl.packed_filter = packed_filter;
    //  - LAM_ITERATE (0/1): last-arc merge walks the leftover queue instead
    //    of popping it (same merges, different order).
    const bool lam_iterate = env_int("KAYROS_BPC_LAM_ITERATE", 0) != 0;
    lbl.last_arc_merge_iterate = lam_iterate;
    //  - ADAPT_S (>= 0): when positive, drop-back, early cuts (mode 2) and
    //    near-zero columns only act after an Exact pass that lasted at least
    //    this many seconds, judged on the most recent Exact pass. Instances
    //    whose Exact passes are cheap keep the legacy ladder, where these
    //    levers only add iterations and cuts.
    //  - MERGE_PREFILTER (0/1): exact pre-merge bound with shared cut
    //    penalties.
    const double adapt_s = env_double("KAYROS_BPC_ADAPT_S", 0.0);
    if (adapt_s < 0.0) throw std::invalid_argument("KAYROS_BPC_ADAPT_S must be non-negative");
    double last_exact_pass_s = 0.0;
    auto levers_active = [&]() { return adapt_s <= 0.0 || last_exact_pass_s >= adapt_s; };
    const bool merge_prefilter = env_int("KAYROS_BPC_MERGE_PREFILTER", 0) != 0;
    lbl.merge_prefilter = merge_prefilter;
    //  - VERIFY_FINAL (0/1): diagnostic and safety net. When an Exact pass
    //    with label correction finishes without a new column, the pass is run
    //    again with correction off before the node is declared optimal. Any
    //    column it finds is a column the corrected pass missed.
    // The value is a bit mask of what the verification pass changes:
    // 1 correction off, 2 opening state (iterative merge from the start),
    // 4 no cost sorting, 8 weak unreachable sets, 16 no partial dominance,
    // 32 no limited extension, 64 no packed filter and no merge prefilter.
    const int verify_mask = env_int("KAYROS_BPC_VERIFY_FINAL", 0);
    const bool verify_final = verify_mask != 0;
    int verify_final_runs = 0, verify_final_catches = 0;
    long long verify_final_columns = 0;
    const int dssr = env_int("KAYROS_BPC_DSSR", 0);
    if (dssr < 0 || dssr > 2) throw std::invalid_argument("KAYROS_BPC_DSSR must be 0, 1 or 2");
    //  - DSSR_LOCAL (0/1): per-vertex memories instead of one global critical
    //    set (a dynamic ng-route relaxation). A repeated vertex i is added to
    //    the memory of every vertex lying between its two visits.
    //  - DSSR_GROWTH (>= 1): number of most negative non-elementary routes
    //    whose repeats feed each growth step.
    const bool dssr_local = env_int("KAYROS_BPC_DSSR_LOCAL", 0) != 0;
    const int dssr_growth = env_int("KAYROS_BPC_DSSR_GROWTH", 10);
    if (dssr_growth < 1) throw std::invalid_argument("KAYROS_BPC_DSSR_GROWTH must be positive");
    lbl.theta_growth_routes = dssr_growth;
    nyr::VertexSet theta;
    std::vector<nyr::VertexSet> theta_local;
    if (dssr > 0 && dssr_local) theta_local.assign(vrp.D.NbVertices(), nyr::VertexSet());
    // Tightens the relaxation with the non-elementary routes of the last
    // pass. Returns whether anything changed (it always does when a route of
    // the relaxed space repeats a vertex).
    auto grow_relaxation = [&]() {
        bool changed = false;
        if (!dssr_local) {
            const nyr::VertexSet before = theta;
            theta |= lbl.last_repeated;
            return theta != before;
        }
        for (const auto& path : lbl.last_nonelementary_paths) {
            std::vector<int> last_position(vrp.D.NbVertices(), -1);
            for (int b = 0; b < (int) path.size(); ++b) {
                const int a = last_position[path[b]];
                if (a >= 0)
                    for (int k = a + 1; k < b; ++k)
                        if (!theta_local[path[k]].test(path[b])) { theta_local[path[k]].set(path[b]); changed = true; }
                last_position[path[b]] = b;
            }
        }
        return changed;
    };
    auto relaxation_size = [&]() {
        if (!dssr_local) return (int) theta.count();
        size_t total = 0;
        for (const auto& memory : theta_local) total += memory.count();
        return (int) total;
    };
    //  - DSSR_AUTO_S (>= 0, with DSSR > 0): start with the plain Exact level
    //    and switch to the relaxation for the rest of the solve the first
    //    time a plain Exact pass runs longer than this many seconds. The
    //    interrupted pass is not a proof of anything: its columns are kept
    //    and the level is run again under the relaxation.
    const double dssr_auto_s = env_double("KAYROS_BPC_DSSR_AUTO_S", 0.0);
    if (dssr_auto_s < 0.0) throw std::invalid_argument("KAYROS_BPC_DSSR_AUTO_S must be non-negative");
    bool dssr_active = dssr > 0 && dssr_auto_s <= 0.0;
    bool exact_pass_interrupted = false;
    int dssr_auto_switch_exact_call = -1;
    int theta_growths = 0, dssr_reruns = 0;
    int dropback_count = 0, early_cut_rounds = 0, early_cut_count = 0;
    long long near_columns_added = 0, near_columns_discarded = 0;
    bool exact_state_saved = false, saved_closing_state = false;
    int saved_merge_start = 0;

    //  - HE_CUTBLIND (0 off, 1, 2): the heuristic elementarity level ignores
    //    cut memory in dominance. Mode 1 replaces the level. Mode 2 inserts
    //    the cut-blind pass before the legacy one (4-level ladder).
    const int he_cutblind = env_int("KAYROS_BPC_HE_CUTBLIND", 0);
    if (he_cutblind < 0 || he_cutblind > 2) throw std::invalid_argument("KAYROS_BPC_HE_CUTBLIND must be 0, 1 or 2");
    vector<LabelingLevel> levels = {
        LabelingLevel::HeuristicCost, LabelingLevel::HeuristicElementarity, LabelingLevel::Exact};
    vector<string> level_names = {"Heuristic Cost", "Heuristic Elementarity", "Exact"};
    vector<bool> level_cut_blind = {false, he_cutblind == 1, false};
    if (he_cutblind == 2) {
        levels = {LabelingLevel::HeuristicCost, LabelingLevel::HeuristicElementarity,
                  LabelingLevel::HeuristicElementarity, LabelingLevel::Exact};
        level_names = {"Heuristic Cost", "Heuristic Elementarity Blind", "Heuristic Elementarity", "Exact"};
        level_cut_blind = {false, true, false, false};
    }
    size_t heuristic_level = 0;
    int last_negative = 0; // (ms61) leading negative routes in the last R.
    auto run_one_level = [&](const PricingProblem& pp, CGExecutionLog* cg_execution_log) {
        vector<Route> R;
        // (M5.2) Residual budget from the BCP's single absolute deadline,
        // not from a per-call stopwatch chain.
        lbl.time_limit = bcp.deadline.Remaining();
        if (heuristic_level == levels.size() - 1) ++exact_calls;
        // (ms61) Near-zero collection only at the Exact level, scaled by the
        // mean positive customer dual so one setting serves every instance.
        lbl.near_threshold = 0.0;
        lbl.near_limit = near_limit;
        if (near_rel > 0.0 && near_limit > 0 && heuristic_level == levels.size() - 1 && levers_active()) {
            double dual_sum = 0.0;
            int dual_count = 0;
            for (Vertex v : exclude(vrp.D.Vertices(), {vrp.o, vrp.d})) {
                if (pp.P[v] > 0.0) dual_sum += pp.P[v];
                ++dual_count;
            }
            if (dual_count > 0) lbl.near_threshold = near_rel * dual_sum / dual_count;
        }
        lbl.heuristic_cut_blind = level_cut_blind[heuristic_level];
        lbl.theta_relaxation = dssr_active;
        lbl.theta = theta;
        const bool probing_exact_cost = dssr > 0 && !dssr_active && heuristic_level == levels.size() - 1;
        if (probing_exact_cost)
            lbl.time_limit = std::min(lbl.time_limit, Duration(dssr_auto_s, DurationUnit::Seconds));
        lbl.theta_local = theta_local;
        Stopwatch pass_rolex(true);
        auto lbl_log = lbl.Run(pp, &R, levels[heuristic_level]);
        if (heuristic_level == levels.size() - 1) last_exact_pass_s = pass_rolex.Peek().Amount(DurationUnit::Seconds);
        last_negative = lbl.last_negative_count;
        // (ms61) A plain Exact pass cut short by the cost probe, not by the
        // solve deadline: switch to the relaxation for good.
        exact_pass_interrupted = probing_exact_cost && lbl_log.status == BLBStatus::TimeLimitReached
            && !bcp.deadline.Reached();
        if (exact_pass_interrupted) {
            dssr_active = true;
            dssr_auto_switch_exact_call = exact_calls;
        }
        last_exact_finished = heuristic_level == levels.size() - 1 && lbl_log.status == BLBStatus::Finished;
        if (last_exact_finished) ++exact_finished;

        // Add iteration log.
        cg_execution_log->iterations->push_back(lbl_log);
        cg_execution_log->iterations->back()["iteration_name"] = level_names[heuristic_level];
        cg_execution_log->iterations->back()["ms61_negative_routes"] = lbl.last_negative_count;
        cg_execution_log->iterations->back()["ms61_near_routes"] = lbl.last_near_count;
        cg_execution_log->iterations->back()["ms61_near_threshold"] = lbl.near_threshold;
        cg_execution_log->iterations->back()["ms61_theta_size"] = relaxation_size();
        cg_execution_log->iterations->back()["ms61_nonelementary_routes"] = lbl.last_nonelementary_count;
        cg_execution_log->iterations->back()["ms61_bcp_elapsed_s"] =
            (bcp.time_limit - bcp.deadline.Remaining()).Amount(DurationUnit::Seconds);

        // Update merge_start and closing_state.
        lbl.closing_state |=
            heuristic_level == levels.size() - 1 && lbl_log.status == BLBStatus::Finished;
        lbl.merge_start = (lbl.merge_start + lbl_log.forward_log->processed_count) / 2;
        return R;
    };

    int checker_infeasible_count = 0; // M5.9 (19/n): poisons the certificate.

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
    auto add_columns = [&](const vector<Route>& R, size_t begin, size_t end) {
        int added = 0;
        for (size_t route_index = begin; route_index < end && route_index < R.size(); ++route_index) {
            const auto& r = R[route_index];
            if (bcp.deadline.Reached()) break;
            if (MemoryMonitor::Exceeded()) break; // kayros (M13.2)
            // Lera's solution pool holds an empty-path INFTY placeholder for
            // merge candidates whose duration never resolved (Route({}, 0,
            // INFTY) in AddSolution); pool repricing can emit it. It is not a
            // column (no customers, INFTY value, never improving), so it is
            // skipped rather than certified — the loud-fail below is for real
            // routes the checker rejects.
            if (r.path.size() < 3) continue;
            if (!column_keys.insert(path_key(r.path)).second) { ++duplicate_columns; continue; }
            auto ev = checker_eval(checker_inst, r.path);
            if (!ev.feasible)
            {
                // M5.9 (19/n): the labeling CAN price a checker-infeasible
                // route (first materialized on Rifki-10 n=60 once the Exact
                // level actually ran, on the retired mollified stepwise path,
                // where dep on bridged arrivals believed slightly-later
                // departures feasible; retained as a general integrity net,
                // since the reverse-side normalizer still bridges). Crashing
                // loses the whole
                // solve and silently dropping could let CG close with
                // improving columns outstanding — so the column is skipped,
                // COUNTED, and the count poisons the certificate: the caller
                // exposes it and the analyzer refuses instances with a
                // nonzero count. Sound outcome: no certificate, not a wrong
                // one, and the campaign run survives.
                ++checker_infeasible_count;
                clog << "M5.9: skipped checker-infeasible priced route (path "
                     << STR(r.path) << "), certificate poisoned" << endl;
                continue;
            }
            spf.AddRoute(Route(r.path, ev.departure, ev.duration));
            ++added;
        }
        return added;
    };

    // M5.9 (17/n): ladder escalation is driven by NEW columns, not by a
    // non-empty pricing pool. A heuristic level can keep re-pricing an
    // EXISTING column (its labeling-arithmetic reduced cost slightly negative
    // while the checker-exact master cost is not), the fingerprint dedup then
    // adds 0, and the old ladder broke without escalating — so the BCP read
    // "no new columns" as node-optimal WITHOUT the Exact level ever running:
    // unsound termination (the Rifki-14 k=15 reproducer certified 5606 with
    // pricing logs showing zero Exact iterations). Now the ladder climbs until
    // some level contributes new columns or the Exact level finishes empty.
    auto run_ladder_and_add = [&](const PricingProblem& pp, CGExecutionLog* lg, bool true_duals) {
        int added = 0;
        ladder_certified = false;
        while (!bcp.deadline.Reached()) {
            if (MemoryMonitor::Exceeded()) break; // kayros (M13.2): don't climb levels on a doomed run
            const bool at_exact = heuristic_level + 1 >= levels.size();
            if (at_exact && exact_state_saved) {
                // (ms61) Re-entering Exact after a drop-back: resume the
                // merge policy the Exact level had when it was left.
                lbl.closing_state = saved_closing_state;
                lbl.merge_start = saved_merge_start;
                exact_state_saved = false;
            }
            if (at_exact && true_duals && early_cuts > 0 && bcp.CanSeparateCuts()
                && (early_cuts == 1 || (exact_productive_since_cuts && levers_active()))) {
                // (ms61) Try cuts on the current master solution before an
                // Exact pass. Cuts change the master, so the CG loop
                // re-solves and pricing restarts at the first level. An Exact
                // pass is still owed before the node can be declared optimal.
                exact_productive_since_cuts = false;
                const int cuts = bcp.SeparateCutRounds();
                if (cuts > 0) {
                    ++early_cut_rounds;
                    early_cut_count += cuts;
                    clog << "\tEarly cuts: " << cuts << endl;
                    heuristic_level = 0;
                    lbl.closing_state = false;
                    lbl.merge_start = 0;
                    exact_state_saved = false;
                    break;
                }
            }
            const vector<Route> R = run_one_level(pp, lg);
            const size_t negative = std::min<size_t>(R.size(), (size_t) std::max(last_negative, 0));
            added = add_columns(R, 0, negative);
            // (ms61) "Near converged" for early cuts mode 2: the last Exact
            // pass explored its whole space and found something, new columns
            // or, under the relaxation, routes that repeat a vertex.
            if (at_exact) exact_productive_since_cuts = lbl.last_complete
                && (added > 0 || lbl.last_nonelementary_count > 0);
            if (added > 0 && at_exact && dssr_active && dssr == 2 && lbl.last_nonelementary_count > 0) {
                if (grow_relaxation()) ++theta_growths;
            }
            if (added > 0) {
                // (ms61) Near-zero routes ride along with a productive pass
                // only. A pass without a new negative column discards them,
                // so "no new column" keeps its meaning for termination.
                if (negative < R.size()) near_columns_added += add_columns(R, negative, R.size());
                if (at_exact && dropback_level >= 0 && added >= dropback_min_cols && levers_active()) {
                    saved_closing_state = lbl.closing_state;
                    saved_merge_start = lbl.merge_start;
                    exact_state_saved = true;
                    heuristic_level = (size_t) dropback_level;
                    lbl.closing_state = false;
                    lbl.merge_start = 0;
                    ++dropback_count;
                }
                break;
            }
            near_columns_discarded += (long long) (R.size() - negative);
            if (at_exact && exact_pass_interrupted) {
                // (ms61) The cost probe interrupted a plain Exact pass that
                // produced no new column. Nothing was proven: run the level
                // again, now under the relaxation.
                ++dssr_reruns;
                continue;
            }
            if (at_exact && dssr_active && lbl.last_nonelementary_count > 0) {
                // (ms61) The relaxed pass met negative routes that repeat a
                // vertex and produced no new column: tighten the relaxation
                // and run the Exact level again. A route of the relaxed space
                // never repeats a vertex of theta, so theta strictly grows.
                if (!grow_relaxation())
                    throw std::logic_error("DSSR: non-elementary route that does not tighten the relaxation");
                ++theta_growths;
                ++dssr_reruns;
                continue;
            }
            if (at_exact && verify_final && last_exact_finished && !bcp.deadline.Reached()) {
                // (ms61) Re-run the empty Exact pass with the masked features changed.
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
                lbl.correcting = s_corr; lbl.closing_state = s_closing; lbl.sort_by_cost = s_sort;
                lbl.unreachable_strengthened = s_unreach; lbl.partial = s_partial; lbl.limited_extension = s_limited;
                lbl.merge_prefilter = s_prefilter; lbl.merge_start = s_merge_start; lbl.packed_filter = s_packed;
                const size_t negative2 = std::min<size_t>(R2.size(), (size_t) std::max(last_negative, 0));
                const int added2 = add_columns(R2, 0, negative2);
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
                // The Exact level contributed nothing new: pricing exhausted.
                // (If it still REPORTED a negative-reduced-cost duplicate, the
                // discrepancy is labeling-vs-checker dust within the documented
                // certificate tolerance; the Exact level did run.)
                ladder_certified = last_exact_finished && !bcp.deadline.Reached() && !MemoryMonitor::Exceeded();
                heuristic_level = 0;
                lbl.closing_state = false;
                lbl.merge_start = 0;
                exact_state_saved = false;
                break;
            }
            ++heuristic_level;
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
        bool certified_true_duals = false;
        if (alpha > 0.0 && center_set) {
            PricingProblem pps = pricing_problem;  // A and S are structural.
            for (size_t i = 0; i < pps.P.size(); ++i)
                pps.P[i] = alpha * center_P[i] + (1.0 - alpha) * pricing_problem.P[i];
            center_sigma.resize(pricing_problem.sigma.size(), 0.0);
            for (size_t i = 0; i < pps.sigma.size(); ++i)
                pps.sigma[i] = alpha * center_sigma[i] + (1.0 - alpha) * pricing_problem.sigma[i];
            int added = run_ladder_and_add(pps, cg_execution_log, false);
            if (added == 0 && !bcp.deadline.Reached()) {
                ++misprice_count;
                run_ladder_and_add(pricing_problem, cg_execution_log, true);
                certified_true_duals = ladder_certified;
                center_P = pricing_problem.P;
                center_sigma = pricing_problem.sigma;
            } else {
                // Neame EMA: the center is the point we just priced at.
                center_P = std::move(pps.P);
                center_sigma = std::move(pps.sigma);
            }
        } else {
            run_ladder_and_add(pricing_problem, cg_execution_log, true);
            certified_true_duals = ladder_certified;
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
        // kayros (M13.2): same argument for the memory watermark: a
        // memory-truncated pricing pass must never read as node-optimal.
        // Set last so memory takes precedence; colgen exits with this status
        // when no new columns landed, and ProcessNode maps it to
        // BCStatus::MemoryLimitReached (the existing CG->BC branch).
        if (MemoryMonitor::Exceeded())
            cg_execution_log->status = CGStatus::MemoryLimitReached;
        if (cg_execution_log->iterations.IsSet() && !cg_execution_log->iterations->empty())
            cg_execution_log->iterations->back()["ablation_final_pricing_certified"] =
                certified_true_duals && !bcp.deadline.Reached() && !MemoryMonitor::Exceeded();
    };

    VRPSolution solution(INFTY, {});
    auto log = bcp.Run(&solution);

    nlohmann::json result;
    result["effective_backend_settings"] = goc::ablation_backend_settings();
    result["effective_algorithm_settings"] = {{"cut_limit", params.cut_limit}, {"cut_violation_threshold", 0.1}, {"algorithm_epsilon", 1e-6}, {"solution_limit", params.solution_limit}, {"node_limit", params.node_limit}, {"stab_alpha", params.stab_alpha}, {"ng_routes", false}};
    result["effective_algorithm_settings"]["label_correction"] = lbl.correcting;
    result["effective_algorithm_settings"]["lazy_extension"] = lbl.lazy_extension;
    result["effective_algorithm_settings"]["ms61"] = {
        {"dropback_level", dropback_level}, {"dropback_min_cols", dropback_min_cols},
        {"early_cuts", early_cuts}, {"near_rel", near_rel}, {"near_limit", near_limit},
        {"correction_switch", correction_switch}, {"dssr", dssr}, {"dssr_local", dssr_local}, {"dssr_auto_s", dssr_auto_s}, {"dssr_growth", dssr_growth}, {"packed_filter", packed_filter}, {"he_cutblind", he_cutblind}, {"lam_iterate", lam_iterate},
        {"adapt_s", adapt_s}, {"merge_prefilter", merge_prefilter}, {"verify_final", verify_final}};
    result["ms61_counters"] = {
        {"dropbacks", dropback_count}, {"early_cut_rounds", early_cut_rounds},
        {"early_cut_count", early_cut_count}, {"near_columns_added", near_columns_added},
        {"near_columns_discarded", near_columns_discarded},
        {"theta_size", relaxation_size()}, {"merge_prefilter_hits", lbl.merge_prefilter_hits},
        {"verify_final_runs", verify_final_runs}, {"verify_final_catches", verify_final_catches},
        {"verify_final_columns", verify_final_columns}, {"theta_growths", theta_growths}, {"dssr_active", dssr_active}, {"dssr_auto_switch_exact_call", dssr_auto_switch_exact_call}, {"dssr_reruns", dssr_reruns}};
    result["exact_log"] = log;
    result["bound_events"] = bound_events;
    result["closure_census"] = closure_census();
    result["exact_pricing"] = {{"calls", exact_calls}, {"finished", exact_finished}};
    result["checker_infeasible_columns"] = checker_infeasible_count;
    // kayros (M13.2): guard observability, never used in control flow.
    result["memory"] = {
        {"limit_mb", params.memory_limit_mb},
        {"peak_rss_mb", (double)MemoryMonitor::PeakRSSBytes() / 1048576.0},
        {"limit_reached", MemoryMonitor::Exceeded()},
    };
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
