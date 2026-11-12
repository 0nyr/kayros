// kayros-owned bridge to the vendored Lera-Romero BPC (NOT vendored code).
// Driver adapted from TDVRPTW-solver commit 8329844 main_bp.cpp (Lera's last
// all-features-working state) onto the current labeling API.
#pragma once

#include <climits>
#include <functional>
#include <string>
#include <vector>

namespace kayros::lera {

struct SolveParams {
    double time_limit_s = 7200.0;
    int cut_limit = 100;
    int node_limit = INT_MAX;
    int solution_limit = 3000;  // labeling negative-reduced-cost pool size
    // Warm start (M5.3): heuristic incumbent routes as customer-id sequences
    // (MAMUT numbering 1..n, no depots — e.g. kayros Solution.routes). Each
    // route is repriced (checker-exact fold since M5.6) and added as an
    // initial SPF column; when the surviving routes partition the customers
    // exactly, their total becomes the initial UB for pruning.
    std::vector<std::vector<int>> initial_routes;
    // Memory self-guard (M13.2): RSS watermark in MiB; <= 0 disables. When the
    // process RSS crosses it, every component unwinds at its next check point
    // (the same loop heads as the M5.2 deadline) and the solve returns an
    // honest MemoryLimitReached status with valid bounds, instead of the
    // OS OOM-killing the process with no verdict. The Python bridge resolves
    // its default from the machine (own RSS + ~80% of available memory,
    // capped by the cgroup limit); this field is the resolved value.
    double memory_limit_mb = 0.0;
    // Dual stabilization (M5.1b): Neame smoothing factor in [0, 1); pricing
    // runs on the EMA alpha*center + (1-alpha)*duals with misprice-safe
    // termination (CG only ever stops on true duals). Default OFF: measured
    // monotonically harmful with this pool-based pricing ladder (thousands of
    // columns per pass tuned to the wrong duals beat any oscillation damping
    // — C103: 15 s at 0.0, 37 s at 0.3, TL at >=0.5). Experimental knob.
    double stab_alpha = 0.0;
    // Pricing options (1.7.0). The defaults are the configuration validated
    // against the repaired Lera-Romero control on Dabia2013 n=100. Every
    // option keeps the termination rule: a node is declared optimal only
    // after an Exact pricing pass on true duals that finishes without a new
    // negative column, followed by a cut round that finds no violated cut.
    //  - near_rel: a productive Exact pass also contributes up to near_limit
    //    routes whose reduced cost lies in [0, near_rel * mean customer
    //    dual), the mean being the sum of the positive customer duals
    //    divided by the number of customers. 0 disables it. The labeling never prunes on reduced cost, so
    //    these routes cost only merges, and they spare later Exact passes.
    //  - heuristic_cut_blind: the heuristic elementarity level ignores cut
    //    memory in dominance. With many active subset-row cuts that memory
    //    blocks nearly every dominance and a heuristic pass becomes slower
    //    than an Exact one.
    //  - packed_filter, merge_prefilter: execution only, identical search.
    //  - last_arc_merge_iterate: the last-arc merge walks the leftover queue
    //    instead of popping it (same merges, different order).
    // Each has a KAYROS_BPC_* environment override for experiments, as do
    // the experimental levers that stay off (see lera_bridge.cpp). A set
    // override takes precedence over the value given here, without warning.
    double near_rel = 0.10;
    int near_limit = 3000;
    bool heuristic_cut_blind = true;
    bool packed_filter = true;
    bool merge_prefilter = true;
    bool last_arc_merge_iterate = true;
    // Anytime hook (M5.2): called synchronously on every new BCP incumbent
    // with the incumbent record as a JSON string ({time, value, origin,
    // routes}); the same records are also collected in the result's
    // "incumbents" array. Keep it cheap — the solve blocks on it.
    std::function<void(const std::string& incumbent_json)> on_incumbent;
    std::function<void(const std::string&)> on_event;
};

// payload: normalized Lera instance JSON (built by the kayros.lera Python
// bridge from a loaded MAMUT TD instance + ATF sidecars; travel_times already
// present). Runs Lera's shared preprocessing (capacity, service+waiting
// folding, TW tightening, depot triangle arc removal), then the duration-
// objective BCP. Returns a result JSON string.
std::string solve_duration_json(const std::string& payload, const SolveParams& params);

}  // namespace kayros::lera
