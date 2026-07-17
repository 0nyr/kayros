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
    // Anytime hook (M5.2): called synchronously on every new BCP incumbent
    // with the incumbent record as a JSON string ({time, value, origin,
    // routes}); the same records are also collected in the result's
    // "incumbents" array. Keep it cheap — the solve blocks on it.
    std::function<void(const std::string& incumbent_json)> on_incumbent;
};

// payload: normalized Lera instance JSON (built by the kayros.lera Python
// bridge from a loaded MAMUT TD instance + ATF sidecars; travel_times already
// present). Runs Lera's shared preprocessing (capacity, service+waiting
// folding, TW tightening, depot triangle arc removal), then the duration-
// objective BCP. Returns a result JSON string.
std::string solve_duration_json(const std::string& payload, const SolveParams& params);

}  // namespace kayros::lera
