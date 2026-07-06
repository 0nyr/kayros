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
    // route is repriced under Lera arithmetic and added as an initial SPF
    // column; when the surviving routes partition the customers exactly, their
    // Lera-arithmetic total becomes the initial UB for pruning.
    std::vector<std::vector<int>> initial_routes;
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
