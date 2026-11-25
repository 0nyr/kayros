#pragma once

#include <gmpxx.h>
#include <optional>

#include "heuristics/heuristics.h"

namespace kayros {
using Route = std::vector<std::int32_t>;
using Routes = std::vector<Route>;
using ReferenceCallback = std::function<void(const Routes&, const mpq_class&, double, std::uint64_t, std::int32_t)>;

struct ReferenceResult {
    Routes routes;
    std::optional<mpq_class> exact_cost;
    SolveStatus status = SolveStatus::Infeasible;
    std::uint64_t iterations_run = 0;
    std::int64_t work_units = 0;
    std::int64_t restarts = 0;
    KStats k_stats;
    KCapStats k_cap_stats;
    FleetKickStats fleet_stats;
    FdStats fd_stats;
};

ReferenceResult solve_reference(const Instance& instance, const IlsParams& params, std::uint64_t seed,
                                double time_limit, const ReferenceCallback& notify, const Routes& initial_routes,
                                bool fleet_cost_objective);
}  // namespace kayros
