// Exact column costs in the master (M2 step 2 of the design note
// reports/design/2026-11-26-m2-exact-soundness-design.md in the workspace).
// The master keeps the exact cost of every column and gives its nearest
// binary64 value to the LP. The bridge evaluates each route with the exact
// header and refuses a route that the header finds infeasible. The master
// holds 1 artificial column per customer that branching never fixes. The
// incumbent value is the exact cost of a solution, never an LP or MIP value.
#include <goc/goc.h>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "bcp/bcp.h"
#include "bcp/exact_cost.h"
#include "bcp/spf.h"
#include "exact_columns.h"

using namespace goc;
using solver::ExactCost;

static void require(bool value, const std::string& what)
{
    if (!value) { std::cerr << "exact master regression: " << what << "\n"; std::exit(1); }
}

template <class Status>
static std::string name(Status status)
{
    std::ostringstream out;
    out << status;
    return out.str();
}

static ExactCost power_of_two(int exponent)
{
    ExactCost q = 1;
    for (int k = 0; k < exponent; ++k) q *= 2;
    return q;
}

// References from Python float(Fraction(...)), which rounds to nearest, ties to even.
static void rounding()
{
    using solver::binary64_at_least;
    using solver::nearest_binary64;
    const ExactCost two53 = power_of_two(53), two54 = power_of_two(54);
    require(nearest_binary64(ExactCost(1, 10)) == 0x1.999999999999ap-4, "1/10 rounds up, mpq_get_d would truncate");
    require(nearest_binary64(ExactCost(1, 3)) == 0x1.5555555555555p-2, "1/3 rounds down");
    require(nearest_binary64(1 + 1 / two53) == 1.0, "a tie goes to the even neighbour below");
    require(nearest_binary64(1 + 3 / two53) == 0x1.0000000000002p+0, "a tie goes to the even neighbour above");
    require(nearest_binary64(1 + 3 / two54) == 0x1.0000000000001p+0, "3/4 of an ulp rounds up");
    require(nearest_binary64(-(1 + 3 / two54)) == -0x1.0000000000001p+0, "negative values round to nearest");
    require(nearest_binary64(ExactCost(4128)) == 4128.0, "an integer is exact");
    require(binary64_at_least(ExactCost(1, 10)) == 0x1.999999999999ap-4, "1/10 rounded up");
    require(binary64_at_least(ExactCost(1, 3)) == 0x1.5555555555556p-2, "1/3 rounded up");
    require(binary64_at_least(ExactCost(7)) == 7.0, "an exact value is kept");
}

// 2 customers, horizon [0, 100]. Arc (0, 1) has a slope of 1/3 on [0, 3] and
// customer 1 closes at 10.2, so route (1) departs at 3 * (10.2 - 10) and
// lasts 50 - 2 * 10.2 in exact arithmetic. Customer 2 closes at 5 and is
// reached at 20 at the earliest, so every route through 2 is infeasible.
static kayros::Instance small_instance(std::int64_t capacity)
{
    kayros::Instance ci;
    ci.num_customers = 2;
    ci.vehicle_capacity = capacity;
    ci.horizon_start = 0.0;
    ci.horizon_end = 100.0;
    ci.has_time_windows = true;
    ci.demands = {0, 4, 1};
    ci.service_times = {0.0, 0.0, 0.0};
    ci.tw_earliest = {0.0, 0.0, 0.0};
    ci.tw_latest = {100.0, 10.2, 5.0};
    std::vector<std::vector<double>> xs(9), ys(9);
    auto arc = [&](int i, int j, std::vector<double> x, std::vector<double> y) { xs[i * 3 + j] = x; ys[i * 3 + j] = y; };
    arc(0, 1, {0.0, 3.0, 100.0}, {10.0, 11.0, 110.0});
    arc(1, 0, {0.0, 100.0}, {20.0, 120.0});
    arc(0, 2, {0.0, 100.0}, {20.0, 120.0});
    arc(2, 0, {0.0, 100.0}, {20.0, 120.0});
    arc(1, 2, {0.0, 100.0}, {5.0, 105.0});
    arc(2, 1, {0.0, 100.0}, {5.0, 105.0});
    ci.atf_offset.assign(10, 0);
    for (int a = 0; a < 9; ++a) {
        ci.atf_offset[a + 1] = ci.atf_offset[a] + (std::int64_t)xs[a].size();
        ci.atf_xs.insert(ci.atf_xs.end(), xs[a].begin(), xs[a].end());
        ci.atf_ys.insert(ci.atf_ys.end(), ys[a].begin(), ys[a].end());
    }
    return ci;
}

static void columns()
{
    using kayros::lera::exact_column;
    const kayros::Instance ci = small_instance(10);
    kayros::ExactRoutePricer pricer(ci);
    const auto one = exact_column(pricer, ci, GraphPath{0, 1, 3});
    const ExactCost close(10.2);
    require(one.feasible, "route (1) is feasible");
    require(one.cost == 50 - 2 * close, "route (1) has its exact cost, not a binary64 one");
    require(one.departure == solver::nearest_binary64(3 * (close - 10)), "route (1) departs at the exact minimizer");
    require(!exact_column(pricer, ci, GraphPath{0, 2, 3}).feasible, "route (2) misses the window of 2");
    require(!exact_column(pricer, ci, GraphPath{0, 1, 2, 3}).feasible, "route (1, 2) misses the window of 2");

    const kayros::Instance tight = small_instance(3);
    kayros::ExactRoutePricer tight_pricer(tight);
    require(!exact_column(tight_pricer, tight, GraphPath{0, 1, 3}).feasible, "route (1) exceeds the capacity");

    // n = 2 routes of at most 120 - 0 each, plus 1.
    require(kayros::lera::artificial_cost(ci) == 241, "artificial cost above every solution");
}

// The LP coefficient of a column is the nearest binary64 value of its exact
// cost, and the artificial columns keep every restricted master feasible.
static void master()
{
    solver::SPF spf(4);
    spf.AddRoute(Route({0, 1, 3}, 0.0, 99.0), ExactCost(1, 10));
    spf.AddArtificial(1, 1000);
    spf.AddArtificial(2, 1000);
    const auto& route = spf.RouteOf(spf.formulation->VariableAtIndex(0));
    require(route.duration == 0x1.999999999999ap-4, "the stored route carries the nearest binary64 cost");
    require(spf.ExactCostOf(spf.formulation->VariableAtIndex(0)) == ExactCost(1, 10), "the master keeps the exact cost");
    require(!spf.IsArtificial(spf.formulation->VariableAtIndex(0)) && spf.IsArtificial(spf.formulation->VariableAtIndex(1)),
            "artificial columns are marked");
    bool refused = false;
    try { spf.AddArtificial(1, ExactCost(1, 10)); } catch (const std::invalid_argument&) { refused = true; }
    require(refused, "an artificial cost must be a binary64 value");
    // Forbidding the arcs of route (1) fixes the route and not the artificial columns.
    spf.SetForbiddenArcs({{0, 1}, {1, 3}});
    LPSolver lp;
    auto log = lp.Solve(spf.formulation, {LPOption::Incumbent});
    require(name(*log.status) == "Optimum", "the master stays feasible after branching, status " + name(*log.status));
    require(*log.incumbent_value == 2000.0, "the master covers both customers with artificial columns");
}

struct Search
{
    Digraph D{4};
    solver::SPF spf{4};
    int incumbents = 0;
    std::vector<std::string> origins;

    Search()
    {
        for (int i = 0; i < 3; ++i)
            for (int j = 1; j < 4; ++j)
                if (i != j && !(i == 0 && j == 3)) D.AddArc({i, j});
    }

    BCPExecutionLog Run(solver::BCP& bcp, VRPSolution* solution)
    {
        bcp.time_limit = Duration(60, DurationUnit::Seconds);
        bcp.cut_limit = 0;
        bcp.node_limit = 1000;
        bcp.pricing_solver = [](const solver::PricingProblem&, int, Duration, CGExecutionLog*) {};
        bcp.on_incumbent = [&](Duration, const VRPSolution&, const std::string& origin) { ++incumbents; origins.push_back(origin); };
        return bcp.Run(solution);
    }
};

static void search()
{
    // The incumbent value is the exact cost 1/10 + 2/10 = 3/10, whose nearest
    // binary64 value 0.3 differs from the binary64 sum 0.1 + 0.2.
    {
        Search s;
        s.spf.AddRoute(Route({0, 1, 3}, 0.0, 0.0), ExactCost(1, 10));
        s.spf.AddRoute(Route({0, 2, 3}, 0.0, 0.0), ExactCost(2, 10));
        s.spf.AddArtificial(1, 1000);
        s.spf.AddArtificial(2, 1000);
        solver::BCP bcp(s.D, &s.spf);
        VRPSolution solution(INFTY, {});
        auto log = s.Run(bcp, &solution);
        require(name(log.status) == "Optimum", "exact incumbent search status " + name(log.status));
        require(bcp.ExactIncumbentValue() && *bcp.ExactIncumbentValue() == ExactCost(3, 10), "exact incumbent value 3/10");
        require(log.best_int_value.IsSet() && *log.best_int_value == 0.3 && 0.1 + 0.2 != 0.3,
                "the binary64 incumbent value is the nearest value of the exact cost");
        require(solution.value == 0.3 && solution.routes.size() == 2 && s.incumbents == 1, "the solution carries the exact value");
        require(bcp.artificial_candidate_count == 0 && bcp.rejected_candidate_count == 0, "no rejected candidate");
    }
    // Customer 2 has no route. The root is integral on an artificial column:
    // it gives no incumbent, where the master without artificial columns was
    // infeasible.
    {
        Search s;
        s.spf.AddRoute(Route({0, 1, 3}, 0.0, 0.0), 1);
        s.spf.AddArtificial(1, 1000);
        s.spf.AddArtificial(2, 1000);
        solver::BCP bcp(s.D, &s.spf);
        VRPSolution solution(INFTY, {});
        auto log = s.Run(bcp, &solution);
        require(log.root_lp_value.IsSet() && *log.root_lp_value == 1001.0, "the root LP covers 2 with its artificial column");
        require(bcp.cg_infeasible_count == 0, "the restricted master is never infeasible");
        require(bcp.artificial_candidate_count == 1, "the integral root is a candidate with an artificial column");
        require(!log.best_int_value.IsSet() && s.incumbents == 0 && solution.value == INFTY, "no incumbent from an artificial column");
    }
    // The exact evaluation enforces the fleet limit, which the master lacks.
    {
        Search s;
        s.spf.AddRoute(Route({0, 1, 3}, 0.0, 0.0), 1);
        s.spf.AddRoute(Route({0, 2, 3}, 0.0, 0.0), 1);
        s.spf.AddRoute(Route({0, 1, 2, 3}, 0.0, 0.0), 3);
        s.spf.AddArtificial(1, 1000);
        s.spf.AddArtificial(2, 1000);
        solver::BCP bcp(s.D, &s.spf);
        bcp.fleet_limit = 1;
        VRPSolution solution(INFTY, {});
        s.Run(bcp, &solution);
        require(bcp.rejected_candidate_count == 1, "2 routes exceed a fleet of 1");
        require(!bcp.ExactIncumbentValue() && s.incumbents == 0, "a rejected candidate never becomes the incumbent");
    }
    // A warm-start value is exact, and a candidate of equal exact cost does not replace it.
    {
        Search s;
        s.spf.AddRoute(Route({0, 1, 3}, 0.0, 0.0), ExactCost(1, 10));
        s.spf.AddRoute(Route({0, 2, 3}, 0.0, 0.0), ExactCost(2, 10));
        s.spf.AddArtificial(1, 1000);
        s.spf.AddArtificial(2, 1000);
        solver::BCP bcp(s.D, &s.spf);
        bcp.SetInitialIncumbent(ExactCost(3, 10));
        VRPSolution solution(INFTY, {});
        auto log = s.Run(bcp, &solution);
        require(name(log.status) == "Optimum", "warm search status " + name(log.status));
        require(s.incumbents == 0 && *bcp.ExactIncumbentValue() == ExactCost(3, 10), "an equal exact cost is not an improvement");
        require(*log.best_int_value == 0.3 && solution.routes.empty(), "the warm-start value is kept");
    }
}

int main()
{
    rounding();
    columns();
    master();
    search();
    std::cout << "exact master: ok\n";
    return 0;
}
