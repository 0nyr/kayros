// An LP outcome that the backend does not map to a known status must stop the
// search without incumbent or bound (defect D1 of the M2 inventory, HiGHS
// backend). The search cases inject DidNotStart, the status that an
// unmapped LP status produced before the repair, and Unresolved.
#include <goc/goc.h>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "goc/linear_programming/colgen/colgen.h"

#include "bcp/bcp.h"
#include "bcp/spf.h"

using namespace goc;

template <class Status>
static std::string name(Status status)
{
    std::ostringstream out;
    out << status;
    return out.str();
}

static void require(bool value, const std::string& what)
{
    if (!value) { std::cerr << "lp unresolved regression: " << what << "\n"; std::exit(1); }
}

// min x + y  s.t.  x + 2y >= 1, 3x + y >= 1, x, y >= 0. The slack basis is
// infeasible, so the simplex needs at least 1 iteration.
static Formulation* small_lp()
{
    Formulation* formulation = LPSolver::NewFormulation();
    formulation->Minimize(Expression());
    formulation->AddConstraint(Expression().GEQ(1.0));
    formulation->AddConstraint(Expression().GEQ(1.0));
    Variable x = formulation->AddVariable("x", VariableDomain::Real, 0.0, INFTY);
    Variable y = formulation->AddVariable("y", VariableDomain::Real, 0.0, INFTY);
    formulation->SetConstraintCoefficient(0, x, 1.0);
    formulation->SetConstraintCoefficient(0, y, 2.0);
    formulation->SetConstraintCoefficient(1, x, 3.0);
    formulation->SetConstraintCoefficient(1, y, 1.0);
    formulation->SetObjectiveCoefficient(x, 1.0);
    formulation->SetObjectiveCoefficient(y, 1.0);
    return formulation;
}

static void lp_and_colgen()
{
    {
        Formulation* formulation = small_lp();
        LPSolver solver;
        auto log = solver.Solve(formulation, {LPOption::Duals, LPOption::Incumbent});
        require(name(*log.status) == "Optimum", "control LP status " + name(*log.status));
        require(log.incumbent_value.IsSet() && log.duals.IsSet(), "control LP value and duals");
        delete formulation;
    }
    // An iteration limit is a HiGHS status outside the mapped ones.
    {
        Formulation* formulation = small_lp();
        LPSolver solver;
        solver.config = {{"simplex_iteration_limit", 0}, {"presolve", "off"}};
        auto log = solver.Solve(formulation, {LPOption::Duals, LPOption::Incumbent});
        require(name(*log.status) == "Unresolved", "iteration-limited LP status " + name(*log.status));
        require(!log.incumbent_value.IsSet() && !log.duals.IsSet() && !log.incumbent.IsSet(),
                "an unresolved LP carries no value, duals or solution");
        delete formulation;
    }
    {
        Formulation* formulation = small_lp();
        LPSolver solver;
        solver.config = {{"simplex_iteration_limit", 0}, {"presolve", "off"}};
        int pricing_calls = 0;
        PricingFunction pricing = [&](const std::vector<double>&, double, Duration, CGExecutionLog*) { ++pricing_calls; };
        auto log = solve_colgen(formulation, nullptr, Duration(60, DurationUnit::Seconds), pricing, &solver, {});
        require(name(*log.status) == "Unresolved", "column generation status " + name(*log.status));
        require(pricing_calls == 0, "no pricing after an unresolved LP");
        require(!log.incumbent.IsSet() && !log.incumbent_value.IsSet(), "column generation carries no solution");
        delete formulation;
    }
}

// 3 customers, every pair route and every singleton of cost 1. The root
// relaxation is the 3 pairs at 1/2 (value 1.5) and the optimum is 2. Cuts
// are off so that the root stays fractional.
struct Search
{
    Digraph D{5};
    solver::SPF spf{5};
    int pricing_calls = 0, incumbents = 0;

    Search()
    {
        for (int i = 0; i < 4; ++i)
            for (int j = 1; j < 5; ++j)
                if (i != j && !(i == 0 && j == 4)) D.AddArc({i, j});
        for (const GraphPath& path : std::vector<GraphPath>{
                 {0, 1, 4}, {0, 2, 4}, {0, 3, 4}, {0, 1, 2, 4}, {0, 2, 3, 4}, {0, 1, 3, 4}})
            spf.AddRoute(Route(path, 0.0, 1.0), 1);
    }

    // The injected status is the one an unmapped LP status produced before
    // the repair, after the given number of pricing calls without column.
    BCPExecutionLog Run(int clean_calls, CGStatus injected, VRPSolution* solution)
    {
        solver::BCP bcp(D, &spf);
        bcp.time_limit = Duration(60, DurationUnit::Seconds);
        bcp.cut_limit = 0;
        bcp.node_limit = 1000;
        bcp.pricing_solver = [&](const solver::PricingProblem&, int, Duration, CGExecutionLog* log) {
            if (++pricing_calls > clean_calls) log->status = injected;
        };
        bcp.on_incumbent = [&](Duration, const VRPSolution&, const std::string&) { ++incumbents; };
        return bcp.Run(solution);
    }
};

static void search()
{
    // Control: the search closes at 2 after branching. Without a ledger the
    // status is Unresolved for that reason only (M2 step 5).
    {
        Search s;
        VRPSolution solution(INFTY, {});
        auto log = s.Run(1000, CGStatus::Optimum, &solution);
        require(name(log.status) == "Unresolved", "control search status " + name(log.status));
        require(log.root_lp_value.IsSet() && epsilon_equal(*log.root_lp_value, 1.5), "control root LP value");
        require(log.best_int_value.IsSet() && epsilon_equal(*log.best_int_value, 2.0), "control incumbent");
        require(s.pricing_calls > 1, "control search processes a child node");
    }
    for (CGStatus injected : {CGStatus::DidNotStart, CGStatus::Unresolved})
    {
        // At the root: no claim, no bound, no incumbent.
        {
            Search s;
            VRPSolution solution(INFTY, {});
            auto log = s.Run(0, injected, &solution);
            require(name(log.status) == "Unresolved", "root search status " + name(log.status));
            require(!log.root_lp_value.IsSet() && !log.best_bound.IsSet(), "an unresolved root gives no bound");
            require(!log.best_int_value.IsSet() && s.incumbents == 0 && solution.value == INFTY,
                    "an unresolved root gives no incumbent");
        }
        // At the first child: the incumbent of the freeze heuristic and the
        // root bound remain, the search is not closed.
        {
            Search s;
            VRPSolution solution(INFTY, {});
            auto log = s.Run(1, injected, &solution);
            require(name(log.status) == "Unresolved", "child search status " + name(log.status));
            require(s.incumbents == 1 && log.best_int_value.IsSet() && epsilon_equal(*log.best_int_value, 2.0),
                    "only the freeze heuristic incumbent");
            require(log.best_bound.IsSet() && epsilon_equal(*log.best_bound, 1.5), "the bound stays at the root value");
        }
    }
}

int main()
{
    lp_and_colgen();
    search();
    std::cout << "lp unresolved: ok\n";
    return 0;
}
