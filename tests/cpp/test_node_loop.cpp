// Wiring of the search to the ledger of the exact layer (M2 design note of
// session 139, section 12). The helpers of bcp/exact_wiring.h, and whole searches
// in exact mode on small instances whose optimum is known.
//
// Built twice. Without KAYROS_WIRING_MUTANT_TEST it checks that the searches
// certify the known optimum and that the ledger record replays. With it, the
// file is compiled together with a copy of bcp.cpp in which 1 exit of the wiring
// was broken (see cpp/lera/CMakeLists.txt), and it exits 0 only when the broken
// exit was reached and the run ends without claim.
#include <goc/goc.h>

#include <cstdlib>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "bcp/bcp.h"
#include "bcp/exact_wiring.h"
#include "bcp/spf.h"
#include "exact/ledger.h"

using namespace goc;
using solver::ExactCost;
namespace ex = kayros::exact;
namespace w = solver::wiring;

#ifdef KAYROS_WIRING_MUTANT_TEST
static const bool mutant = true;
#else
static const bool mutant = false;
#endif

template <class Status>
static std::string name(Status status)
{
	std::ostringstream out;
	out << status;
	return out.str();
}

static int failures = 0;
static void require(bool value, const std::string& what)
{
	if (value) return;
	++failures;
	std::cerr << "node loop: " << what << "\n";
	if (!mutant) std::exit(1);
}

// ---------- helpers ----------

static void helpers()
{
	const w::VertexMap map{3};
	require(map.ledger_tail(0) == 0 && map.ledger_head(4) == 0 && map.ledger_tail(2) == 2 && map.ledger_head(2) == 2, "depot mapping");
	bool threw = false;
	try { (void)map.ledger_head(0); } catch (const std::invalid_argument&) { threw = true; }
	require(threw, "the start depot is no head");
	threw = false;
	try { (void)map.ledger_tail(4); } catch (const std::invalid_argument&) { threw = true; }
	require(threw, "the end depot is no tail");
	require(map.search_arc(2, 0).tail == 2 && map.search_arc(2, 0).head == 4 && map.search_arc(0, 1).head == 1, "ledger arcs to search arcs");
	require(map.search_path({3, 1}) == GraphPath({0, 3, 1, 4}), "ledger route to search path");
	require(map.customers_of(GraphPath{0, 3, 1, 4}) == std::vector<int>({3, 1}), "search path to customers");
	threw = false;
	try { (void)map.customers_of(GraphPath{0, 3, 1, 0}); } catch (const std::invalid_argument&) { threw = true; }
	require(threw, "a search path must end at n+1");

	// Reduced cost to cost, with a cut visited twice and a cut visited once.
	w::Multipliers m;
	m.pi = {0, ExactCost(1, 3), 2, ExactCost(-1, 7)};
	m.cuts = {{1, 2, 3}, {1, 2, 3}};
	m.sigma = {ExactCost(-1, 5), -1};
	require(w::cost_from_reduced({1, 2}, ExactCost(1, 2), m) == ExactCost(1, 2) + ExactCost(1, 3) + 2 - ExactCost(1, 5) - 1, "cost from reduced cost, 2 cut visits");
	require(w::cost_from_reduced({3}, 0, m) == ExactCost(-1, 7), "cost from reduced cost, 1 cut visit");

	// Multipliers of an exact solution, with the depot rows dropped and the cuts after the n+2 rows.
	kayros::exact_lp::Solution sol;
	sol.status = kayros::exact_lp::Status::Optimal;
	sol.duals = {7, 1, 2, 3, 9, ExactCost(-1, 2)};
	const auto mm = w::multipliers_of(sol, 3, 1, [](int) { return std::array<int, 3>{1, 2, 3}; });
	require(mm.pi == std::vector<ex::Q>({0, 1, 2, 3}) && mm.sigma == std::vector<ex::Q>({ExactCost(-1, 2)}) && mm.cuts.size() == 1, "multipliers of the rows");
	threw = false;
	try { (void)w::multipliers_of(sol, 3, 0, [](int) { return std::array<int, 3>{}; }); } catch (const std::invalid_argument&) { threw = true; }
	require(threw, "multipliers of another master are refused");

	sol.primal = {{0, 1}, {4, 1}};
	require(w::integral_columns(sol) == std::vector<int>({0, 4}), "integral primal");
	sol.primal = {{0, 1}, {4, ExactCost(1, 2)}, {5, ExactCost(1, 2)}};
	require(!w::integral_columns(sol), "fractional primal");

	// Forbidden arcs of the ledger outside the search digraph only fix columns.
	Digraph D(5);
	D.AddArc({0, 1}); D.AddArc({1, 2}); D.AddArc({2, 4});
	const auto [in_d, outside] = w::split_forbidden({{0, 1}, {1, 3}, {2, 0}, {3, 0}}, map, D);
	require(in_d.size() == 2 && outside.size() == 2 && outside[0].tail == 1 && outside[0].head == 3 && outside[1].head == 4, "split of the forbidden arcs");
}

// ---------- searches in exact mode ----------

// n customers of demand 1 and capacity cap. Every arc out of the depot and
// into the depot takes `depot` time, every arc between customers `between`,
// constant over the horizon [0, 100]. No windows, no service.
static ex::InstanceData instance(int n, long long cap, double depot, double between)
{
	ex::InstanceData d;
	d.capacity = cap;
	d.lo = 0, d.hi = 100;
	for (int v = 1; v <= n; ++v) d.customers.push_back({std::nullopt, 0.0, 1});
	for (int i = 0; i <= n; ++i)
		for (int j = 0; j <= n; ++j)
		{
			if (i == j) continue;
			const double t = (i == 0 || j == 0) ? depot : between;
			d.arcs.push_back({i, j, {0.0, 100.0}, {t, 100.0 + t}});
		}
	return d;
}

struct Search
{
	int n;
	Digraph D;
	solver::SPF spf;
	ex::Ledger ledger;
	std::set<GraphPath> paths;
	int admitted = 0;

	Search(int n, long long cap, double depot, double between, const std::vector<GraphPath>& seeds)
		: n(n), D(n + 2), spf(n + 2), ledger(instance(n, cap, depot, between), std::nullopt)
	{
		for (int i = 0; i <= n; ++i)
			for (int j = 1; j <= n + 1; ++j)
				if (i != j && !(i == 0 && j == n + 1)) D.AddArc({i, j});
		for (const GraphPath& p: seeds) Add(p);
		for (int i = 1; i <= n; ++i) spf.AddArtificial(i, 1024);
	}

	ExactCost Cost(const GraphPath& p)
	{
		const auto c = ex::path_duration(ledger.instance(), w::VertexMap{n}.customers_of(p));
		if (!c) throw std::logic_error("infeasible seed");
		return *c;
	}

	void Add(const GraphPath& p)
	{
		paths.insert(p);
		spf.AddRoute(Route(p, 0.0, 0.0), Cost(p));
	}

	// When set, replaces the outcome of every admission (tests of the refusals).
	std::optional<solver::BCP::Admission> forced;

	BCPExecutionLog Run(solver::BCP& bcp, VRPSolution* solution)
	{
		bcp.time_limit = Duration(60, DurationUnit::Seconds);
		bcp.cut_limit = 0;
		bcp.node_limit = 1000;
		bcp.ledger = &ledger;
		// The binary64 pricing adds nothing, so every new column comes from the exact pass.
		bcp.pricing_solver = [](const solver::PricingProblem&, int, Duration, CGExecutionLog*) {};
		bcp.admit_route = [&](const GraphPath& p, const ExactCost& expected) {
			if (forced) return *forced;
			if (paths.count(p)) return solver::BCP::Admission::Duplicate;
			const auto c = ex::path_duration(ledger.instance(), w::VertexMap{n}.customers_of(p));
			if (!c) return solver::BCP::Admission::Rejected;
			if (*c != expected) return solver::BCP::Admission::CostMismatch;
			++admitted;
			Add(p);
			return solver::BCP::Admission::Added;
		};
		return bcp.Run(solution);
	}
};

static bool replays(const ex::Ledger& ledger)
{
	std::ostringstream out;
	ledger.write(out);
	std::istringstream in(out.str());
	const ex::Replay r = ex::replay(in);
	return r.certified && r.U && ledger.U() && *r.U == *ledger.U();
}

// 3 customers, capacity 2, every pair and singleton of cost 1. The root
// relaxation is the 3 pairs at 1/2 (value 3/2) and the optimum is 2. The master
// holds 1 direction of each pair, so the exact pass finds the reverse ones
// below the root. The tree branches.
static void triangle()
{
	Search s(3, 2, 0.5, 0.0, {{0, 1, 4}, {0, 2, 4}, {0, 3, 4}, {0, 1, 2, 4}, {0, 2, 3, 4}, {0, 1, 3, 4}});
	solver::BCP bcp(s.D, &s.spf);
	VRPSolution solution(INFTY, {});
	auto log = s.Run(bcp, &solution);
	std::clog << "triangle: status " << name(log.status) << ", ledger open " << s.ledger.open_count()
	          << ", branches " << bcp.ledger_branches << ", pop prunes " << bcp.ledger_removed_by_prune
	          << ", removed by close " << bcp.ledger_removed_by_close << ", admitted " << s.admitted
	          << ", reason '" << bcp.unresolved_reason << "'\n";
#ifndef KAYROS_WIRING_MUTANT_TEST
	require(name(log.status) == "Optimum", "triangle status " + name(log.status));
	require(s.ledger.certified() && s.ledger.U() && *s.ledger.U() == 2, "triangle certified at 2");
	require(bcp.ExactIncumbentValue() && *bcp.ExactIncumbentValue() == 2, "the search prunes with U of the ledger");
	require(bcp.ledger_branches >= 1 && s.admitted >= 1, "the triangle branches and admits routes of the exact pass");
	require(log.root_lp_value.IsSet() && *log.root_lp_value == 1.5, "triangle root LP");
	require(replays(s.ledger), "the triangle record replays to the same U");
#else
	// The broken exit must have been reached on this instance.
#if defined(KAYROS_WIRING_MUTANT_pop_skip_prune)
	require(log.nodes_closed >= 1 && bcp.ledger_branches == 0, "the mutated search popped the root and deleted it unbranched");
#elif defined(KAYROS_WIRING_MUTANT_drop_child)
	require(bcp.ledger_branches >= 1, "the mutated search branched");
#elif defined(KAYROS_WIRING_MUTANT_open_as_removed)
	bool open_seen = false;
	for (const auto& v: bcp.exact_visits) open_seen |= v.outcome == "open";
	require(open_seen, "a close returned no negative route on an open node");
#endif
	require(name(log.status) != "Optimum", "no claim from the broken wiring, status " + name(log.status));
	require(!s.ledger.certified() && s.ledger.open_count() > 0, "the ledger keeps the dropped node open");
#endif
}

// The triangle with a warm start at the optimum, offered through the ledger
// before the search starts.
static void warm()
{
	Search s(3, 2, 0.5, 0.0, {{0, 1, 4}, {0, 2, 4}, {0, 3, 4}, {0, 1, 2, 4}, {0, 2, 3, 4}, {0, 1, 3, 4}});
	solver::BCP bcp(s.D, &s.spf);
	bcp.ledger = &s.ledger;
	require(bcp.OfferInitialRoutes({{1, 2}, {3}}), "warm start accepted by the ledger");
	require(*bcp.ExactIncumbentValue() == 2, "warm start value");
	require(!bcp.OfferInitialRoutes({{1}, {2}, {3}}), "an equal or worse warm start is not accepted");
	VRPSolution solution(INFTY, {});
	auto log = s.Run(bcp, &solution);
#ifndef KAYROS_WIRING_MUTANT_TEST
	require(name(log.status) == "Optimum" && s.ledger.certified() && *s.ledger.U() == 2, "warm search certified at 2");
	require(replays(s.ledger), "the warm record replays");
#else
	(void)log;
#endif
}

// With capacity 1 no pair is feasible and the optimum is the 3 singletons at 3.
// The root is integral and closes by its certificate.
static void singletons()
{
	Search s(3, 1, 0.5, 0.0, {{0, 1, 4}, {0, 2, 4}, {0, 3, 4}});
	solver::BCP bcp(s.D, &s.spf);
	VRPSolution solution(INFTY, {});
	auto log = s.Run(bcp, &solution);
#ifndef KAYROS_WIRING_MUTANT_TEST
	require(name(log.status) == "Optimum" && s.ledger.certified() && *s.ledger.U() == 3, "integral root certified at 3");
	require(bcp.ledger_branches == 0 && bcp.ledger_removed_by_close == 1, "the integral root closes by its certificate");
#else
	(void)log;
#endif
}

// The triangle again, with the admission of the ledger routes refused, or with
// the reverse pairs missing from the search digraph. The exact pass finds them
// below the root, so each run must stop at that exit, with the node open.
static void refusals()
{
	using A = solver::BCP::Admission;
	const std::vector<GraphPath> seeds{{0, 1, 4}, {0, 2, 4}, {0, 3, 4}, {0, 1, 2, 4}, {0, 2, 3, 4}, {0, 1, 3, 4}};
	const std::vector<std::pair<std::optional<A>, std::string>> cases{
		{A::CostMismatch, "route_cost_mismatch"}, {A::Rejected, "route_rejected_by_column_evaluation"},
		{A::Duplicate, "negative_route_already_column"}, {std::nullopt, "route_not_representable"}};
	for (const auto& [forced, reason]: cases)
	{
		Search s(3, 2, 0.5, 0.0, seeds);
		s.forced = forced;
		if (!forced) for (const Arc e: std::vector<Arc>{{2, 1}, {3, 2}, {3, 1}}) s.D.RemoveArc(e);
		solver::BCP bcp(s.D, &s.spf);
		VRPSolution solution(INFTY, {});
		auto log = s.Run(bcp, &solution);
		require(name(log.status) == "Unresolved" && bcp.unresolved_reason == reason,
		        "refusal " + reason + " ends Unresolved for that reason, got " + name(log.status) + " '" + bcp.unresolved_reason + "'");
		require(!s.ledger.certified() && s.ledger.open_count() > 0 && s.admitted == 0, "refusal " + reason + " leaves the node open");
	}
}

// Without a ledger the search runs as before and is never optimal.
static void without_ledger()
{
	Search s(3, 2, 0.5, 0.0, {{0, 1, 4}, {0, 2, 4}, {0, 3, 4}, {0, 1, 2, 4}, {0, 2, 3, 4}, {0, 1, 3, 4}});
	solver::BCP bcp(s.D, &s.spf);
	VRPSolution solution(INFTY, {});
	bcp.time_limit = Duration(60, DurationUnit::Seconds);
	bcp.cut_limit = 0;
	bcp.node_limit = 1000;
	bcp.pricing_solver = [](const solver::PricingProblem&, int, Duration, CGExecutionLog*) {};
	auto log = bcp.Run(&solution);
#ifndef KAYROS_WIRING_MUTANT_TEST
	require(name(log.status) == "Unresolved" && bcp.unresolved_reason == "ledger_unavailable", "no claim without a ledger");
	require(solution.value == 2.0, "the search still finds the incumbent");
#else
	(void)log;
#endif
}

int main()
{
	helpers();
	without_ledger();
	singletons();
	warm();
#ifndef KAYROS_WIRING_MUTANT_TEST
	refusals();
#endif
	triangle();
#ifdef KAYROS_WIRING_MUTANT_TEST
	if (failures == 0) { std::cout << "node loop mutant: caught (no claim)\n"; return 0; }
	std::cerr << "node loop mutant: NOT caught\n";
	return 1;
#else
	std::cout << "node loop: ok\n";
	return 0;
#endif
}
