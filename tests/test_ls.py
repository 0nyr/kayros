"""M3.7 TD-LS gates.

Discipline per the Stream-4 decision memo (td-ls-structures-decision.md):
- tree-based splice evaluations must agree with direct checker-fold pricing of
  the spliced vertex vector on FEASIBILITY always; on value they may differ by
  association-class noise only (P4.1) — asserted tightly on Dabia (no vertical
  steps), feasibility-only on step families;
- local_search output must be a valid solution whose value is the canonical
  checker Duration, never worse than its input;
- solve() with LS stays checker-refereed end to end (enforced by solve itself).
"""

import random

import pytest

import kayros
from kayros import _core
from kayros.io import load_instance, to_core

from conftest import family_instances, require_benchmarks

REL_TOL = 1e-9


def spliced(route1, i1, j1, route2, i2, j2):
    segment = route2[i2 : j2 + 1] if i2 <= j2 else []
    return route1[:i1] + segment + route1[j1 + 1 :]


def greedy_routes(core):
    ok, routes = _core.greedy_makespan(core)
    assert ok
    return routes


def load_core(path):
    return to_core(load_instance(path))


def pick_one(problem_type, family, size_dir, name=None):
    paths = family_instances(problem_type, family, [size_dir])
    if name is not None:
        paths = [p for p in paths if p.name.startswith(name)]
    return paths[0] if paths else None


GATE_PICKS = [
    ("TDVRPTW", "Dabia2013", "n=25", "R101", True),
    ("TDVRPTW", "Dabia2013", "n=50", "C103", True),
    ("TDVRP", "Dabia2013", "n=25", "R101", True),
    ("TDVRPTW", "Ari2018", "n=15", None, True),
    ("TDVRPTW", "Vu2020", "n=59", None, True),
    ("TDVRPTW", "Rifki2020", "n=10", None, False),  # vertical steps: feasibility-only
]


def gate_cases():
    require_benchmarks()
    cases = []
    for problem_type, family, size_dir, name, tight in GATE_PICKS:
        path = pick_one(problem_type, family, size_dir, name)
        if path is not None:
            cases.append((path, tight))
    assert cases, "no benchmark instances found"
    return cases


@pytest.fixture(scope="module", params=range(len(GATE_PICKS)))
def gate_case(request):
    cases = gate_cases()
    if request.param >= len(cases):
        pytest.skip("pick not present in this checkout")
    return cases[request.param]


def test_evaluate_splice_matches_fold(gate_case) -> None:
    """Random splices: tree evaluation vs direct evaluate_route on the spliced
    vertex vector — feasibility identical, values association-close."""
    path, tight = gate_case
    core = load_core(path)
    routes = [r for r in greedy_routes(core) if r]
    rng = random.Random(7)
    checked = 0
    for _ in range(400):
        r1 = rng.randrange(len(routes))
        r2 = rng.randrange(len(routes))
        route1, route2 = routes[r1], routes[r2]
        m1, m2 = len(route1), len(route2)
        i1 = rng.randrange(0, m1 + 1)
        j1 = rng.randrange(i1 - 1, m1)
        if rng.random() < 0.3:
            i2, j2 = 1, 0  # empty incoming segment
        else:
            i2 = rng.randrange(0, m2)
            j2 = rng.randrange(i2, m2)
        candidate = spliced(route1, i1, j1, route2, i2, j2)
        if not candidate:
            continue
        feasible, duration, _ = _core.ls_evaluate_splice(
            core, route1, i1, j1, route2, i2, j2
        )
        ref_feasible, ref_duration, _ = core.evaluate_route(candidate)
        assert feasible == ref_feasible, (path.name, route1, i1, j1, route2, i2, j2)
        if feasible and tight:
            # Duration only: the earliest-argmin departure is association-
            # fragile on duration plateaus (P4.1) — committed states take their
            # departure from the checker fold, never from the tree evaluation.
            assert duration == pytest.approx(ref_duration, rel=REL_TOL)
        checked += 1
    assert checked > 100


def test_evaluate_intra_relocate_matches_fold(gate_case) -> None:
    path, tight = gate_case
    core = load_core(path)
    routes = [r for r in greedy_routes(core) if len(r) >= 2]
    if not routes:
        pytest.skip("no route long enough")
    checked = 0
    for route in routes[:6]:
        m = len(route)
        for i in range(m):
            for p in range(m + 1):
                if p in (i, i + 1):
                    continue
                feasible, duration, _ = _core.ls_evaluate_intra_relocate(
                    core, route, i, p
                )
                c = route[i]
                rest = route[:i] + route[i + 1 :]
                q = p if p < i else p - 1
                candidate = rest[:q] + [c] + rest[q:]
                ref_feasible, ref_duration, _ = core.evaluate_route(candidate)
                assert feasible == ref_feasible, (path.name, route, i, p)
                if feasible and tight:
                    assert duration == pytest.approx(ref_duration, rel=REL_TOL)
                checked += 1
    assert checked


def test_local_search_invariants(gate_case) -> None:
    """LS output: valid partition of customers, value = canonical checker
    Duration, never worse than the input, and the whole solution passes the
    reference checker (capacity, TWs, fleet, cost) exactly."""
    from mamut_routing_lib.models import BenchmarkSolution
    from mamut_routing_lib.td import check_td_solution

    path, _ = gate_case
    loaded = load_instance(path)
    core = to_core(loaded)
    routes = greedy_routes(core)
    before = _core.solution_duration(core, routes)
    assert before != float("inf")
    served_before = sorted(c for route in routes for c in route)

    new_routes, value, applied, reverted = _core.ls_local_search(core, routes)
    assert value <= before
    assert value == _core.solution_duration(core, new_routes)
    assert sorted(c for route in new_routes for c in route) == served_before
    assert len(new_routes) <= len(routes)
    assert all(new_routes), "no empty route may survive"
    assert applied >= 0 and reverted >= 0

    check = check_td_solution(
        loaded,
        BenchmarkSolution(
            instance_name=loaded.instance.instance_name,
            routes=[list(r) for r in new_routes],
            cost=value,
        ),
    )
    assert check.is_valid(), (check.status, check.error_message)
    assert check.routing_cost == value


@pytest.mark.parametrize(
    "problem_type,size_dir,name",
    [("TDVRPTW", "n=25", "R101"), ("TDVRPTW", "n=50", "R205"), ("TDVRP", "n=25", "C101")],
)
def test_local_search_improves_greedy_dabia(problem_type, size_dir, name) -> None:
    require_benchmarks()
    path = pick_one(problem_type, "Dabia2013", size_dir, name)
    if path is None:
        pytest.skip("instance not present")
    core = load_core(path)
    routes = greedy_routes(core)
    before = _core.solution_duration(core, routes)
    _, value, applied, _ = _core.ls_local_search(core, routes)
    assert value < before, "TD-LS should improve the raw greedy seed"
    assert applied > 0


def test_solve_with_ls_beats_construction_only() -> None:
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", "n=25", "R101")
    if path is None:
        pytest.skip("instance not present")
    budget = kayros.Params(strategy="aco", max_iterations=40, max_no_improvement=40)
    budget_no_ls = kayros.Params(
        strategy="aco",
        max_iterations=40, max_no_improvement=40, local_search=False
    )
    with_ls = kayros.solve(path, budget, seed=3)
    without = kayros.solve(path, budget_no_ls, seed=3)
    assert with_ls.duration <= without.duration


def test_solve_with_ls_deterministic() -> None:
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", "n=25", "C101")
    if path is None:
        pytest.skip("instance not present")
    budget = kayros.Params(strategy="aco", max_iterations=30, max_no_improvement=30)
    a = kayros.solve(path, budget, seed=11)
    b = kayros.solve(path, budget, seed=11)
    assert a.routes == b.routes
    assert a.duration == b.duration
