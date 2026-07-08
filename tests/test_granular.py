"""M7.0 granular-LS gates (Stream 7, td-ils-design.md).

Discipline:
- neighbour lists: symmetric adjacency (union-symmetrised), deterministic,
  depot- and self-free, TW-infeasible edges excluded, monotone in k;
- granular local_search keeps every M3.7 invariant (partition, checker-exact
  value, never-worse) — candidate lists prune enumeration only, the repricing
  rule is untouched;
- with full lists the enumeration is justified everywhere, so granular must
  reproduce the exhaustive descent bitwise on TDVRP (no TW screen). On TDVRPTW
  the swap operator's pair-proximity justification may legitimately skip swaps
  between mutually TW-unreachable clients, so the identity gate is TDVRP-only.
"""

import pytest

import kayros
from kayros import _core
from kayros.io import load_instance, to_core

from conftest import family_instances, require_benchmarks


def load_core(path):
    return to_core(load_instance(path))


def pick_one(problem_type, family, size_dir, name=None):
    paths = family_instances(problem_type, family, [size_dir])
    if name is not None:
        paths = [p for p in paths if p.name.startswith(name)]
    return paths[0] if paths else None


def greedy_routes(core):
    ok, routes = _core.greedy_makespan(core)
    assert ok
    return routes


PICKS = [
    ("TDVRPTW", "Dabia2013", "n=25", "R101"),
    ("TDVRPTW", "Dabia2013", "n=50", "C103"),
    ("TDVRP", "Dabia2013", "n=25", "R101"),
    ("TDVRPTW", "Rifki2020", "n=10", None),  # vertical steps
]


@pytest.fixture(scope="module", params=range(len(PICKS)))
def pick(request):
    require_benchmarks()
    problem_type, family, size_dir, name = PICKS[request.param]
    path = pick_one(problem_type, family, size_dir, name)
    if path is None:
        pytest.skip("pick not present in this checkout")
    return path


def test_neighbour_lists_symmetric_and_clean(pick) -> None:
    core = load_core(pick)
    n = core.num_customers
    lists = _core.ls_neighbour_lists(core, 10)
    assert len(lists) == n + 1
    assert lists[0] == []
    members = [set(l) for l in lists]
    for i in range(1, n + 1):
        assert i not in members[i], "self must not be a neighbour"
        assert 0 not in members[i], "depot must not be a neighbour"
        assert lists[i] == sorted(lists[i])
        for j in members[i]:
            assert i in members[j], "adjacency must be union-symmetric"
    # Requested size is a floor before union symmetrisation, not a cap after.
    sizes = [len(l) for l in lists[1:]]
    assert max(sizes) >= min(10, n - 1)


def test_neighbour_lists_deterministic_and_monotone(pick) -> None:
    core = load_core(pick)
    a = _core.ls_neighbour_lists(core, 10)
    b = _core.ls_neighbour_lists(core, 10)
    assert a == b
    small = _core.ls_neighbour_lists(core, 5)
    large = _core.ls_neighbour_lists(core, 15)
    for i in range(1, core.num_customers + 1):
        assert set(small[i]) <= set(large[i]), "top-k lists must nest in k"


def test_neighbour_lists_exhaustive_sentinel(pick) -> None:
    core = load_core(pick)
    lists = _core.ls_neighbour_lists(core, 0)
    assert all(l == [] for l in lists)


@pytest.mark.parametrize("k", [10, 50])
def test_granular_ls_invariants(pick, k) -> None:
    """Granular LS keeps the M3.7 contract: partition preserved, value is the
    canonical checker Duration, never worse, checker-valid end to end."""
    from mamut_routing_lib.models import BenchmarkSolution
    from mamut_routing_lib.td import check_td_solution

    loaded = load_instance(pick)
    core = to_core(loaded)
    routes = greedy_routes(core)
    before = _core.solution_duration(core, routes)
    assert before != float("inf")
    served_before = sorted(c for route in routes for c in route)

    new_routes, value, applied, reverted = _core.ls_local_search(
        core, routes, num_neighbours=k
    )
    assert value <= before
    assert value == _core.solution_duration(core, new_routes)
    assert sorted(c for route in new_routes for c in route) == served_before
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


def test_granular_full_lists_match_exhaustive_tdvrp() -> None:
    """With k >= n-1 every granular justification passes on a TDVRP instance
    (no TW screen excludes edges), so the descent must be bitwise-identical to
    the exhaustive enumeration — trajectory, routes and value."""
    require_benchmarks()
    for size_dir, name in [("n=25", "R101"), ("n=25", "C101")]:
        path = pick_one("TDVRP", "Dabia2013", size_dir, name)
        if path is None:
            pytest.skip("instance not present")
        core = load_core(path)
        routes = greedy_routes(core)
        ex_routes, ex_value, ex_applied, _ = _core.ls_local_search(
            core, routes, num_neighbours=0
        )
        gr_routes, gr_value, gr_applied, _ = _core.ls_local_search(
            core, routes, num_neighbours=core.num_customers
        )
        assert gr_routes == ex_routes
        assert gr_value == ex_value
        assert gr_applied == ex_applied


def test_granular_ls_deterministic(pick) -> None:
    core = load_core(pick)
    routes = greedy_routes(core)
    a_routes, a_value, *_ = _core.ls_local_search(core, routes, num_neighbours=10)
    b_routes, b_value, *_ = _core.ls_local_search(core, routes, num_neighbours=10)
    assert a_routes == b_routes
    assert a_value == b_value


def test_solve_granular_default_deterministic() -> None:
    """solve() defaults (granular k=50 since 0.4.0) stay seed-deterministic and
    checker-refereed (solve itself raises on any disagreement)."""
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", "n=25", "C101")
    if path is None:
        pytest.skip("instance not present")
    budget = kayros.Params(strategy="aco", max_iterations=30, max_no_improvement=30)
    assert budget.num_neighbours == 50
    a = kayros.solve(path, budget, seed=11)
    b = kayros.solve(path, budget, seed=11)
    assert a.routes == b.routes
    assert a.duration == b.duration


def test_solve_granular_quality_close_to_exhaustive() -> None:
    """Granular defaults must not collapse quality on a small instance: within
    5% of the exhaustive run under the same small budget (both are heuristics;
    this is a sanity band, not an equivalence claim)."""
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", "n=25", "R101")
    if path is None:
        pytest.skip("instance not present")
    budget = kayros.Params(strategy="aco", max_iterations=40, max_no_improvement=40)
    exhaustive = kayros.Params(
        strategy="aco", max_iterations=40, max_no_improvement=40, num_neighbours=0
    )
    gr = kayros.solve(path, budget, seed=3)
    ex = kayros.solve(path, exhaustive, seed=3)
    assert gr.duration <= ex.duration * 1.05
