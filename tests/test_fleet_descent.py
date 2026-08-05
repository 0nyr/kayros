"""Plan-12 M4: the fleet-descent phase (NBRMH-lite ejection ladder).

Synthetic isolation of the two ladder rungs (feasible best-insert, then
contiguous-window insertion-with-ejection), the all-or-nothing rollback, and
the solve()-level contract: inert under Duration (the F-gate sits before any
draw), armed and deterministic under FleetCostDuration.
"""

import pytest
from mamut_routing_lib.td import load_td_instance

import kayros
from kayros import _core

from conftest import family_instances
from test_fleet_cost_duration import BLAUTH_F, with_fleet_cost

DABIA_25 = family_instances("TDVRPTW", "Dabia2013", ["n=25"])


def _ladder_core():
    """Five customers, constant travel 10, engineered so dissolving the
    singleton [4] exercises BOTH rungs: clients 1 and 4 have point windows
    [10, 10] (they can only be served first in a route), 2/3/5 share a late
    window; demands (1,1,1,2,1) against capacity 3 make every step-1 insert of
    client 4 capacity-infeasible, so only ejection windows can place it."""
    h = 100000.0
    n = 5
    arcs = [
        (i, j, [0.0, h], [10.0, h + 10.0])
        for i in range(n + 1)
        for j in range(n + 1)
        if i != j
    ]
    return _core.Instance(
        num_customers=n,
        num_vehicles=None,
        vehicle_capacity=3,
        horizon=(0.0, h),
        time_windows=[
            (0.0, h),
            (10.0, 10.0),
            (500.0, 600.0),
            (500.0, 600.0),
            (10.0, 10.0),
            (500.0, 600.0),
        ],
        demands=[0, 1, 1, 1, 2, 1],
        service_times=[0.0] * (n + 1),
        arcs=arcs,
    )


START = [[1, 2], [3, 5], [4]]


def test_fleet_descent_drops_a_route() -> None:
    core = _ladder_core()
    assert _core.solution_duration(core, START) != float("inf")
    routes, value, success, pops, step1, ejections, dead, budget, work_rb, evaluated = (
        _core.ls_fleet_descent(core, START, 0, route_choice=1, num_neighbours=0)
    )
    assert success
    assert len(routes) == 2
    assert sorted(c for r in routes for c in r) == [1, 2, 3, 4, 5]
    assert value == _core.solution_duration(core, routes)
    assert step1 >= 1 and ejections >= 1  # both ladder rungs fired
    assert dead == 0 and budget == 0 and work_rb == 0
    assert pops >= 2
    assert evaluated >= pops  # every pop prices at least one candidate set


def test_fleet_descent_all_or_nothing_on_impossible() -> None:
    """Total demand 4 exceeds one route's capacity 3, so K-1 is impossible:
    the attempt must end in a rollback and restore the input exactly."""
    core = _ladder_core()
    start = [[1, 2], [4]]
    routes, value, success, _pops, _s1, _ej, dead, budget, _wrb, _ev = (
        _core.ls_fleet_descent(
            core, start, 0, route_choice=1, ep_budget=25, num_neighbours=0
        )
    )
    assert not success
    assert routes == start  # exact restore, route order preserved
    assert value == _core.solution_duration(core, start)
    assert dead + budget == 1  # exactly one rollback ended the attempt


def test_fleet_descent_deterministic() -> None:
    core = _ladder_core()
    a = _core.ls_fleet_descent(core, START, 7, num_neighbours=0)
    b = _core.ls_fleet_descent(core, START, 7, num_neighbours=0)
    assert a == b


def test_fleet_descent_work_cap_rolls_back() -> None:
    """A tiny work cap must end the attempt in an all-or-nothing rollback
    (plan 15 M0.2: the drain is capped by work, never by wall clock)."""
    core = _ladder_core()
    routes, value, success, _pops, _s1, _ej, dead, budget, work_rb, evaluated = (
        _core.ls_fleet_descent(
            core, START, 0, route_choice=1, num_neighbours=0, work_cap=1
        )
    )
    assert not success
    assert routes == START  # exact restore, route order preserved
    assert value == _core.solution_duration(core, START)
    assert work_rb == 1 and dead == 0 and budget == 0
    assert evaluated >= 1  # the cap is checked between pops, after charging


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_fd_inert_under_duration(instance_path) -> None:
    """fd_attempts > 0 at F == 0 is dead code: same routes, value and
    incumbent stream as fd_attempts == 0 (the gate sits before any draw),
    even across restart-to-best triggers."""
    loaded = load_td_instance(instance_path)
    base = dict(ils_max_iterations=100, restart_no_improvement=10)
    a = kayros.solve(loaded, kayros.Params(fd_attempts=0, **base), seed=5)
    b = kayros.solve(loaded, kayros.Params(fd_attempts=3, **base), seed=5)
    assert a.routes == b.routes
    assert a.duration == b.duration
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
    assert b.fd_stats is None


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_fd_triggers_on_priced_solve(instance_path) -> None:
    loaded = with_fleet_cost(load_td_instance(instance_path), BLAUTH_F)
    params = kayros.Params(
        objective="fleet_cost_duration",
        ils_max_iterations=200,
        restart_no_improvement=10,
    )
    s = kayros.solve(loaded, params, seed=3)
    fd = s.fd_stats
    assert fd is not None
    assert fd["triggers"] > 0
    assert fd["attempts"] >= 1
    assert fd["successes"] <= fd["attempts"]
    assert all(v >= 0 for v in fd.values())
    again = kayros.solve(loaded, params, seed=3)
    assert again.fd_stats == fd
    assert again.routes == s.routes
