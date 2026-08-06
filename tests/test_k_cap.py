"""Plan-15 S3: the K-cap confinement (``k_cap``, design memo section 10).

Contract under test: inert at 0 under both objectives and F-gated dead code
under Duration at any value; a seed or warm start above the cap drains toward
it before the loop; incumbents publish only at K <= k_cap; a run that never
reaches the cap returns Infeasible with an EMPTY incumbent stream (D-S3.1);
armed runs are deterministic.
"""

import pytest

import kayros
from kayros import _core

from conftest import blauth2024_instances
from test_fleet_descent import START

BLAUTH_N10 = blauth2024_instances("n=10")
TOY_F = 1000.0


def _priced_ladder_core():
    """The test_fleet_descent ladder toy, with routes PRICED so the fleet
    machinery (and the cap) arms. K=1 is structurally impossible: clients 1
    and 4 both carry point windows [10, 10] (each must be served first in a
    route) and total demand 6 exceeds capacity 3, so 2 is the true floor."""
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
        fixed_route_cost=TOY_F,
    )


def _solve_capped(core, k_cap, seed=3, iterations=600):
    params = kayros.Params(
        ils_max_iterations=iterations, k_cap=k_cap, fd_period=25
    )._to_ils_core()
    published_k = []

    def hook(_inc, routes):
        published_k.append(len(routes))

    result = _core.solve_ils(core, params, seed, 10.0, hook, list(START))
    return result, published_k


@pytest.mark.parametrize("instance_path", BLAUTH_N10[:1])
def test_k_cap_inert_at_zero_fcd(instance_path) -> None:
    """k_cap=0 (the default) must reproduce the uncapped FleetCostDuration
    stream bitwise, and the stats slot stays None."""
    from mamut_routing_lib.td import load_td_instance

    loaded = load_td_instance(instance_path)
    base = dict(objective="fleet_cost_duration", ils_max_iterations=400)
    a = kayros.solve(loaded, kayros.Params(**base), seed=11)
    b = kayros.solve(loaded, kayros.Params(**base, k_cap=0), seed=11)
    assert a.routes == b.routes
    assert a.duration == b.duration
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
    assert a.k_cap_stats is None and b.k_cap_stats is None


@pytest.mark.parametrize("instance_path", BLAUTH_N10[:1])
def test_k_cap_inert_under_duration(instance_path) -> None:
    """Under Duration the cap is F-gated dead code at any value."""
    from mamut_routing_lib.td import load_td_instance

    loaded = load_td_instance(instance_path)
    base = dict(objective="duration", ils_max_iterations=400)
    a = kayros.solve(loaded, kayros.Params(**base), seed=7)
    b = kayros.solve(loaded, kayros.Params(**base, k_cap=2), seed=7)
    assert a.routes == b.routes
    assert a.duration == b.duration
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
    assert b.k_cap_stats is None


def test_k_cap_reachable_drains_and_publishes_only_capped() -> None:
    """Warm start at K=3, cap 2 (the true floor): the seed drain reaches the
    cap, every published incumbent respects it, and the run is deterministic."""
    core = _priced_ladder_core()
    result, published_k = _solve_capped(core, k_cap=2)
    assert result.status != _core.SolveStatus.Infeasible
    assert 0 < len(result.routes) <= 2
    assert published_k and all(k <= 2 for k in published_k)
    assert result.k_cap_stats.reached_cap == 1
    assert result.k_cap_stats.seed_drain_attempts >= 1
    assert result.k_cap_stats.first_capped_work >= 0
    again, again_k = _solve_capped(core, k_cap=2)
    assert [i.value for i in result.incumbents] == [
        i.value for i in again.incumbents
    ]
    assert result.routes == again.routes and published_k == again_k


def test_k_cap_unreachable_returns_nothing() -> None:
    """Cap 1 is structurally impossible on the toy: the run must publish
    NOTHING and come back Infeasible with empty routes (D-S3.1), not hand
    back an above-cap solution."""
    core = _priced_ladder_core()
    result, published_k = _solve_capped(core, k_cap=1)
    assert result.status == _core.SolveStatus.Infeasible
    assert result.routes == [] and result.incumbents == [] and not published_k
    assert result.k_cap_stats.reached_cap == 0
    assert result.k_cap_stats.seed_drain_attempts >= 1
    assert result.k_cap_stats.first_capped_work == -1


@pytest.mark.parametrize("instance_path", BLAUTH_N10[:1])
def test_k_cap_solve_level_reachable(instance_path) -> None:
    """solve() with the cap set at the uncapped final K: reached, stats
    exposed as a dict, deterministic."""
    from mamut_routing_lib.td import load_td_instance

    loaded = load_td_instance(instance_path)
    base = dict(objective="fleet_cost_duration", ils_max_iterations=400)
    free = kayros.solve(loaded, kayros.Params(**base), seed=11)
    capped_params = kayros.Params(**base, k_cap=free.num_routes)
    a = kayros.solve(loaded, capped_params, seed=11)
    b = kayros.solve(loaded, capped_params, seed=11)
    assert a.num_routes <= free.num_routes
    assert a.k_cap_stats is not None and a.k_cap_stats["reached_cap"] == 1
    assert a.routes == b.routes and a.k_cap_stats == b.k_cap_stats
