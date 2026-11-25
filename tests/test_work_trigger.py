"""Check deterministic work-based fleet-descent and restart triggers. Defaults are 100 million candidate pricings between fleet-descent triggers and 1 billion without improvement before restart. Zero disables the respective work trigger. Fleet descent also requires a positive route charge."""

import pytest
from mamut_routing_lib.td import load_td_instance

import kayros

from conftest import family_instances
from test_fleet_cost_duration import BLAUTH_F, with_fleet_cost

DABIA_25 = family_instances("TDVRPTW", "Dabia2013", ["n=25"])

parametrized = pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)


@parametrized
def test_work_units_reported_and_deterministic(instance_path) -> None:
    """A fixed seed reproduces total work. Disabling the work trigger leaves the 20,000-iteration restart threshold beyond this run."""
    loaded = load_td_instance(instance_path)
    params = kayros.Params(ils_max_iterations=120, restart_no_improvement_work=0)
    a = kayros.solve(loaded, params, seed=3)
    b = kayros.solve(loaded, params, seed=3)
    assert a.work_units > 0
    assert a.work_units == b.work_units
    assert a.restarts == 0 and b.restarts == 0


@parametrized
def test_work_restart_fires_and_is_deterministic(instance_path) -> None:
    """With the flat count out of reach, a small work threshold drives
    restart-to-best on its own, visibly (restarts > 0) and reproducibly."""
    loaded = load_td_instance(instance_path)
    params = kayros.Params(ils_max_iterations=200, restart_no_improvement_work=20_000)
    a = kayros.solve(loaded, params, seed=7)
    b = kayros.solve(loaded, params, seed=7)
    assert a.restarts > 0
    assert a.restarts == b.restarts
    assert a.routes == b.routes
    assert a.cost == b.cost


@parametrized
def test_fd_work_trigger_fires_on_priced_solve(instance_path) -> None:
    """Under FleetCostDuration with the stagnation and period triggers both
    dead (flat restart unreachable, work restart explicitly off),
    fd_period_work alone must produce FD triggers; without it the baseline
    stays at zero."""
    loaded = with_fleet_cost(load_td_instance(instance_path), BLAUTH_F)
    base = dict(objective="fleet_cost_duration", ils_max_iterations=300,
                restart_no_improvement_work=0)
    off = kayros.solve(loaded, kayros.Params(**base, fd_period_work=0), seed=11)
    on = kayros.solve(
        loaded, kayros.Params(**base, fd_period_work=20_000), seed=11
    )
    assert off.fd_stats["triggers"] == 0
    assert on.fd_stats["triggers"] > 0
    again = kayros.solve(
        loaded, kayros.Params(**base, fd_period_work=20_000), seed=11
    )
    assert again.fd_stats == on.fd_stats
    assert again.routes == on.routes


@parametrized
def test_work_restart_default_active(instance_path) -> None:
    """The default restarts after 1 billion candidate evaluations without improvement."""
    assert kayros.Params().restart_no_improvement_work == 1_000_000_000
    loaded = load_td_instance(instance_path)
    on = kayros.solve(loaded, kayros.Params(ils_max_iterations=1500), seed=9)
    pinned = kayros.solve(
        loaded,
        kayros.Params(ils_max_iterations=1500,
                      restart_no_improvement_work=1_000_000_000),
        seed=9,
    )
    off = kayros.solve(
        loaded,
        kayros.Params(ils_max_iterations=1500, restart_no_improvement_work=0),
        seed=9,
    )
    assert on.routes == pinned.routes and on.cost == pinned.cost
    assert on.work_units == pinned.work_units
    assert on.restarts == 0 and off.restarts == 0
    assert on.routes == off.routes and on.work_units == off.work_units


@parametrized
def test_fd_work_trigger_inert_under_duration(instance_path) -> None:
    """fd_period_work sits behind the F-gate like every FD knob: under
    Duration the counter accumulates but the trigger never arms, so the whole
    solve is bitwise the knob-off run."""
    loaded = load_td_instance(instance_path)
    a = kayros.solve(loaded, kayros.Params(ils_max_iterations=120), seed=5)
    b = kayros.solve(
        loaded, kayros.Params(ils_max_iterations=120, fd_period_work=5_000), seed=5
    )
    assert a.routes == b.routes
    assert a.cost == b.cost
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
    assert a.iterations == b.iterations
    assert a.work_units == b.work_units
