"""Session-44 work-based triggers.

LS work units (LsStats.evaluated: candidate pricings entered) are the
deterministic wall-time proxy behind ``fd_period_work`` and
``restart_no_improvement_work``: flat iteration counts fit one size class
only (Blauth2024 iteration velocity spans four orders of magnitude), while
the work rate per wall second is near-constant at a fixed instance. These
tests pin the trigger contracts: both knobs default off, fire
deterministically when set, and fd_period_work stays behind the F-gate so
Duration streams never move.
"""

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
    """work_units is populated on every ILS solve and is the same for the
    same seed; with the work restart explicitly off, the flat threshold
    (20000) stays unreachable in a 120-iteration run (restarts == 0, the
    1.1.x behavior)."""
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
    assert a.duration == b.duration


@parametrized
def test_fd_work_trigger_fires_on_priced_solve(instance_path) -> None:
    """Under FleetCostDuration with the stagnation and period triggers both
    dead (flat restart unreachable, work restart explicitly off, fd_period 0),
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
    """1.3.0 BREAKING default: restart_no_improvement_work = 1G is ON.
    The 1G window is a deliberate ~25-minute stall on one modern core (the
    250M variant measurably hurt sub-half-hour budgets in the M7 daytime
    A/B), so a short suite run must NOT fire it: defaults are bitwise the
    explicit-1G run and short runs stay restart-free at any size. The
    at-scale firing behavior is validated on Grid'5000, not here."""
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
    assert on.routes == pinned.routes and on.duration == pinned.duration
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
    assert a.duration == b.duration
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
    assert a.iterations == b.iterations
    assert a.work_units == b.work_units
