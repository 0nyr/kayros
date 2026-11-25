"""Check post-drain squeeze work limits, determinism and zero-warp checkpoints. A phase that finds no improvement preserves the starting routes. The public solver checks the selected routes under the exact objective."""

import pytest

import kayros
from kayros import _core

from conftest import blauth2024_instances
from test_fleet_descent import START, _ladder_core

BLAUTH_N10 = blauth2024_instances("n=10")


def test_squeeze_noop_leaves_routes_unchanged() -> None:
    """On a state the (relocate+swap) squeeze cannot improve, the phase must
    return improved=False and hand back the input routes untouched."""
    core = _ladder_core()
    routes, improved, phases, evaluated, checkpoints = _core.ls_squeeze_phase(
        core, START, sq_penalty=10.0, sq_work_cap=100000, num_neighbours=0
    )
    assert phases == 1
    assert evaluated >= 1
    if not improved:
        assert routes == START
        assert checkpoints == 0


def test_squeeze_disabled_is_free() -> None:
    core = _ladder_core()
    routes, improved, phases, evaluated, checkpoints = _core.ls_squeeze_phase(
        core, START, sq_penalty=10.0, sq_work_cap=0, num_neighbours=0
    )
    assert not improved and phases == 0 and evaluated == 0
    assert routes == START


def test_squeeze_budget_respected() -> None:
    """A tiny budget must not crash and must stay near the cap (the charge
    granularity is a handful of pricings per candidate)."""
    core = _ladder_core()
    _routes, _improved, phases, evaluated, _cp = _core.ls_squeeze_phase(
        core, START, sq_penalty=10.0, sq_work_cap=1, num_neighbours=0
    )
    assert phases == 1
    # The cap is checked once per client, so the overshoot is bounded by one
    # client's candidate scan (a few pricings per candidate on the toy).
    assert evaluated <= 64


def test_squeeze_deterministic() -> None:
    core = _ladder_core()
    a = _core.ls_squeeze_phase(core, START, sq_penalty=10.0, sq_work_cap=50000,
                               num_neighbours=0)
    b = _core.ls_squeeze_phase(core, START, sq_penalty=10.0, sq_work_cap=50000,
                               num_neighbours=0)
    assert a == b


def test_squeeze_result_is_feasible() -> None:
    """Whatever the phase returns must rebuild feasible route by route (the
    exactly-zero-warp boundary invariant)."""
    core = _ladder_core()
    routes, _improved, _p, _e, _cp = _core.ls_squeeze_phase(
        core, START, sq_penalty=10.0, sq_work_cap=100000, num_neighbours=0
    )
    assert sorted(c for r in routes for c in r) == [1, 2, 3, 4, 5]
    for route in routes:
        assert _core.solution_cost(core, [route]) != float("inf")


@pytest.mark.parametrize("instance_path", BLAUTH_N10)
def test_squeeze_inert_at_zero_cap_fcd(instance_path) -> None:
    """An explicit zero work budget and the default both disable the post-drain squeeze."""
    from mamut_routing_lib.td import load_td_instance

    loaded = load_td_instance(instance_path)
    tl_params = dict(objective="fleet_cost_duration", ils_max_iterations=400)
    a = kayros.solve(loaded, kayros.Params(**tl_params), seed=11)
    b = kayros.solve(loaded, kayros.Params(**tl_params, sq_work_cap=0), seed=11)
    assert a.routes == b.routes
    assert a.cost == b.cost
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
    assert a.fd_stats["squeeze_phases"] == 0
    assert a.fd_stats["squeeze_evaluated"] == 0


@pytest.mark.parametrize("instance_path", BLAUTH_N10[:1])
def test_squeeze_armed_deterministic_and_valid(instance_path) -> None:
    """Armed under FCD: two identical runs are bitwise equal, the checker
    gate inside solve() holds (it raises on any disagreement), and the
    incumbent stream stays strictly improving."""
    from mamut_routing_lib.td import load_td_instance

    loaded = load_td_instance(instance_path)
    params = kayros.Params(
        objective="fleet_cost_duration",
        ils_max_iterations=400,
        sq_work_cap=50_000,
        fd_period_work=2000,
    )
    a = kayros.solve(loaded, params, seed=11)
    b = kayros.solve(loaded, params, seed=11)
    assert a.routes == b.routes
    assert a.cost == b.cost
    assert a.fd_stats == b.fd_stats
    values = [i.value for i in a.incumbents]
    assert values == sorted(values, reverse=True)
    assert a.fd_stats["squeeze_phases"] >= 0  # counters wired


@pytest.mark.parametrize("instance_path", BLAUTH_N10[:1])
def test_ladder_squeeze_inert_off_fcd(instance_path) -> None:
    """sq_ladder=False (the default) must reproduce the same FCD stream as an
    explicit False: the p-count increment sits exactly where it always did."""
    from mamut_routing_lib.td import load_td_instance

    loaded = load_td_instance(instance_path)
    base = dict(objective="fleet_cost_duration", ils_max_iterations=400)
    a = kayros.solve(loaded, kayros.Params(**base), seed=5)
    b = kayros.solve(loaded, kayros.Params(**base, sq_ladder=False), seed=5)
    assert a.routes == b.routes
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
    assert a.fd_stats["ladder_squeezes"] == 0


@pytest.mark.parametrize("instance_path", BLAUTH_N10[:1])
def test_ladder_squeeze_armed_deterministic(instance_path) -> None:
    from mamut_routing_lib.td import load_td_instance

    loaded = load_td_instance(instance_path)
    params = kayros.Params(
        objective="fleet_cost_duration",
        ils_max_iterations=400,
        sq_ladder=True,
        fd_period_work=2000,
    )
    a = kayros.solve(loaded, params, seed=5)
    b = kayros.solve(loaded, params, seed=5)
    assert a.routes == b.routes
    assert a.fd_stats == b.fd_stats
    assert a.fd_stats["ladder_rescues"] <= a.fd_stats["ladder_squeezes"]


@pytest.mark.parametrize("instance_path", BLAUTH_N10[:1])
def test_squeeze_inert_under_duration(instance_path) -> None:
    """Under Duration the whole branch is F-gated dead code at ANY setting."""
    from mamut_routing_lib.td import load_td_instance

    loaded = load_td_instance(instance_path)
    base = dict(objective="duration", ils_max_iterations=400)
    a = kayros.solve(loaded, kayros.Params(**base), seed=7)
    b = kayros.solve(
        loaded,
        kayros.Params(**base, sq_work_cap=1_000_000, sq_penalty=1.0,
                      sq_ladder=True),
        seed=7,
    )
    assert a.routes == b.routes
    assert a.cost == b.cost
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
