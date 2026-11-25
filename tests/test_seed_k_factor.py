"""Check route splitting in a Duration seed. Splits preserve coverage and feasibility, change the seed count reported by k_stats and consume no draws when disabled. FleetCostDuration starts from the unsplit seed."""

import pytest

from kayros import _core
from kayros.io import load_instance, to_core

from conftest import family_instances, require_benchmarks


def pick(problem_type, family, size_dir, name=None):
    paths = family_instances(problem_type, family, [size_dir])
    if name is not None:
        paths = [p for p in paths if p.name.startswith(name)]
    return paths[0] if paths else None


def a_duration_instance():
    for args in (("TDVRPTW", "Dabia2013", "n=50"), ("TDVRPTW", "Vu2020", "n=59")):
        p = pick(*args)
        if p is not None:
            return p
    return None


def test_split_preserves_the_solution() -> None:
    require_benchmarks()
    path = a_duration_instance()
    if path is None:
        pytest.skip("no Duration instance available")
    core = to_core(load_instance(path, verify=True))
    ok, seed = _core.greedy_construct(core)
    assert ok
    seed = [list(r) for r in seed]
    target = 2 * len(seed)

    split_ok, routes = _core.split_to_k(core, seed, target)
    assert split_ok
    routes = [list(r) for r in routes]

    assert sorted(c for r in routes for c in r) == sorted(c for r in seed for c in r)
    assert all(routes), "a split must never leave an empty route"
    assert len(routes) > len(seed)
    assert len(routes) <= target
    assert _core.solution_cost(core, routes) != float("inf")


def test_split_is_a_no_op_below_the_current_count() -> None:
    require_benchmarks()
    path = a_duration_instance()
    if path is None:
        pytest.skip("no Duration instance available")
    core = to_core(load_instance(path, verify=True))
    ok, seed = _core.greedy_construct(core)
    assert ok
    seed = [list(r) for r in seed]
    split_ok, routes = _core.split_to_k(core, seed, len(seed))
    assert not split_ok
    assert [list(r) for r in routes] == seed


def test_default_is_the_calibrated_multiplier() -> None:
    """Duration splits toward twice the greedy route count by default."""
    assert _core.IlsParams().seed_k_factor == 2.0

    from kayros import Params

    assert Params().seed_k_factor == 2.0


def test_disabling_recovers_the_pre_1_5_seed_bitwise() -> None:
    require_benchmarks()
    path = a_duration_instance()
    if path is None:
        pytest.skip("no Duration instance available")
    core = to_core(load_instance(path, verify=True))
    params = _core.IlsParams()
    params.ils_max_iterations = 120
    params.seed_k_factor = 1.0

    a = _core.solve_ils(core, params, 13, 0.0)
    b = _core.solve_ils(core, params, 13, 0.0)
    assert a.value == b.value
    assert [list(r) for r in a.routes] == [list(r) for r in b.routes]
    assert a.work_units == b.work_units

    ok, seed = _core.greedy_construct(core)
    assert ok
    assert a.k_stats.k_seed == len(seed), "disabled must start from the raw seed"


def test_seeding_starts_the_search_from_the_split_solution() -> None:
    require_benchmarks()
    path = a_duration_instance()
    if path is None:
        pytest.skip("no Duration instance available")
    core = to_core(load_instance(path, verify=True))
    params = _core.IlsParams()
    params.ils_max_iterations = 120

    params.seed_k_factor = 1.0
    plain = _core.solve_ils(core, params, 13, 0.0)
    params.seed_k_factor = 2.0
    seeded = _core.solve_ils(core, params, 13, 0.0)

    assert seeded.k_stats.k_seed > plain.k_stats.k_seed
    assert seeded.k_stats.k_seed <= 2 * plain.k_stats.k_seed
    assert seeded.value != float("inf")


def test_seeding_is_inert_under_fleet_cost_duration() -> None:
    """Under FleetCostDuration every extra route is priced, so the knob must be
    ignored whatever its value: splitting there is the wrong direction."""
    require_benchmarks()
    paths = family_instances("TDVRPTW", "Blauth2024", ["n=10"])
    if not paths:
        pytest.skip("Blauth2024 (the fleet-priced family) not present")
    core = to_core(load_instance(paths[0], verify=True), objective_function="FleetCostDuration")
    assert core.fixed_route_cost > 0.0

    params = _core.IlsParams()
    params.ils_max_iterations = 120
    params.seed_k_factor = 1.0
    off = _core.solve_ils(core, params, 13, 0.0)
    params.seed_k_factor = 3.0
    on = _core.solve_ils(core, params, 13, 0.0)

    assert off.value == on.value
    assert [list(r) for r in off.routes] == [list(r) for r in on.routes]
    assert off.k_stats.k_seed == on.k_stats.k_seed
