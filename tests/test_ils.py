"""M7.2 TD-ILS gates (Stream 7, td-ils-design.md).

Discipline:
- the returned best is a complete feasible solution, checker-exact
  (partition, canonical Duration, check_td_solution agreement);
- the incumbent stream is strictly monotone — LAHC worse-accepts stay
  internal to the loop and never surface;
- seeded determinism under an iteration budget (no TL in the loop);
- ILS beats its own greedy+LS seed given a few hundred iterations;
- TL compliance: overshoot bounded (one operator pass at these sizes is
  milliseconds, assert a generous absolute bound).
"""

import time

import pytest

from kayros import _core
from kayros.io import load_instance, to_core

from conftest import family_instances, require_benchmarks


def pick_one(problem_type, family, size_dir, name=None):
    paths = family_instances(problem_type, family, [size_dir])
    if name is not None:
        paths = [p for p in paths if p.name.startswith(name)]
    return paths[0] if paths else None


def ils_params(**overrides):
    params = _core.IlsParams()
    for key, value in overrides.items():
        setattr(params, key, value)
    return params


PICKS = [
    ("TDVRPTW", "Dabia2013", "n=25", "R101"),
    ("TDVRPTW", "Dabia2013", "n=50", "C103"),
    ("TDVRP", "Dabia2013", "n=25", "R101"),
    ("TDVRPTW", "Rifki2020", "n=10", None),
]


@pytest.fixture(scope="module", params=range(len(PICKS)))
def pick(request):
    require_benchmarks()
    problem_type, family, size_dir, name = PICKS[request.param]
    path = pick_one(problem_type, family, size_dir, name)
    if path is None:
        pytest.skip("pick not present in this checkout")
    return path


def test_ils_solution_checker_valid(pick) -> None:
    from mamut_routing_lib.models import BenchmarkSolution
    from mamut_routing_lib.td import check_td_solution

    loaded = load_instance(pick, verify=True)
    core = to_core(loaded)
    result = _core.solve_ils(core, ils_params(max_iterations=200), 42, 0.0)
    assert result.status.name in ("Finished", "TimeLimit")
    assert result.value == _core.solution_duration(core, result.routes)
    n = core.num_customers
    assert sorted(c for route in result.routes for c in route) == list(
        range(1, n + 1)
    )
    assert all(result.routes)

    check = check_td_solution(
        loaded,
        BenchmarkSolution(
            instance_name=loaded.instance.instance_name,
            routes=[list(r) for r in result.routes],
            cost=result.value,
        ),
    )
    assert check.is_valid(), (check.status, check.error_message)
    assert check.routing_cost == result.value


def test_ils_incumbents_monotone(pick) -> None:
    core = to_core(load_instance(pick, verify=True))
    events = []
    result = _core.solve_ils(
        core,
        ils_params(max_iterations=300),
        7,
        0.0,
        lambda inc, routes: events.append((inc.value, [list(r) for r in routes])),
    )
    assert events, "the seed incumbent must fire"
    values = [v for v, _ in events]
    assert values == sorted(values, reverse=True)
    assert len(set(values)) == len(values), "incumbents must strictly improve"
    assert values[-1] == result.value
    assert events[-1][1] == [list(r) for r in result.routes]
    # Origins: the seed phase publishes 0 and can do so more than once — the raw
    # construction (1.4.0 publish-early), the K-diverse split when it improves
    # (1.5.0 seed_k_factor), then the first descent. Everything the search finds
    # afterwards is 2 (= ils), and the phases never interleave, so the origin
    # sequence is a non-empty prefix of 0s followed by all 2s.
    origins = [inc.origin for inc in result.incumbents]
    assert set(origins) <= {0, 2}
    assert origins[0] == 0
    first_ils = next((i for i, o in enumerate(origins) if o == 2), len(origins))
    assert all(o == 0 for o in origins[:first_ils])
    assert all(o == 2 for o in origins[first_ils:])


def test_ils_publishes_the_raw_seed_first(pick) -> None:
    """1.4.0 publish-early: the first incumbent is the greedy seed exactly as
    built, before any descent has touched it."""
    core = to_core(load_instance(pick, verify=True))
    ok, seed_routes = _core.greedy_makespan(core)
    assert ok
    seed_value = _core.solution_duration(core, seed_routes)

    events = []
    _core.solve_ils(
        core, ils_params(max_iterations=50), 7, 0.0,
        lambda inc, routes: events.append((inc.value, [list(r) for r in routes])),
    )
    assert events
    assert events[0][0] == seed_value
    assert events[0][1] == [list(r) for r in seed_routes]


def test_ils_deterministic(pick) -> None:
    core = to_core(load_instance(pick, verify=True))
    a = _core.solve_ils(core, ils_params(max_iterations=250), 11, 0.0)
    b = _core.solve_ils(core, ils_params(max_iterations=250), 11, 0.0)
    assert a.routes == b.routes
    assert a.value == b.value
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]


def test_ils_seeds_diverge() -> None:
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", "n=50", "C103")
    if path is None:
        pytest.skip("instance not present")
    core = to_core(load_instance(path, verify=True))
    finals = {
        _core.solve_ils(core, ils_params(max_iterations=100), s, 0.0).value
        for s in range(4)
    }
    assert len(finals) > 1


def test_ils_beats_greedy_seed() -> None:
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", "n=50", "R205")
    if path is None:
        pytest.skip("instance not present")
    core = to_core(load_instance(path, verify=True))
    ok, routes = _core.greedy_makespan(core)
    assert ok
    seed_after_ls, *_ = (
        _core.ls_local_search(core, routes, num_neighbours=50)[1],
    )
    result = _core.solve_ils(core, ils_params(max_iterations=500), 3, 0.0)
    assert result.value < seed_after_ls, "ILS must improve on its own seed"


def test_ils_time_limit_compliance() -> None:
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", "n=100", "R101")
    if path is None:
        pytest.skip("instance not present")
    core = to_core(load_instance(path, verify=True))
    t0 = time.perf_counter()
    result = _core.solve_ils(core, ils_params(), 1, 2.0)
    wall = time.perf_counter() - t0
    assert result.status.name == "TimeLimit"
    assert wall < 2.0 + 2.0, f"overshoot too large: {wall:.2f}s"
    assert result.value == _core.solution_duration(core, result.routes)


def test_ils_restart_path_runs() -> None:
    """A tiny restart threshold forces the restart-to-best branch; the run
    must stay valid and deterministic through it."""
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", "n=25", "C101")
    if path is None:
        pytest.skip("instance not present")
    core = to_core(load_instance(path, verify=True))
    params = ils_params(max_iterations=120, restart_no_improvement=10)
    a = _core.solve_ils(core, params, 5, 0.0)
    b = _core.solve_ils(core, params, 5, 0.0)
    assert a.routes == b.routes and a.value == b.value
    assert a.value == _core.solution_duration(core, a.routes)
