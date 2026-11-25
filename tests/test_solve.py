"""Check feasibility, seeded determinism and solution quality against stored incumbents."""

import pytest
from mamut_routing_lib.bks import get_bks_path_for_instance, load_bks
from mamut_routing_lib.enums import ObjectiveFunction

import kayros

from conftest import family_instances

SMALL_BUDGET = kayros.Params(ils_max_iterations=80)


def pick(paths, names):
    wanted = [p for p in paths if p.name.removesuffix(".vrp.json") in names]
    return wanted


DABIA_25_TDVRPTW = pick(
    family_instances("TDVRPTW", "Dabia2013", ["n=25"]), {"C101", "R101", "RC101"}
)
DABIA_25_TDVRP = pick(family_instances("TDVRP", "Dabia2013", ["n=25"]), {"C101", "R101"})


@pytest.mark.parametrize(
    "instance_path", DABIA_25_TDVRPTW, ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_tdvrptw(instance_path) -> None:
    solution = kayros.solve(instance_path, SMALL_BUDGET, seed=1)
    assert solution.cost > 0.0
    assert solution.incumbents, "at least the greedy seed must be recorded"
    # every customer served exactly once
    served = sorted(c for route in solution.routes for c in route)
    n = max(served)
    assert served == list(range(1, n + 1))


@pytest.mark.parametrize(
    "instance_path", DABIA_25_TDVRP, ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_tdvrp(instance_path) -> None:
    solution = kayros.solve(instance_path, SMALL_BUDGET, seed=1)
    assert solution.cost > 0.0


@pytest.mark.parametrize(
    "instance_path", DABIA_25_TDVRPTW[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_deterministic(instance_path) -> None:
    a = kayros.solve(instance_path, SMALL_BUDGET, seed=42)
    b = kayros.solve(instance_path, SMALL_BUDGET, seed=42)
    assert a.routes == b.routes
    assert a.cost == b.cost
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]


@pytest.mark.parametrize(
    "instance_path", DABIA_25_TDVRPTW[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_on_incumbent_hook(instance_path) -> None:
    """Each synchronous callback receives the complete routes for the corresponding incumbent record."""
    events: list[tuple[kayros.Incumbent, list[list[int]]]] = []
    solution = kayros.solve(
        instance_path, SMALL_BUDGET, seed=1,
        on_incumbent=lambda inc, routes: events.append((inc, routes)),
    )
    assert len(events) == len(solution.incumbents)
    assert [e[0].value for e in events] == [i.value for i in solution.incumbents]
    assert events[0][0].origin == "greedy"
    values = [e[0].value for e in events]
    assert values == sorted(values, reverse=True), "incumbents must improve"
    n = max(c for route in solution.routes for c in route)
    for _, routes in events:
        served = sorted(c for route in routes for c in route)
        assert served == list(range(1, n + 1))
    assert events[-1][1] == solution.routes


@pytest.mark.parametrize(
    "instance_path", DABIA_25_TDVRPTW[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_on_incumbent_exception_propagates(instance_path) -> None:
    class Boom(RuntimeError):
        pass

    def hook(inc, routes):
        raise Boom("stop")

    with pytest.raises(Boom):
        kayros.solve(instance_path, SMALL_BUDGET, seed=1, on_incumbent=hook)


@pytest.mark.parametrize(
    "instance_path", DABIA_25_TDVRPTW, ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_quality_band_vs_bks(instance_path) -> None:
    """Loose sanity: the heuristic should land within 2x of the exact BKS."""
    bks_path = get_bks_path_for_instance(instance_path, ObjectiveFunction.DURATION)
    if not bks_path.exists():
        pytest.skip("no stored BKS")
    bks = load_bks(bks_path)
    solution = kayros.solve(instance_path, SMALL_BUDGET, seed=1)
    assert solution.cost <= 2.0 * bks.cost, (solution.cost, bks.cost)


RIFKI_10 = family_instances("TDVRPTW", "Rifki2020", ["n=10"])[:1]


@pytest.mark.parametrize(
    "instance_path",
    DABIA_25_TDVRPTW[:2] + DABIA_25_TDVRP[:1] + RIFKI_10,
    ids=lambda p: f"{p.parent.parent.name}-{p.name.removesuffix('.vrp.json')}",
)
def test_solution_cost_contract(instance_path) -> None:
    """Library 0.13 contract of Solution: ``exact_cost`` is the library's exact
    Duration cost of the routes, ``cost`` its binary64 display (bitwise), the
    per-route fields are the exact route durations in the order of ``routes``,
    and the solver's internal float value ``search_value`` is within TOL of it
    (no bit-level contract: it can differ from ``cost`` in the last bits)."""
    from mamut_routing_lib.td import check_td_solution

    from exact_reference import assert_value_close, exact_duration_cost, exact_route
    from kayros.io import load_instance

    solution = kayros.solve(instance_path, SMALL_BUDGET, seed=1)
    loaded = load_instance(instance_path, verify=True)
    want = exact_duration_cost(loaded.instance, loaded.atfs, solution.routes)
    assert solution.objective == "Duration"
    assert solution.exact_cost == want
    assert solution.cost == float(want)
    assert solution.search_value is not None
    assert_value_close(solution.search_value, want, instance_path, family="search_value")

    assert len(solution.exact_route_durations) == len(solution.routes)
    for route, duration, departure, float_duration, float_departure in zip(
        solution.routes,
        solution.exact_route_durations,
        solution.exact_route_departures,
        solution.route_durations,
        solution.route_departures,
    ):
        evaluation = exact_route(loaded, route)
        assert evaluation is not None
        assert duration == evaluation.duration
        assert departure == evaluation.departure_time
        assert float_duration == float(duration)
        assert float_departure == float(departure)
    assert sum(solution.exact_route_durations) == solution.exact_cost

    artifact = solution.to_benchmark_solution()
    check = check_td_solution(loaded, artifact, ObjectiveFunction.DURATION)
    assert check.is_valid(), (check.status, check.error_message)
    assert check.routing_cost == solution.cost
