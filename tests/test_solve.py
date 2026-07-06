"""M3.3: solve() behavior on real instances — validity (checker-refereed by
construction), seeded determinism, and loose quality sanity vs stored BKS."""

import pytest
from mamut_routing_lib.bks import get_bks_path_for_instance, load_bks
from mamut_routing_lib.enums import ObjectiveFunction

import kayros

from conftest import family_instances

SMALL_BUDGET = kayros.Params(max_iterations=80, max_no_improvement=40)


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
    assert solution.duration > 0.0
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
    assert solution.duration > 0.0


@pytest.mark.parametrize(
    "instance_path", DABIA_25_TDVRPTW[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_deterministic(instance_path) -> None:
    a = kayros.solve(instance_path, SMALL_BUDGET, seed=42)
    b = kayros.solve(instance_path, SMALL_BUDGET, seed=42)
    assert a.routes == b.routes
    assert a.duration == b.duration
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]


@pytest.mark.parametrize(
    "instance_path", DABIA_25_TDVRPTW[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_on_incumbent_hook(instance_path) -> None:
    """M3.6 anytime hook: fires live on every incumbent, in step with
    Solution.incumbents, and each event carries a complete routes snapshot."""
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
    assert solution.duration <= 2.0 * bks.cost, (solution.duration, bks.cost)
