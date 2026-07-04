"""M3.2 hard gate: core route pricing must equal the canonical checker exactly.

For random routes over real MAMUT instances (feasible or not), the compiled
core's ready-time function, duration, and departure must be ``==``-identical
to ``mamut_routing_lib.td`` pricing. Any divergence is a kayros bug — never an
epsilon to add.
"""

import random
import zlib

import pytest
from mamut_routing_lib.td import (
    check_td_solution,
    compute_route_duration,
    compute_route_ready_time_function,
    load_td_instance,
)
from mamut_routing_lib.models import BenchmarkSolution

from kayros.io import to_core

from conftest import family_instances

ROUTES_PER_INSTANCE = 30

DABIA_TDVRPTW = family_instances("TDVRPTW", "Dabia2013", ["n=25", "n=50", "n=100"])
DABIA_TDVRP = family_instances("TDVRP", "Dabia2013", ["n=25", "n=50", "n=100"])


def random_routes(rng: random.Random, instance) -> list[list[int]]:
    """Random partition of all customers into routes.

    Half the draws shuffle uniformly (mostly TW-infeasible on TDVRPTW — they
    exercise empty-function agreement); half order customers by TW earliest
    and slice consecutive segments, which is strongly feasibility-biased on
    Solomon-style instances and exercises the priced path.
    """
    num_customers = instance.num_customers
    customers = list(range(1, num_customers + 1))
    time_windows = getattr(instance, "time_windows", None)
    if time_windows is not None and rng.random() < 0.5:
        customers.sort(key=lambda c: (time_windows[c][0], rng.random()))
    else:
        rng.shuffle(customers)
    routes: list[list[int]] = []
    index = 0
    while index < len(customers):
        size = min(rng.randint(1, max(2, num_customers // 6)), len(customers) - index)
        routes.append(customers[index : index + size])
        index += size
    if time_windows is not None and len(routes) > 1 and rng.random() < 0.5:
        # Interleave the TW-sorted slices across routes instead: each route
        # takes every k-th customer, a classic feasible spread pattern.
        k = len(routes)
        routes = [customers[offset::k] for offset in range(k)]
    return routes


def assert_instance_equivalence(instance_path) -> None:
    loaded = load_td_instance(instance_path)
    core = to_core(loaded)
    rng = random.Random(zlib.crc32(str(instance_path).encode()))

    feasible_routes = 0
    for _ in range(ROUTES_PER_INSTANCE):
        routes = random_routes(rng, loaded.instance)
        all_feasible = True
        for route in routes:
            reference = compute_route_duration(loaded.instance, loaded.atfs, route)
            feasible, duration, departure = core.evaluate_route(route)
            assert feasible == reference.feasible, (instance_path, route)
            if reference.feasible:
                feasible_routes += 1
                assert duration == reference.duration, (instance_path, route)
                assert departure == reference.departure_time, (instance_path, route)
            else:
                all_feasible = False
            delta = compute_route_ready_time_function(loaded.instance, loaded.atfs, route)
            core_xs, core_ys = core.route_ready_time_function(route)
            assert core_xs == delta.xs, (instance_path, route)
            assert core_ys == delta.ys, (instance_path, route)

        if all_feasible:
            # Full-solution agreement: checker total == canonical-order sum of
            # core durations (exact float addition order).
            total = 0.0
            for route in sorted(routes, key=lambda r: r[0]):
                total += core.evaluate_route(route)[1]
            result = check_td_solution(
                loaded,
                BenchmarkSolution(
                    instance_name=loaded.instance.instance_name, routes=routes
                ),
            )
            from mamut_routing_lib.checker import SolutionCheckStatus

            capacity_ok = result.status not in (
                SolutionCheckStatus.VEHICLE_CAPACITY_EXCEEDED,
                SolutionCheckStatus.TOO_MANY_VEHICLES_USED,
            )
            if result.is_valid():
                assert result.routing_cost == total, (instance_path, routes)
            else:
                # Random routes may violate capacity/fleet limits; timing must
                # still have been feasible per-route above.
                assert capacity_ok is False, (instance_path, result.status)

    # The generator must actually exercise the priced (feasible) path.
    assert feasible_routes > 0, (instance_path, "no feasible route drawn")


@pytest.mark.parametrize(
    "instance_path", DABIA_TDVRPTW, ids=lambda p: f"TDVRPTW-{p.parent.name}-{p.name.removesuffix('.vrp.json')}"
)
def test_dabia2013_tdvrptw_equivalence(instance_path) -> None:
    assert_instance_equivalence(instance_path)


@pytest.mark.parametrize(
    "instance_path", DABIA_TDVRP, ids=lambda p: f"TDVRP-{p.parent.name}-{p.name.removesuffix('.vrp.json')}"
)
def test_dabia2013_tdvrp_equivalence(instance_path) -> None:
    assert_instance_equivalence(instance_path)


@pytest.mark.parametrize(
    "instance_path", DABIA_TDVRPTW, ids=lambda p: f"BKS-{p.parent.name}-{p.name.removesuffix('.vrp.json')}"
)
def test_dabia2013_bks_repricing(instance_path) -> None:
    """Repriced best-known solutions: core canonical-order total == stored cost."""
    from mamut_routing_lib.bks import get_bks_path_for_instance, load_bks
    from mamut_routing_lib.enums import ObjectiveFunction

    bks_path = get_bks_path_for_instance(instance_path, ObjectiveFunction.DURATION)
    if not bks_path.exists():
        pytest.skip("no stored BKS for this instance")
    bks = load_bks(bks_path)
    loaded = load_td_instance(instance_path)
    core = to_core(loaded)

    total = 0.0
    for route in sorted(bks.routes, key=lambda r: r[0]):
        feasible, duration, _ = core.evaluate_route(route)
        assert feasible, (instance_path, route)
        total += duration
    assert total == bks.cost, (instance_path, total, bks.cost)


def test_benchmarks_available_or_skipped() -> None:
    from conftest import require_benchmarks

    require_benchmarks()
    assert DABIA_TDVRPTW, "benchmarks root found but no Dabia2013 TDVRPTW instances"
