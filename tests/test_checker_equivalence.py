"""M3.2 gate: core route pricing must agree with the library's exact evaluation.

For random routes over real MAMUT instances (feasible or not), the compiled
core's feasibility verdict must equal the library's, and its binary64 duration
must lie within ``exact_reference.TOL`` (relative) of the library's exact
duration. Since library 0.13 the checker evaluates in exact rational
arithmetic and the solver's binary64 fold is an internal approximation, so
there is no bit-identity contract any more (before 0.13 this gate was ``==``).
The ready-time function breakpoints used to be compared with the library's
``compute_route_ready_time_function``, which no longer exists: that comparison
is deleted here (the solver's own function is covered by the warp prefix gate
in test_warp_equivalence and by test_pwlf_equivalence).
"""

import random
import zlib

import pytest
from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.td import check_td_solution, load_td_instance
from mamut_routing_lib.models import BenchmarkSolution

from kayros.io import to_core

from conftest import family_instances
from exact_reference import assert_close, assert_route_agrees, exact_routes_cost

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
            feasible, _, _ = assert_route_agrees(loaded, core, route, instance_path)
            if feasible:
                feasible_routes += 1
            else:
                all_feasible = False

        if all_feasible:
            # Full-solution agreement: the library's exact cost against the
            # sum of the core's float durations (canonical route order).
            total = 0.0
            for route in sorted(routes, key=lambda r: r[0]):
                total += core.evaluate_route(route)[1]
            result = check_td_solution(
                loaded,
                BenchmarkSolution(
                    instance_name=loaded.instance.instance_name, routes=routes
                ),
                ObjectiveFunction.DURATION,
            )
            from mamut_routing_lib.checker import SolutionCheckStatus

            capacity_ok = result.status not in (
                SolutionCheckStatus.VEHICLE_CAPACITY_EXCEEDED,
                SolutionCheckStatus.TOO_MANY_VEHICLES_USED,
            )
            if result.is_valid():
                exact = exact_routes_cost(loaded, routes)
                assert exact is not None and result.routing_cost == float(exact), (
                    instance_path, "library cost is not the display of the exact sum")
                assert_close(loaded, total, exact, (instance_path, routes))
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
    """Stored routes: core total is within TOL of the library's exact repricing.

    The benchmark snapshot's declared cost may use a different numerical
    contract. Compare the same routes under the installed checker instead.
    """
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
        feasible, duration, _ = assert_route_agrees(loaded, core, route, instance_path)
        assert feasible, (instance_path, route)
        total += duration
    reference = check_td_solution(
        loaded,
        BenchmarkSolution(instance_name=loaded.instance.instance_name, routes=bks.routes),
        ObjectiveFunction.DURATION,
    )
    assert reference.is_valid(), (instance_path, reference.status, reference.error_message)
    exact = exact_routes_cost(loaded, bks.routes)
    assert exact is not None and reference.routing_cost == float(exact)
    assert_close(loaded, total, exact, (instance_path, "BKS total"))


def test_benchmarks_available_or_skipped() -> None:
    from conftest import require_benchmarks

    require_benchmarks()
    assert DABIA_TDVRPTW, "benchmarks root found but no Dabia2013 TDVRPTW instances"
