"""M3.4: all four MAMUT TD families × both problem types.

For each satellite family (Ari2018, Vu2020, Rifki2020 — Dabia2013 is covered
exhaustively by test_checker_equivalence): a checker-equivalence spot check on
random routes and a small-budget solve() smoke. Rifki2020 is the stress case:
its canonical ATFs carry genuine vertical steps (non-FIFO envelope
consolidation), exercising the step semantics of the engine end to end.
"""

import random
import zlib

import pytest
from mamut_routing_lib.td import (
    compute_route_duration,
    compute_route_ready_time_function,
    load_td_instance,
)

import kayros
from kayros.io import to_core

from conftest import family_instances
from test_checker_equivalence import random_routes

INSTANCES_PER_CASE = 3
EQUIVALENCE_DRAWS = 10

FAMILY_CASES = [
    ("TDVRPTW", "Ari2018", ["n=15", "n=40"]),
    ("TDVRP", "Ari2018", ["n=15", "n=40"]),
    ("TDVRPTW", "Vu2020", ["n=59"]),
    ("TDVRP", "Vu2020", ["n=59"]),
    ("TDVRPTW", "Rifki2020", ["n=10", "n=30"]),
    ("TDVRP", "Rifki2020", ["n=10", "n=30"]),
]


def cases():
    for problem_type, family, sizes in FAMILY_CASES:
        for size in sizes:
            paths = family_instances(problem_type, family, [size])
            for path in paths[:INSTANCES_PER_CASE]:
                name = path.name.removesuffix(".vrp.json")
                yield pytest.param(
                    path, id=f"{problem_type}-{family}-{size}-{name}"
                )


ALL_CASES = list(cases())


@pytest.mark.parametrize("instance_path", ALL_CASES)
def test_family_checker_equivalence(instance_path) -> None:
    loaded = load_td_instance(instance_path)
    core = to_core(loaded)
    rng = random.Random(zlib.crc32(str(instance_path).encode()))
    feasible_routes = 0
    for _ in range(EQUIVALENCE_DRAWS):
        for route in random_routes(rng, loaded.instance):
            reference = compute_route_duration(loaded.instance, loaded.atfs, route)
            feasible, duration, departure = core.evaluate_route(route)
            assert feasible == reference.feasible, (instance_path, route)
            if reference.feasible:
                feasible_routes += 1
                assert duration == reference.duration, (instance_path, route)
                assert departure == reference.departure_time, (instance_path, route)
            delta = compute_route_ready_time_function(loaded.instance, loaded.atfs, route)
            core_xs, core_ys = core.route_ready_time_function(route)
            assert core_xs == delta.xs, (instance_path, route)
            assert core_ys == delta.ys, (instance_path, route)
    assert feasible_routes > 0, (instance_path, "no feasible route drawn")


@pytest.mark.parametrize("instance_path", ALL_CASES)
def test_family_solve_smoke(instance_path) -> None:
    solution = kayros.solve(
        instance_path,
        kayros.Params(max_iterations=40, max_no_improvement=40),
        seed=1,
    )
    assert solution.duration > 0.0
    served = sorted(c for route in solution.routes for c in route)
    assert served == list(range(1, len(served) + 1))
