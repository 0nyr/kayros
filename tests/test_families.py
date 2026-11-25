"""Check exact feasibility, binary64 duration and short public solves on Ari2018, Vu2020 and Rifki2020, with and without windows. The Dabia2013 routes are covered by test_checker_equivalence. Rifki2020 exercises vertical jumps in the arrival functions."""

import random
import zlib

import pytest
from mamut_routing_lib.td import load_td_instance

import kayros
from kayros.io import to_core

from conftest import family_instances
from exact_reference import assert_route_agrees
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
            feasible, _, _ = assert_route_agrees(loaded, core, route, instance_path)
            feasible_routes += feasible
    assert feasible_routes > 0, (instance_path, "no feasible route drawn")


@pytest.mark.parametrize("instance_path", ALL_CASES)
def test_family_solve_smoke(instance_path) -> None:
    solution = kayros.solve(
        instance_path,
        kayros.Params(ils_max_iterations=250),
        seed=1,
    )
    assert solution.cost > 0.0
    served = sorted(c for route in solution.routes for c in route)
    assert served == list(range(1, len(served) + 1))
