"""Compare direct greedy selection with the retained TD Dijkstra lookahead on the installed families. A time window may absorb a detour and create a tie. The tests compare the constructed routes and measure the cost of the two selection rules."""

import time

import pytest

from kayros import _core
from kayros.io import load_instance, to_core

from conftest import family_instances, require_benchmarks

# One size dir per family, spanning the representations the campaign uses.
FAMILIES = [
    ("TDVRPTW", "Dabia2013", "n=50"),
    ("TDVRPTW", "Dabia2013", "n=100"),
    ("TDVRPTW", "Rifki2020", "n=50"),
    ("TDVRPTW", "Ari2018", "n=40"),
    ("TDVRPTW", "Vu2020", "n=99"),
]


def instances():
    paths = []
    for problem_type, family, size_dir in FAMILIES:
        paths.extend(family_instances(problem_type, family, [size_dir])[:6])
    return paths


@pytest.mark.parametrize("path", instances(), ids=lambda p: p.stem)
def test_seed_matches_the_lookahead_reference(path) -> None:
    require_benchmarks()
    core = to_core(load_instance(path, verify=True))

    ok_fast, routes_fast = _core.greedy_construct(core)
    ok_ref, routes_ref = _core.greedy_construct_lookahead(core)

    assert ok_fast == ok_ref
    assert [list(r) for r in routes_fast] == [list(r) for r in routes_ref], (
        "the shipped seed parted from the lookahead reference on this instance"
    )
    assert _core.solution_cost(core, routes_fast) == _core.solution_cost(
        core, routes_ref
    )


def test_seed_is_cheaper_than_the_lookahead_reference() -> None:
    """The whole point of dropping the lookahead: an O(n^2) construction
    instead of an O(n^3) one. The margin grows with n (about 60x at n=100 and
    1200x at n=1000 on one modern core); assert a deliberately loose 5x so the
    test states the direction without being a benchmark."""
    require_benchmarks()
    paths = family_instances("TDVRPTW", "Dabia2013", ["n=100"])
    if not paths:
        pytest.skip("Dabia2013 n=100 not present")
    core = to_core(load_instance(paths[0], verify=True))

    start = time.perf_counter()
    _core.greedy_construct(core)
    fast = time.perf_counter() - start
    start = time.perf_counter()
    _core.greedy_construct_lookahead(core)
    reference = time.perf_counter() - start

    assert reference > 5.0 * fast, (fast, reference)
