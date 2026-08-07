"""Greedy-seed construction gate (1.4.0).

Through 1.3.0 the seed ran a full TD Dijkstra over the free customers at every
placement and selected on the earliest MULTI-HOP ready time. 1.4.0 selects on
the direct ready time instead, which is the same rule up to ties: Dijkstra's
first settled node is by construction the free customer with the smallest
direct ready time, so the multi-hop minimum equals the direct minimum and is
attained at the same customer. The two can only part where a detour reaches a
customer in exactly the same total time — a time window binding hard enough for
the wait to absorb it — and there the lookahead is a different tie break, not a
better choice.

These tests hold the claim to account: the shipped construction must agree with
the retained lookahead reference on every instance in the checkout, and it must
be dramatically cheaper. A failure here does not mean the seed is wrong; it
means the tie case is real on that instance and the plan-13 M9 arm's
trajectory-equality claim needs re-verification for it.
"""

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

    ok_fast, routes_fast = _core.greedy_makespan(core)
    ok_ref, routes_ref = _core.greedy_makespan_lookahead(core)

    assert ok_fast == ok_ref
    assert [list(r) for r in routes_fast] == [list(r) for r in routes_ref], (
        "the shipped seed parted from the lookahead reference on this instance"
    )
    assert _core.solution_duration(core, routes_fast) == _core.solution_duration(
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
    _core.greedy_makespan(core)
    fast = time.perf_counter() - start
    start = time.perf_counter()
    _core.greedy_makespan_lookahead(core)
    reference = time.perf_counter() - start

    assert reference > 5.0 * fast, (fast, reference)
