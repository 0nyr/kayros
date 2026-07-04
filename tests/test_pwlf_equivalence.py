"""Function-level equivalence: kayros._core pwlf primitives vs the reference
mamut_routing_lib.td.pwlf implementation (the canonical checker's engine).

All comparisons are exact (``==`` on floats, list equality on breakpoints):
the C++ port must be bit-identical, including at vertical steps and plateaus.
"""

import random

from mamut_routing_lib.td import NDCPWLF
from mamut_routing_lib.td.pwlf import make_theta

from kayros import _core


def random_ndcpwlf(rng: random.Random, allow_steps: bool = True) -> NDCPWLF:
    """Random non-decreasing CPWLF with occasional steps and plateaus."""
    n = rng.randint(2, 12)
    xs: list[float] = []
    ys: list[float] = []
    x = rng.uniform(-50.0, 50.0)
    y = rng.uniform(-50.0, 50.0)
    for _ in range(n):
        xs.append(x)
        ys.append(y)
        move = rng.random()
        if allow_steps and move < 0.15:
            y += rng.uniform(0.5, 10.0)  # vertical step: same x, bigger y
        elif move < 0.30:
            x += rng.uniform(0.5, 10.0)  # plateau: bigger x, same y
        else:
            x += rng.uniform(0.5, 10.0)
            y += rng.uniform(0.0, 10.0)
    return NDCPWLF(xs, ys)


def test_evaluate_equivalence() -> None:
    rng = random.Random(20260704)
    for _ in range(500):
        f = random_ndcpwlf(rng)
        for _ in range(20):
            x = rng.uniform(f.min_domain, f.max_domain)
            assert _core.pwlf_evaluate(f.xs, f.ys, x) == f.evaluate(x)
        # domain endpoints and exact breakpoints (step semantics)
        for x in (f.min_domain, f.max_domain, *f.xs):
            assert _core.pwlf_evaluate(f.xs, f.ys, x) == f.evaluate(x)


def test_compose_equivalence() -> None:
    rng = random.Random(42)
    nonempty = 0
    for _ in range(2000):
        f = random_ndcpwlf(rng)
        g = random_ndcpwlf(rng)
        expected = f.compose(g)
        got_xs, got_ys = _core.pwlf_compose(f.xs, f.ys, g.xs, g.ys)
        assert got_xs == expected.xs
        assert got_ys == expected.ys
        if not expected.is_empty():
            nonempty += 1
    assert nonempty > 500  # the generator must actually exercise composition


def test_compose_chain_equivalence() -> None:
    # Chained composition (the route-pricing pattern acc <- f ∘ acc).
    rng = random.Random(7)
    for _ in range(200):
        acc = NDCPWLF.identity(0.0, rng.uniform(10.0, 100.0))
        core_xs, core_ys = list(acc.xs), list(acc.ys)
        for _ in range(rng.randint(1, 8)):
            f = random_ndcpwlf(rng)
            acc = f.compose(acc)
            core_xs, core_ys = _core.pwlf_compose(f.xs, f.ys, core_xs, core_ys)
            assert core_xs == acc.xs
            assert core_ys == acc.ys
            if acc.is_empty():
                break


def test_min_shifted_image_equivalence() -> None:
    rng = random.Random(123)
    for _ in range(1000):
        f = random_ndcpwlf(rng)
        assert _core.pwlf_min_shifted_image(f.xs, f.ys) == f.min_shifted_image()


def test_make_theta_equivalence() -> None:
    rng = random.Random(99)
    cases = [(0.0, 0.0, 0.0), (0.0, 10.0, 2.0), (5.0, 5.0, 1.0), (3.0, 8.0, 0.0)]
    for _ in range(200):
        earliest = rng.uniform(0.0, 100.0)
        cases.append((earliest, earliest + rng.uniform(0.0, 50.0), rng.uniform(0.0, 30.0)))
    for earliest, latest, service_time in cases:
        expected = make_theta(earliest, latest, service_time)
        got_xs, got_ys = _core.pwlf_make_theta(earliest, latest, service_time)
        assert got_xs == expected.xs
        assert got_ys == expected.ys


def test_identity() -> None:
    assert _core.pwlf_identity(0.0, 10.0) == ([0.0, 10.0], [0.0, 10.0])
    assert _core.pwlf_identity(5.0, 5.0) == ([5.0], [5.0])
