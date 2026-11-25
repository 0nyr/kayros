"""Semantic checks of the packed heuristic fold against the exact arrival curve."""

import math
from fractions import Fraction

import pytest
from mamut_routing_lib.td import load_td_instance

from kayros import _core
from kayros.io import to_core

from conftest import family_instances
from exact_reference import assert_close, prepared


def _instances(family, size, name):
    return [p for p in family_instances("TDVRPTW", family, [size]) if p.name == name + ".vrp.json"]


@pytest.mark.parametrize("path", _instances("Dabia2013", "n=25", "C101"))
def test_continuous_windowed_curve(path):
    loaded = load_td_instance(path)
    core = to_core(loaded)
    exact = prepared(loaded)
    route = [1]
    curve = exact.route_curve(route)
    xs, ys = core.route_ready_time_function(route)
    assert_close(loaded, xs[0], curve[0].departure)
    assert_close(loaded, xs[-1], curve[-1].departure)
    for knot in curve:
        assert knot.arrival == knot.right_arrival
        x = min(max(float(knot.departure), xs[0]), xs[-1])
        assert_close(loaded, _core.pwlf_evaluate(xs, ys, x), knot.arrival)
    for left, right in zip(curve, curve[1:]):
        mid = float((left.departure + right.departure) / 2)
        if left.departure < mid < right.departure:
            weight = (Fraction(mid) - left.departure) / (right.departure - left.departure)
            expected = left.right_arrival + weight * (right.arrival - left.right_arrival)
            assert_close(loaded, _core.pwlf_evaluate(xs, ys, mid), expected)
    feasible, duration, departure = core.evaluate_route(route)
    assert feasible
    timing = exact.route_timing(route)
    assert_close(loaded, duration, timing.duration)
    start = min(max(Fraction(departure), curve[0].departure), curve[-1].departure)
    assert_close(loaded, departure, start)
    for left, right in zip(curve, curve[1:]):
        if left.departure <= start <= right.departure:
            weight = (start - left.departure) / (right.departure - left.departure)
            arrival = left.arrival + weight * (right.arrival - left.arrival)
            assert_close(loaded, duration, arrival - start)
            break
    else:
        pytest.fail(f"heuristic departure {departure} outside the exact curve")


@pytest.mark.parametrize("path", _instances("Rifki2020", "n=30", "Rifki-10"))
def test_windowed_jump_has_both_sides_and_an_exact_minimizer(path):
    loaded = load_td_instance(path)
    core = to_core(loaded)
    exact = prepared(loaded)
    route = [6, 17, 12, 22, 16]
    curve = exact.route_curve(route)
    timing = exact.route_timing(route)
    assert (curve[0].departure, curve[-1].departure) == (0, 32011)
    assert (timing.duration, timing.departure_time) == (2002, 26949)
    jump = next(knot for knot in curve if knot.departure == 27236)
    assert (jump.arrival, jump.right_arrival) == (29238, 29308)
    assert jump.arrival - jump.departure == timing.duration

    xs, ys = core.route_ready_time_function(route)
    assert (xs[0], xs[-1]) == (0, 32011)
    # The fold can shift a jump by one ULP. Verify both heights at the fold's jump.
    candidates = [i for i in range(len(xs) - 1) if xs[i] == xs[i + 1]]
    pos = min(candidates, key=lambda i: abs(xs[i] - jump.departure))
    assert abs(xs[pos] - jump.departure) <= math.ulp(float(jump.departure))
    assert_close(loaded, ys[pos], jump.arrival)
    assert_close(loaded, ys[pos + 1], jump.right_arrival)
    feasible, duration, _ = core.evaluate_route(route)
    assert feasible
    assert_close(loaded, duration, timing.duration)
