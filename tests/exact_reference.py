"""Exact reference evaluation of routes through mamut-routing-lib 0.13.

The solver's binary64 fold is an internal approximation with no bit-level
contract with the library (library 0.13 evaluates exactly, in rational
arithmetic). Tests that used to compare the solver with the library float
checker bit for bit now compare the solver's float route evaluation with the
library's exact one:

1. the feasibility verdicts agree (``route_timing`` raises ``ExactRouteError``
   on an infeasible route), and
2. the float duration lies within ``TOL * max(1, |exact|)`` of the exact one,
   measured in exact arithmetic (``Fraction(float_value)`` is exact).

``TOL`` is a round value about 100 times above the largest distance measured on
the jump-free families (see the migration report). The largest relative and
absolute distances seen during a run are printed at the end of the pytest
session, per family, so the margin stays visible.
"""

from __future__ import annotations

import os
from fractions import Fraction

from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.exact_td import prepare_td
from mamut_routing_lib.exact_types import ExactRouteError

# Relative tolerance of the float fold against the exact evaluation. The
# environment override exists for measurement runs (KAYROS_FOLD_TOL=inf).
TOL = float(os.environ.get("KAYROS_FOLD_TOL", "1e-12"))

# One prepared snapshot per loaded instance (the library copies every arc on
# prepare, so building it per route would be prohibitive). The loaded object is
# kept alive in the value so that its id() cannot be reused.
_PREPARED: dict[int, tuple[object, object]] = {}

# family -> [max relative, max absolute, number of routes]
DISTANCES: dict[str, list] = {}
# (family, context) -> max relative, for per-instance reporting
DISTANCES_BY_CONTEXT: dict[tuple[str, str], float] = {}


def prepared(loaded):
    """PreparedTD under the Duration objective, built once per loaded instance."""
    entry = _PREPARED.get(id(loaded))
    if entry is None:
        handle = prepare_td(
            loaded.instance, loaded.atfs, objective_function=ObjectiveFunction.DURATION
        )
        entry = (loaded, handle)
        _PREPARED[id(loaded)] = entry
    return entry[1]


def family_of(loaded) -> str:
    return str(loaded.atfs.benchmark_name)


def exact_route(loaded, route):
    """Exact route evaluation (duration, departure_time as Fractions), or None
    when the route is timing-infeasible."""
    try:
        return prepared(loaded).route_timing(list(route))
    except ExactRouteError as error:
        status = getattr(error.status, "name", str(error.status))
        if status not in ("TIME_WINDOW_VIOLATED", "ROUTE_TIMING_INFEASIBLE"):
            raise
        return None


def exact_routes_cost(loaded, routes) -> Fraction | None:
    """Exact sum of the route durations (no coverage, capacity or fleet check),
    or None when one route is infeasible."""
    total = Fraction(0)
    for route in routes:
        evaluation = exact_route(loaded, route)
        if evaluation is None:
            return None
        total += evaluation.duration
    return total


def distance(float_value: float, exact: Fraction) -> tuple[float, float]:
    """(relative, absolute) distance of a float to an exact value, the relative
    one being scaled by max(1, |exact|). Computed exactly, then displayed."""
    diff = abs(Fraction(float_value) - exact)
    scale = max(Fraction(1), abs(exact))
    return float(diff / scale), float(diff)


def record(loaded, float_value: float, exact: Fraction, context: str = "") -> tuple[float, float]:
    rel, absolute = distance(float_value, exact)
    family = family_of(loaded)
    entry = DISTANCES.setdefault(family, [0.0, 0.0, 0])
    entry[0] = max(entry[0], rel)
    entry[1] = max(entry[1], absolute)
    entry[2] += 1
    key = (family, context or str(loaded.instance_path))
    DISTANCES_BY_CONTEXT[key] = max(DISTANCES_BY_CONTEXT.get(key, 0.0), rel)
    return rel, absolute


def assert_close(loaded, float_value: float, exact: Fraction, ctx=None) -> None:
    """The solver's float value is within TOL of the exact one."""
    rel, absolute = record(loaded, float_value, exact, context=str(getattr(loaded, "instance_path", "")))
    assert rel <= TOL, (ctx, "fold distance", rel, absolute, float_value, float(exact))


def assert_route_agrees(loaded, core, route, ctx=None):
    """Feasibility agreement and duration distance for one route, solver float
    evaluation (``core.evaluate_route``) against the exact library one. Returns
    (feasible, float_duration, exact_evaluation_or_None)."""
    feasible, duration, departure = core.evaluate_route(route)
    reference = exact_route(loaded, route)
    assert feasible == (reference is not None), (ctx, route, "feasibility")
    if reference is not None:
        assert_close(loaded, duration, reference.duration, (ctx, route))
    return feasible, duration, reference


def distance_summary_lines() -> list[str]:
    lines = [f"fold distance versus exact evaluation (TOL = {TOL:g})"]
    for family in sorted(DISTANCES):
        rel, absolute, count = DISTANCES[family]
        lines.append(f"  {family}: max relative {rel:.3e}, max absolute {absolute:.3e}, {count} evaluations")
    if os.environ.get("KAYROS_FOLD_VERBOSE") == "1":
        for (family, context), rel in sorted(DISTANCES_BY_CONTEXT.items()):
            lines.append(f"    {family} {context}: {rel:.3e}")
    return lines


def assert_exact_check_valid(loaded, routes, float_value: float) -> None:
    """The whole solution passes the library's exact checker under Duration
    (capacity, time windows, fleet) and the solver's float value for it lies
    within TOL of the exact cost (the declared cost is deliberately omitted:
    library 0.13 requires it to be the display of the exact cost)."""
    from mamut_routing_lib.models import BenchmarkSolution
    from mamut_routing_lib.td import check_td_solution

    routes = [list(r) for r in routes]
    check = check_td_solution(
        loaded,
        BenchmarkSolution(instance_name=loaded.instance.instance_name, routes=routes),
        ObjectiveFunction.DURATION,
    )
    assert check.is_valid(), (check.status, check.error_message)
    exact = exact_routes_cost(loaded, routes)
    assert exact is not None
    assert check.routing_cost == float(exact)
    assert_close(loaded, float_value, exact, "solution")


def exact_duration_cost(instance, atfs, routes) -> Fraction:
    """Exact Duration cost of a complete solution (library ``compute_solution_cost``,
    objective given explicitly since the library default is the family's)."""
    from mamut_routing_lib.td import compute_solution_cost

    return compute_solution_cost(
        instance, atfs, routes, objective_function=ObjectiveFunction.DURATION
    )


def assert_value_close(
    float_value: float, exact: Fraction, ctx=None, family: str = "exact-component (lera)"
) -> None:
    """A float objective value within TOL of an exact one (no loaded instance
    needed). Used for the exact component, whose internal arithmetic still
    runs the old float fold."""
    rel, absolute = distance(float_value, exact)
    entry = DISTANCES.setdefault(family, [0.0, 0.0, 0])
    entry[0] = max(entry[0], rel)
    entry[1] = max(entry[1], absolute)
    entry[2] += 1
    assert rel <= TOL, (ctx, "value distance", rel, absolute, float_value, float(exact))
