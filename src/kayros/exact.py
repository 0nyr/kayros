"""Branch-price-and-cut search with exact Duration evaluation of its incumbents."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Any, Literal

from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.exact import prepare_exact
from mamut_routing_lib.exact_types import ExactEvaluation, ExactRouteError
from mamut_routing_lib.td import LoadedTDInstance

from kayros import _bpc
from kayros.io import load_instance
from kayros.solve import Incumbent, IncumbentHook, KayrosError

__all__ = ["ExactResult", "solve_exact"]


@dataclass
class ExactResult:
    """Checked incumbent and diagnostic BPC termination for the Duration objective.

    Status is "feasible" when a checked solution exists, including a retained warm start, or "no_incumbent" otherwise. The independent termination reason "search_closed" records the BPC's numerical closure.

    The rational cost and its display belong to the returned customer-only routes. A run without a checked incumbent has empty routes and no cost. The raw record contains the native numerical bounds and pricing diagnostics. Its routes also use customer IDs. A certified dual bound requires verification under the exact profile.
    """

    status: Literal["feasible", "no_incumbent"]
    termination_reason: str
    routes: list[list[int]]
    cost: float | None
    exact_cost: Fraction | None
    incumbents: list[Incumbent]
    exact_evaluation: dict | None
    raw: dict[str, Any]
    rejected_incumbents: int
    objective: Literal["Duration"] = "Duration"
    certified_dual_bound: Fraction | None = None

    @property
    def certified_gap(self) -> Fraction | None:
        """Absolute gap when the exact incumbent and a certified dual bound are available."""
        if self.exact_cost is None or self.certified_dual_bound is None:
            return None
        return self.exact_cost - self.certified_dual_bound

    @property
    def reported_best_bound(self) -> float | None:
        """The numerical bound reported by the BPC, when its root relaxation finished."""
        return self.raw["exact_log"].get("best_bound")

    @property
    def closure_conditions(self) -> dict[str, bool | int]:
        return self.raw["closure_conditions"]

    @property
    def pricing_integrity(self) -> dict[str, float | int]:
        return self.raw["pricing_integrity"]


_TERMINATION = {
    "Optimum": "search_closed",
    "TimeLimitReached": "time_limit",
    "MemoryLimitReached": "memory_limit",
    "NodeLimitReached": "node_limit",
    "DidNotStart": "did_not_start",
    "Infeasible": "solver_reported_infeasible",
    "Unbounded": "solver_reported_unbounded",
    "Unresolved": "lp_unresolved",
}


def _customer_path(path: list[int], end_depot: int) -> list[int]:
    if len(path) < 3 or path[0] != 0 or path[-1] != end_depot:
        raise KayrosError("unexpected depot numbering in a native BPC path")
    return path[1:-1]


def solve_exact(
    instance: str | Path | LoadedTDInstance,
    *,
    time_limit: float = 7200.0,
    initial_routes: Sequence[Sequence[int]] | None = None,
    on_incumbent: IncumbentHook | None = None,
    verify: bool = False,
    node_limit: int | None = None,
    memory_limit_mb: float | None = None,
) -> ExactResult:
    """Search for a minimum-duration solution and check incumbents under td-binary64-exact-v1.

    Input, warm starts and incumbent callbacks follow kayros.solve. The native search optimizes Duration. Exact feasibility and strict rational improvement determine which candidates reach the callback and the result. The BPC's floating-point closure and bounds remain diagnostics pending exact dual verification.
    """
    if isinstance(instance, LoadedTDInstance):
        if verify:
            raise ValueError("verify=True requires an instance path")
        loaded = instance
    else:
        loaded = load_instance(instance, verify=verify)

    prepared = prepare_exact(loaded, objective_function=ObjectiveFunction.DURATION)
    end_depot = loaded.instance.num_customers + 1
    best: ExactEvaluation | None = None
    best_routes = []
    last_native_routes = None
    incumbents = []
    rejected = 0

    def record(evaluation, routes, seconds, origin, search_value):
        nonlocal best, best_routes
        if best is not None and evaluation.cost >= best.cost:
            return
        best = evaluation
        best_routes = [list(route) for route in routes]
        event = Incumbent(evaluation.display_cost, seconds, None, origin, evaluation.cost, search_value)
        incumbents.append(event)
        if on_incumbent is not None:
            on_incumbent(event, [list(route) for route in routes])

    def consider(routes, seconds, origin, search_value):
        nonlocal rejected
        if not routes or routes == best_routes:
            return
        try:
            evaluation = prepared.evaluate(routes)
        except ExactRouteError:
            rejected += 1
            return
        record(evaluation, routes, seconds, origin, search_value)

    seed = None
    if initial_routes is not None:
        try:
            evaluated = prepared.evaluate(initial_routes)
        except ExactRouteError as error:
            raise ValueError(f"invalid initial_routes ({error.status.value}): {error}") from error
        seed = [list(route) for route in initial_routes]
        record(evaluated, seed, 0.0, "warm_start", None)

    def native_incumbent(event):
        nonlocal last_native_routes
        last_native_routes = [_customer_path(route["path"], end_depot) for route in event["routes"]]
        consider(last_native_routes, event["time"], event["origin"], event["value"])

    raw = _bpc.solve_duration(
        loaded, time_limit=time_limit, initial_routes=seed, on_incumbent=native_incumbent,
        node_limit=node_limit, memory_limit_mb=memory_limit_mb,
    )
    log = raw["exact_log"]
    for entry in (raw, *raw["incumbents"]):
        if entry.get("routes"):
            entry["routes"] = [
                {**route, "path": _customer_path(route["path"], end_depot)} for route in entry["routes"]
            ]
    final_routes = [route["path"] for route in raw.get("routes", ())]
    if final_routes and final_routes != last_native_routes:
        consider(final_routes, log["time"], "bpc", raw["value"])

    return ExactResult(
        status="feasible" if best is not None else "no_incumbent",
        termination_reason=_TERMINATION[log["status"]],
        routes=best_routes,
        cost=best.display_cost if best is not None else None,
        exact_cost=best.cost if best is not None else None,
        exact_evaluation=best.as_dict() if best is not None else None,
        incumbents=incumbents,
        raw=raw,
        rejected_incumbents=rejected,
    )
