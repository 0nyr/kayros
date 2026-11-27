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
    """Checked incumbent and termination of the BPC for the Duration objective.

    Status is "optimal" only when the ledger of the exact layer certified the run and the final gate passed. The gate requires that the exact cost U of the ledger equals, as a Fraction, the cost that the library checker computes for the returned routes, the returned routes are the ledger's incumbent, and the header of the ledger record describes the instance. Then certified_dual_bound is U. Otherwise status is "feasible" when a checked solution exists, including a retained warm start, or "no_incumbent", and no dual bound is claimed.

    The rational cost and its display belong to the returned customer-only routes. A run without a checked incumbent has empty routes and no cost. The raw record contains the native numerical bounds and pricing diagnostics. Its routes also use customer IDs.
    """

    status: Literal["optimal", "feasible", "no_incumbent"]
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
    certified_dual_bound_binary64: float | None = None
    # Why a run that the search closed is not optimal (the final gate), or None.
    certificate_gate: str | None = None
    # Where the ledger record was written, when the caller asked for it.
    record_path: str | None = None

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


def _hex_or_none(token: str) -> float | None:
    return None if token == "none" else float.fromhex(token)


def _record_header(text: str) -> dict[str, Any]:
    """Parse the header of a ledger record (format of Ledger::write) with no C++ code."""
    lines = iter(text.split("\n"))

    def words(expected: str) -> list[str]:
        tokens = next(lines).split()
        if not tokens or tokens[0] != expected:
            raise ValueError(f"record header: expected {expected!r}")
        return tokens[1:]

    if next(lines).split() != ["kayros-ledger-record", "1"]:
        raise ValueError("record header: unknown format")
    (fleet,) = words("fleet")
    (capacity,) = words("capacity")
    lo, hi = words("departure")
    (close,) = words("close")
    (count,) = words("customers")
    customers = []
    for _ in range(int(count)):
        t = words("customer")
        if t[0] != "window":
            raise ValueError("record header: bad customer line")
        if t[1] == "none":
            window, rest = None, t[2:]
        else:
            window, rest = (float.fromhex(t[1]), float.fromhex(t[2])), t[3:]
        if rest[0] != "service" or rest[2] != "demand" or len(rest) != 4:
            raise ValueError("record header: bad customer line")
        customers.append((window, float.fromhex(rest[1]), int(rest[3])))
    (arc_count,) = words("arcs")
    arcs = {}
    for _ in range(int(arc_count)):
        t = words("arc")
        i, j, k = int(t[0]), int(t[1]), int(t[3])
        if t[2] != "knots" or len(t) != 4 + 2 * k:
            raise ValueError("record header: bad arc line")
        values = [float.fromhex(x) for x in t[4:]]
        arcs[(i, j)] = (values[0::2], values[1::2])
    if next(lines).split() != ["operations"]:
        raise ValueError("record header: no operations line")
    return {
        "fleet": None if fleet == "none" else int(fleet), "capacity": int(capacity),
        "departure": (float.fromhex(lo), float.fromhex(hi)), "close": _hex_or_none(close),
        "customers": customers, "arcs": arcs,
    }


def _expected_header(loaded: LoadedTDInstance) -> dict[str, Any]:
    """The header that the record must carry for this instance, from the loaded instance file.

    Depot departure bounds and close follow the exact column evaluation of the bridge, i.e. the horizon of the ATFs, narrowed by the depot window when the instance has windows.
    """
    instance, atfs = loaded.instance, loaded.atfs
    n = instance.num_customers
    tws = getattr(instance, "time_windows", None)
    lo, hi = float(atfs.horizon[0]), float(atfs.horizon[1])
    close = None
    if tws is not None:
        lo = max(lo, float(tws[0][0]))
        close = float(tws[0][1])
        hi = min(hi, close)
    customers = []
    for v in range(1, n + 1):
        window = (float(tws[v][0]), float(tws[v][1])) if tws is not None else None
        customers.append((window, float(instance.service_times[v]), int(instance.demands[v])))
    fleet = getattr(instance, "num_vehicles", None)
    return {
        "fleet": None if fleet is None else int(fleet), "capacity": int(instance.vehicle_capacity),
        "departure": (lo, hi), "close": close, "customers": customers,
        "arcs": {(int(i), int(j)): ([float(x) for x in f.xs], [float(y) for y in f.ys]) for (i, j), f in atfs.arcs.items()},
    }


def _header_mismatch(record: str, loaded: LoadedTDInstance) -> str | None:
    """None when the header of the record describes the instance, else the first field that differs.

    The verifier certifies the instance of the header, so a certificate counts only for an instance with the same capacity, fleet, depot bounds, customer windows, services and demands, and arc knots. Binary64 values are compared exactly.
    """
    try:
        got = _record_header(record)
    except (ValueError, StopIteration, IndexError) as error:
        return f"unreadable header ({error})"
    expected = _expected_header(loaded)
    for key in ("fleet", "capacity", "departure", "close", "customers", "arcs"):
        if got[key] != expected[key]:
            return key
    return None


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
    record_path: str | Path | None = None,
) -> ExactResult:
    """Search for a minimum-duration solution and check incumbents under td-binary64-exact-v1.

    Input, warm starts and incumbent callbacks follow kayros.solve. The native search optimizes Duration. Exact feasibility and strict rational improvement determine which candidates reach the callback and the result. The status "optimal" comes from the ledger of the exact layer (see ExactResult). When record_path is given, the ledger record is written there, certified or not, for the verifier program kayros_ledger_verify.
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

    ledger = raw.get("ledger") or {}
    record = ledger.get("record")
    written = None
    if record_path is not None and record is not None:
        Path(record_path).write_text(record)
        written = str(record_path)

    termination = _TERMINATION[log["status"]]
    reason = (raw.get("exact_layer") or {}).get("unresolved_reason")
    if log["status"] == "Unresolved" and reason and reason != "lp_unresolved":
        termination = reason

    # Final gate. The native status Optimum means that the ledger certified, and
    # the claim is made only when the Python checker agrees with the ledger.
    gate = None
    certified = None
    if log["status"] == "Optimum":
        if not (ledger.get("available") and ledger.get("certified")):
            gate = "ledger_not_certified"
        else:
            U = Fraction(ledger["U"])
            incumbent = [list(route) for route in ledger["incumbent"]]
            try:
                checked = prepared.evaluate(incumbent)
            except ExactRouteError as error:
                gate = f"checker_rejects_ledger_incumbent ({error.status.value})"
            else:
                if checked.cost != U:
                    gate = "checker_cost_differs_from_ledger"
                elif best is not None and best.cost < U:
                    gate = "cheaper_checked_incumbent"
                else:
                    gate = _header_mismatch(record, loaded)
                    if gate is not None:
                        gate = "record_header_mismatch: " + gate
            if gate is None:
                certified = (U, checked, incumbent)
        if gate is not None:
            termination = "certificate_gate_failed"

    if certified is not None:
        U, checked, incumbent = certified
        return ExactResult(
            status="optimal",
            termination_reason=termination,
            routes=incumbent,
            cost=checked.display_cost,
            exact_cost=checked.cost,
            exact_evaluation=checked.as_dict(),
            incumbents=incumbents,
            raw=raw,
            rejected_incumbents=rejected,
            certified_dual_bound=U,
            certified_dual_bound_binary64=float(U),
            record_path=written,
        )
    return ExactResult(
        status="feasible" if best is not None else "no_incumbent",
        termination_reason=termination,
        routes=best_routes,
        cost=best.display_cost if best is not None else None,
        exact_cost=best.cost if best is not None else None,
        exact_evaluation=best.as_dict() if best is not None else None,
        incumbents=incumbents,
        raw=raw,
        rejected_incumbents=rejected,
        certificate_gate=gate,
        record_path=written,
    )
