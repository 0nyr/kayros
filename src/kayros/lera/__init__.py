"""Optional exact BPC component (Lera-Romero, Networks 2019).

Not shipped in the default build yet: it requires a source build with
``-DKAYROS_WITH_LERA=ON`` (plan 2, Stream 5). The LP backend is HiGHS by
default (no external dependency); CPLEX is an opt-in via
``-DLERA_LP_BACKEND=cplex``. The bridge is in-memory: a loaded MAMUT TD
instance (+ ATF sidecars) is converted to the normalized payload the vendored
solver's preprocessing expects; no legacy instance files are involved.

Vertex conventions: MAMUT uses ``0..n`` with the depot at 0; Lera's BPC uses a
start depot ``o = 0``, customers ``1..n`` and a distinct end depot ``d = n+1``
(a copy of the depot). ``routes_to_mamut`` maps solver paths back.

Certificate arithmetic (M5.6 stage A): all master objective coefficients are
checker-exact (every column repriced by the kayros port of the reference
checker on the raw MAMUT ATFs) and the reported ``value`` of a solution is
bit-identical to ``compute_solution_cost`` on its routes. An optimality
certificate reads: optimal under checker-exact route costs and standard
LP/pricing tolerances, with pricing completeness modulo Lera's epsilon
arithmetic (the labeling-internal engine swap was deliberately skipped —
Onyr, 2026-07-06: dust-probability residue, zero divergences observed).
"""

from __future__ import annotations

import json
from typing import Any, Callable

from mamut_routing_lib.td import LoadedTDInstance


def _atf_to_travel_time_pieces(xs, ys) -> list[list[list[float]]]:
    """ATF breakpoints -> Lera PWL travel-time pieces tau(t) = f(t) - t.

    Each piece is ``[[x1, y1], [x2, y2]]`` (goc ``LinearFunction`` JSON).
    """
    xs = [float(x) for x in xs]
    ys = [float(y) for y in ys]
    return [
        [[xs[k], ys[k] - xs[k]], [xs[k + 1], ys[k + 1] - xs[k + 1]]]
        for k in range(len(xs) - 1)
    ]


def to_lera_payload(loaded: LoadedTDInstance) -> dict[str, Any]:
    """Build the normalized instance JSON consumed by the ``_lera`` bridge."""
    instance = loaded.instance
    atfs = loaded.atfs
    n = instance.num_customers
    nv = n + 2  # 0 = start depot, 1..n = customers, n+1 = end depot (copy of 0)

    # TDVRP instances carry no time windows (M5.4): trivial-TW encoding maps
    # every vertex to the full horizon; Lera's own preprocessing then tightens
    # (earliest arrivals / latest feasible departures) to recover pruning.
    # Checker TDVRP semantics: only DEPARTURES must lie in the ATF domains
    # (the horizon); the return arrival at the depot is unbounded ("the route
    # ends upon arrival"), so the end-depot window and Lera's planning horizon
    # must extend to the maximum ATF image, not stop at h1 — capping at h1
    # shrinks the feasible set and produced provably-wrong "optima" on the
    # short-horizon R1xx TDVRP twins (M5.4 gate anomaly).
    tws = getattr(instance, "time_windows", None)
    horizon = [float(atfs.horizon[0]), float(atfs.horizon[1])]
    if tws is None:
        h0, h1 = horizon
        t_ext = max(h1, max(max(f.ys) for f in atfs.arcs.values()))
        time_windows = [[h0, h1] for _ in range(n + 1)]
        time_windows.append([h0, t_ext])  # end depot: arrival may exceed h1
        horizon = [h0, t_ext]
    else:
        time_windows = [[float(earliest), float(latest)] for earliest, latest in tws]
        time_windows.append(list(time_windows[0]))  # end depot mirrors the depot

    demands = [int(q) for q in instance.demands] + [0]
    service_times = [float(s) for s in instance.service_times] + [0.0]

    arcs = [[0] * nv for _ in range(nv)]
    travel_times: list[list[Any]] = [[[] for _ in range(nv)] for _ in range(nv)]
    for (i, j), f in atfs.arcs.items():
        pieces = _atf_to_travel_time_pieces(f.xs, f.ys)
        if j == 0:
            i2, j2 = i, n + 1  # arcs into the depot go to the end-depot copy
        else:
            i2, j2 = i, j
        arcs[i2][j2] = 1
        travel_times[i2][j2] = pieces

    # Raw MAMUT data for the bridge's checker-exact route pricing (M5.6 stage
    # A). Lera's preprocessing mutates travel_times/time_windows in place and
    # the tau pieces store y-x (whose re-addition is not bit-exact), so the
    # certification arithmetic gets the untouched ATF breakpoints verbatim.
    mamut_raw = {
        "num_customers": n,
        "num_vehicles": int(getattr(instance, "num_vehicles", -1) or -1),
        "vehicle_capacity": int(instance.vehicle_capacity),
        "horizon": [float(atfs.horizon[0]), float(atfs.horizon[1])],
        "has_time_windows": tws is not None,
        "demands": [int(q) for q in instance.demands],
        "service_times": [float(s) for s in instance.service_times],
        "tw_earliest": [float(e) for e, _ in tws] if tws is not None else [],
        "tw_latest": [float(l) for _, l in tws] if tws is not None else [],
        "atfs": [
            [int(i), int(j), [float(x) for x in f.xs], [float(y) for y in f.ys]]
            for (i, j), f in atfs.arcs.items()
        ],
    }

    return {
        "problem_type": "TDVRPTW" if tws is not None else "TDVRP",
        "mamut_raw": mamut_raw,
        "benchmark_basename": "MAMUT-TD",
        "instance_basename": getattr(instance, "name", "mamut-td-instance"),
        "nb_vertices": nv,
        "start_depot": 0,
        "end_depot": n + 1,
        "nb_vehicles": int(getattr(instance, "num_vehicles", -1) or -1),
        "vehicle_capacity": float(instance.vehicle_capacity),
        "horizon": horizon,
        "time_windows": time_windows,
        "demands": demands,
        "service_times": service_times,
        "arc_count": sum(map(sum, arcs)),
        "arcs": arcs,
        "travel_times": travel_times,
    }


def solve_duration(
    loaded: LoadedTDInstance,
    *,
    time_limit_s: float = 7200.0,
    cut_limit: int = 100,
    node_limit: int | None = None,
    solution_limit: int = 3000,
    on_incumbent: Callable[[dict[str, Any]], None] | None = None,
    initial_routes: list[list[int]] | None = None,
    stab_alpha: float = 0.0,
) -> dict[str, Any]:
    """Run the Lera BPC (duration objective) on a loaded MAMUT TD instance.

    Returns the parsed result JSON: ``exact_log`` (BCP execution log),
    ``incumbents`` (anytime UB stream), and ``value``/``routes`` when a
    solution was found. Routes are in Lera vertex numbering (see
    ``routes_to_mamut``).

    ``on_incumbent`` makes the solve anytime (M5.2): it fires synchronously on
    every new BCP incumbent with the same record that lands in the result's
    ``incumbents`` array (``{"time", "value", "origin", "routes"}``, routes in
    Lera numbering). Keep the hook cheap — the solve blocks on it; an exception
    raised inside it aborts the solve and propagates.

    ``initial_routes`` warm-starts the BPC (M5.3) with heuristic incumbent
    routes as customer-id sequences without depots (``kayros.Solution.routes``
    fits directly). They are repriced under the solver's own arithmetic, added
    as initial columns, and — when they still partition the customers — their
    total becomes the initial upper bound; the result carries a ``warm_start``
    record. If the BPC proves optimality without improving on that bound, the
    result has the proven ``value`` but an **empty** ``routes`` list: the
    initial routes are the optimum and the caller already holds them.

    Gap diagnostic (M5.5): on ``TimeLimitReached``, ``exact_log.best_bound``
    is a valid lower bound whenever the root relaxation finished (the best
    open node's bound; absent when the TL hit during root CG — a truncated
    restricted master value is *not* a valid bound), and
    ``exact_log.best_int_value`` the best upper bound, so
    ``(best_int_value - best_bound) / best_int_value`` is the proven gap.
    ``exact_log.root_lp_value`` carries the root LP bound itself.

    ``stab_alpha`` (M5.1b) is the Neame dual-smoothing factor in [0, 1):
    pricing runs on EMA-smoothed duals with misprice-safe termination (CG
    only ever stops on true duals). Default 0.0 (off): smoothing measured
    monotonically harmful with the pool-based pricing ladder — experimental
    knob only. When on, the result carries a ``stabilization`` record.
    """
    try:
        from kayros import _lera
    except ImportError as exc:  # pragma: no cover - build-dependent
        raise ImportError(
            "kayros._lera is not available in this build. The exact BPC "
            "component requires a source build with -DKAYROS_WITH_LERA=ON "
            "(HiGHS backend by default; see cpp/lera/NOTICE.md)."
        ) from exc

    payload = to_lera_payload(loaded)
    kwargs: dict[str, Any] = {
        "time_limit_s": time_limit_s,
        "cut_limit": cut_limit,
        "solution_limit": solution_limit,
        "stab_alpha": float(stab_alpha),
    }
    if node_limit is not None:
        kwargs["node_limit"] = node_limit
    if on_incumbent is not None:
        def _hook(incumbent_json: str) -> None:
            on_incumbent(json.loads(incumbent_json))

        kwargs["on_incumbent"] = _hook
    if initial_routes is not None:
        kwargs["initial_routes"] = [[int(c) for c in route] for route in initial_routes]
    result = _lera.solve_duration_json(json.dumps(payload), **kwargs)
    return json.loads(result)


def routes_to_mamut(routes: list[dict[str, Any]], num_customers: int) -> list[list[int]]:
    """Map Lera route paths back to MAMUT vertex numbering (end depot -> 0)."""
    end_depot = num_customers + 1
    return [[0 if v == end_depot else int(v) for v in route["path"]] for route in routes]
