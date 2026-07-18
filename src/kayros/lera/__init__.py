"""Exact BPC component (Lera-Romero, Networks 2019), shipped in the default build.

The LP backend is HiGHS by default (no external dependency, built statically
into the wheels); CPLEX is a source-build opt-in via
``-DLERA_LP_BACKEND=cplex``. The bridge is in-memory: a loaded MAMUT TD
instance (+ ATF sidecars) is converted to the normalized payload the vendored
solver's preprocessing expects; no legacy instance files are involved.

Vertex conventions: MAMUT uses ``0..n`` with the depot at 0; Lera's BPC uses a
start depot ``o = 0``, customers ``1..n`` and a distinct end depot ``d = n+1``
(a copy of the depot). ``routes_to_mamut`` maps solver paths back.

Certificate arithmetic: all master objective coefficients are checker-exact
(every column repriced by the kayros port of the reference checker on the raw
MAMUT ATFs) and the reported ``value`` of a solution is bit-identical to
``compute_solution_cost`` on its routes. An optimality certificate therefore
reads as :data:`CERTIFICATE`: optimal under checker-exact route costs and
standard LP/pricing tolerances, with pricing completeness modulo Lera's
epsilon arithmetic (the labeling-internal comparisons keep the vendored
epsilon semantics; zero divergences observed in practice).
"""

from __future__ import annotations

import json
from typing import Any, Callable

from mamut_routing_lib.td import LoadedTDInstance


_AUTO_MEMORY_FRACTION = 0.8  # M13.2: fraction of available memory the auto guard may use


def _read_meminfo_kb(field: str) -> int | None:
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith(field + ":"):
                    return int(line.split()[1])
    except OSError:
        return None
    return None


def _read_own_rss_bytes() -> int | None:
    try:
        import os
        with open("/proc/self/statm") as f:
            resident_pages = int(f.read().split()[1])
        return resident_pages * os.sysconf("SC_PAGE_SIZE")
    except (OSError, IndexError, ValueError):
        return None


def _read_cgroup_limit_bytes() -> int | None:
    """Effective cgroup memory limit (v2 then v1), None when unlimited/unknown."""
    for path in ("/sys/fs/cgroup/memory.max", "/sys/fs/cgroup/memory/memory.limit_in_bytes"):
        try:
            with open(path) as f:
                raw = f.read().strip()
        except OSError:
            continue
        if raw == "max":
            return None
        try:
            value = int(raw)
        except ValueError:
            continue
        # cgroup v1 reports a huge sentinel (~PAGE_COUNTER_MAX) when unlimited.
        if 0 < value < 1 << 60:
            return value
    return None


def _auto_memory_limit_mb() -> float:
    """Resolve the default RSS watermark for the M13.2 memory self-guard.

    Rule: own current RSS + a fraction of the memory available right now,
    capped by the cgroup limit when one is set. On a quiet dedicated machine
    this approximates ~80% of RAM; on a shared node it never claims more than
    what is actually free at solve start. Returns 0.0 (guard disabled) when
    the /proc interface is unavailable (non-Linux); never a false verdict.
    """
    own_rss = _read_own_rss_bytes()
    available_kb = _read_meminfo_kb("MemAvailable")
    if own_rss is None or available_kb is None:
        return 0.0
    limit = own_rss + _AUTO_MEMORY_FRACTION * available_kb * 1024
    cgroup = _read_cgroup_limit_bytes()
    if cgroup is not None:
        limit = min(limit, _AUTO_MEMORY_FRACTION * cgroup)
    return limit / 1048576.0


def _atf_to_travel_time_pieces(xs, ys) -> list[list[list[float]]]:
    """ATF breakpoints -> Lera PWL travel-time pieces tau(t) = f(t) - t.

    Each piece is ``[[x1, y1], [x2, y2]]`` (goc ``LinearFunction`` JSON).
    Breakpoints are emitted verbatim: duplicate-x pairs (value jumps of
    stepwise ATFs) become genuine zero-width vertical pieces handled exactly
    by the tagged-vertical labeling (M13.0; the exact value-jump path is the
    production default on step-carrying instances; see
    :func:`_has_stepwise_atfs` in :func:`solve_duration`). The M5.7 forward
    mollifier (``_continuize_breakpoints``, a 1e-3 steep-bridge
    under-approximation) was retired with M13.0: its sliver admitted
    checker-infeasible priced columns (the 23 integrity-guard refusals) and
    its bridges amplified epsilon comparisons into O(step) mispricing. For
    jump-free ATFs this emission is bit-identical to what the mollifier
    produced (it was a no-op without duplicate-x pairs).
    """
    xs = [float(x) for x in xs]
    ys = [float(y) for y in ys]
    return [
        [[xs[k], ys[k] - xs[k]], [xs[k + 1], ys[k + 1] - xs[k + 1]]]
        for k in range(len(xs) - 1)
    ]


def _has_stepwise_atfs(loaded: LoadedTDInstance) -> bool:
    """True iff any arc ATF carries a value jump (duplicate-x breakpoints)."""
    return any(
        f.xs[k + 1] == f.xs[k]
        for f in loaded.atfs.arcs.values()
        for k in range(len(f.xs) - 1)
    )


def to_lera_payload(loaded: LoadedTDInstance) -> dict[str, Any]:
    """Build the normalized instance JSON consumed by the ``_lera`` bridge."""
    instance = loaded.instance
    atfs = loaded.atfs
    n = instance.num_customers
    nv = n + 2  # 0 = start depot, 1..n = customers, n+1 = end depot (copy of 0)

    # TDVRP instances carry no time windows: trivial-TW encoding maps
    # every vertex to the full horizon; Lera's own preprocessing then tightens
    # (earliest arrivals / latest feasible departures) to recover pruning.
    # Checker TDVRP semantics: only DEPARTURES must lie in the ATF domains
    # (the horizon); the return arrival at the depot is unbounded ("the route
    # ends upon arrival"), so the end-depot window and Lera's planning horizon
    # must extend to the maximum ATF image, not stop at h1 — capping at h1
    # shrinks the feasible set and produced provably-wrong "optima" on the
    # short-horizon R1xx TDVRP twins.
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

    # Raw MAMUT data for the bridge's checker-exact route pricing.
    # Lera's preprocessing mutates travel_times/time_windows in place and
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
    memory_limit_mb: float | None = None,
) -> dict[str, Any]:
    """Run the Lera BPC (duration objective) on a loaded MAMUT TD instance.

    Returns the parsed result JSON: ``exact_log`` (BCP execution log),
    ``incumbents`` (anytime UB stream), and ``value``/``routes`` when a
    solution was found. Routes are in Lera vertex numbering (see
    ``routes_to_mamut``).

    ``on_incumbent`` makes the solve anytime: it fires synchronously on
    every new BCP incumbent with the same record that lands in the result's
    ``incumbents`` array (``{"time", "value", "origin", "routes"}``, routes in
    Lera numbering). Keep the hook cheap — the solve blocks on it; an exception
    raised inside it aborts the solve and propagates.

    ``initial_routes`` warm-starts the BPC with heuristic incumbent
    routes as customer-id sequences without depots (``kayros.Solution.routes``
    fits directly). They are repriced under the solver's own arithmetic, added
    as initial columns, and — when they still partition the customers — their
    total becomes the initial upper bound; the result carries a ``warm_start``
    record. If the BPC proves optimality without improving on that bound, the
    result has the proven ``value`` but an **empty** ``routes`` list: the
    initial routes are the optimum and the caller already holds them.

    Gap diagnostic: on ``TimeLimitReached``, ``exact_log.best_bound``
    is a valid lower bound whenever the root relaxation finished (the best
    open node's bound; absent when the TL hit during root CG — a truncated
    restricted master value is *not* a valid bound), and
    ``exact_log.best_int_value`` the best upper bound, so
    ``(best_int_value - best_bound) / best_int_value`` is the proven gap.
    ``exact_log.root_lp_value`` carries the root LP bound itself.

    ``stab_alpha`` is the Neame dual-smoothing factor in [0, 1):
    pricing runs on EMA-smoothed duals with misprice-safe termination (CG
    only ever stops on true duals). Default 0.0 (off): smoothing measured
    monotonically harmful with the pool-based pricing ladder — experimental
    knob only. When on, the result carries a ``stabilization`` record.

    ``memory_limit_mb`` is the M13.2 memory self-guard: an RSS watermark the
    prover polls at the same points as the time-limit deadline. When crossed,
    the solve unwinds cleanly and returns ``exact_log.status ==
    "MemoryLimitReached"`` with honest bounds (an OPEN verdict, never a
    certificate), instead of the OS OOM-killing the process with no result
    (the full-horizon TDVRP label-accumulation pathology). ``None`` (default)
    resolves the limit from the machine: own RSS + ~80% of currently
    available memory, capped by the cgroup limit when one is set. ``0``
    disables the guard. The result's ``memory`` record carries the resolved
    limit, the peak RSS and whether the guard tripped.
    """
    try:
        from kayros import _lera
    except ImportError as exc:  # pragma: no cover - build-dependent
        raise ImportError(
            "kayros._lera is not available in this build. The exact BPC "
            "component requires a source build with -DKAYROS_WITH_LERA=ON "
            "(HiGHS backend by default; see cpp/lera/NOTICE.md)."
        ) from exc

    # M13.0 promotion: the exact value-jump labeling is the production path on
    # step-carrying instances. The KAYROS_STEP_EXACT flag (formerly a dev
    # toggle) is now set per solve from the instance data, BEFORE the payload
    # is built and before the solver constructs its instance (both consult
    # it): stepwise ATFs get true tagged verticals end to end; jump-free
    # instances take the unchanged v1.0.0 arithmetic bit-identically (the
    # reverse-side waiting bridging is a representation choice validated by
    # the standing jump-free certificates, not a step mollifier).
    import os as _os

    if _has_stepwise_atfs(loaded):
        _os.environ["KAYROS_STEP_EXACT"] = "1"
    else:
        _os.environ.pop("KAYROS_STEP_EXACT", None)

    payload = to_lera_payload(loaded)
    kwargs: dict[str, Any] = {
        "time_limit_s": time_limit_s,
        "cut_limit": cut_limit,
        "solution_limit": solution_limit,
        "stab_alpha": float(stab_alpha),
        "memory_limit_mb": (
            _auto_memory_limit_mb() if memory_limit_mb is None else float(memory_limit_mb)
        ),
    }
    if node_limit is not None:
        kwargs["node_limit"] = node_limit
    if on_incumbent is not None:
        def _hook(incumbent_json: str) -> None:
            on_incumbent(json.loads(incumbent_json))

        kwargs["on_incumbent"] = _hook
    if initial_routes is not None:
        kwargs["initial_routes"] = [[int(c) for c in route] for route in initial_routes]
    result = json.loads(_lera.solve_duration_json(json.dumps(payload), **kwargs))
    result["stepwise_atfs"] = any(
        xs[k + 1] == xs[k]
        for _, _, xs, _ in payload["mamut_raw"]["atfs"]
        for k in range(len(xs) - 1)
    )
    return result


def routes_to_mamut(routes: list[dict[str, Any]], num_customers: int) -> list[list[int]]:
    """Map Lera route paths back to MAMUT vertex numbering (end depot -> 0)."""
    end_depot = num_customers + 1
    return [[0 if v == end_depot else int(v) for v in route["path"]] for route in routes]


#: The honest wording of a kayros lera optimality certificate. The route-cost
#: arithmetic is exact by construction (see the module docstring); the LP dual
#: bounds carry the standard solver tolerances and pricing completeness is
#: modulo the vendored epsilon comparisons — so a certificate never claims
#: more than this sentence.
CERTIFICATE = (
    "optimal under checker-exact route costs and standard LP/pricing "
    "tolerances, completeness modulo Lera epsilon dominance"
)


def optimality_metadata(
    result: dict[str, Any],
    *,
    wall_time_s: float | None = None,
    time_limit_s: float | None = None,
    campaign: str | None = None,
    date: str | None = None,
) -> dict[str, Any] | None:
    """Build a structured optimality stamp from a :func:`solve_duration` result.

    Returns ``None`` unless the solve terminated with status ``Optimum``.
    Stepwise (value-jump) instances stamp like any other since M13.0: the
    exact tagged-vertical labeling is their production path, validated by the
    2026-07-17/18 full-family campaign (0 unsound, 0 poisoned, cross-platform
    exact agreement; the former single-run refusal guarded the retired
    mollified path, see ``cpp/lera/NOTICE.md`` item 9).
    Returns a plain dict in the shape mamut-routing-lib stores under
    BKS ``metadata["optimality"]`` (``OptimalityMetadata``): the prover string
    names this kayros version and LP backend, ``certificate`` is
    :data:`CERTIFICATE`, ``proven_optimum`` is the solve's checker-exact
    objective and ``dual_bound`` the matching lower bound. ``campaign`` (a
    self-contained provenance sentence), ``wall_time_s`` and ``time_limit_s``
    are the caller's to provide; ``date`` defaults to today (ISO).
    """
    exact_log = result.get("exact_log") or {}
    if exact_log.get("status") != "Optimum":
        return None

    from datetime import date as _date

    from kayros import __version__, _lera

    backend = getattr(_lera, "LP_BACKEND", "HiGHS")
    stamp: dict[str, Any] = {
        "proven": True,
        "prover": f"kayros {__version__} lera BPC ({backend} LP backend)",
        "certificate": CERTIFICATE,
        "date": date if date is not None else _date.today().isoformat(),
        "arithmetic": "checker-exact-routes",
        "proven_optimum": result.get("value"),
        "dual_bound": exact_log.get("best_bound"),
    }
    if wall_time_s is not None:
        stamp["wall_time_s"] = float(wall_time_s)
    if time_limit_s is not None:
        stamp["time_limit_s"] = float(time_limit_s)
    if campaign is not None:
        stamp["campaign"] = campaign
    return stamp
