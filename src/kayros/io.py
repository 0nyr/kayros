"""Bridge between mamut-routing-lib TD artifacts and the kayros compiled core.

The core consumes the checker conventions verbatim: vertices ``0..n`` with the
depot at 0, arrival-time functions per arc over the horizon, exact doubles.
No family-specific loading happens here — mamut-routing-lib already normalizes
every benchmark family to the canonical format.
"""

from __future__ import annotations

from pathlib import Path

from mamut_routing_lib.td import LoadedTDInstance, load_td_instance

from kayros import _core

# Canonical objective value strings (the lib enum's values), keyed by their
# case/underscore-folded spellings. Kept as plain strings so kayros keeps
# importing against mamut-routing-lib < 0.9.0 for Duration work.
_OBJECTIVE_BY_KEY = {
    "duration": "Duration",
    "fleetcostduration": "FleetCostDuration",
    "fleet_cost_duration": "FleetCostDuration",
}


def canonical_objective(objective_function: object) -> str:
    """``"Duration"`` or ``"FleetCostDuration"`` from a lib
    ``ObjectiveFunction`` member or a string (case/underscore tolerant).
    Raises ``ValueError`` on anything else."""
    key = str(getattr(objective_function, "value", objective_function))
    key = key.replace("-", "_").casefold()
    try:
        return _OBJECTIVE_BY_KEY[key]
    except KeyError:
        raise ValueError(
            f"unknown objective {objective_function!r}: kayros supports "
            f"'duration' and 'fleet_cost_duration'"
        ) from None


def load_instance(path: str | Path, *, verify: bool = False) -> LoadedTDInstance:
    """Load a MAMUT TD instance (``.vrp.json``) together with its ATF sidecar.

    ``verify`` turns on the artifact integrity check of mamut-routing-lib
    (``verify_sha256``): the sidecar file digests, plus ``atf_sha256``, the
    digest of the materialized arrival-time functions, re-derived by a full
    canonical re-serialization and compared against the value the instance
    file declares.

    **Verify on test runs, and on the first run over data you have not used
    before.** That is what catches a truncated download, a sidecar that does
    not belong to the instance sitting next to it, or a hand-edited artifact:
    without the check those failure modes are silent, and the solver happily
    optimizes the wrong travel times.

    **It is off by default here because this is the solver's hot path.**
    Materialization is deterministic, so once a pair of artifacts has been
    checked, re-checking it on every solve tells you nothing new. The check is
    not cheap at scale: the canonical re-serialization costs about 80 seconds
    and roughly 10 GB of extra peak memory on a one-million-arc instance with
    114 million breakpoints (n = 1000), which used to show up as a mysterious
    slow loading phase before the search even started.
    """
    return load_td_instance(path, verify_sha256=verify)


def to_core(
    loaded: LoadedTDInstance, objective_function: object = "Duration"
) -> _core.Instance:
    """Build the compiled-core instance from a loaded MAMUT TD instance.

    Float coercions mirror the checker exactly (``_vertex_time_window`` and the
    ``service_time`` cast in ``compute_route_ready_time_function``), so core
    route pricing is bit-identical to ``check_td_solution``.

    ``objective_function`` selects what the core's fold prices: under
    ``Duration`` (the default) ``fixed_route_cost`` stays 0 even when the
    instance carries a ``fleet_fixed_cost`` (objectives are orthogonal scoring
    contracts; F is ignored, exactly like the checker). Under
    ``FleetCostDuration`` it is read from ``instance.fleet_fixed_cost``
    (``None`` → 0; ``solve()`` rejects the missing field before it gets here).
    """
    instance = loaded.instance
    atfs = loaded.atfs
    objective = canonical_objective(objective_function)
    fixed_route_cost = 0.0
    if objective == "FleetCostDuration":
        fixed_route_cost = float(getattr(instance, "fleet_fixed_cost", None) or 0.0)
    time_windows = getattr(instance, "time_windows", None)
    if time_windows is not None:
        time_windows = [(float(earliest), float(latest)) for earliest, latest in time_windows]
    return _core.Instance(
        num_customers=instance.num_customers,
        num_vehicles=instance.num_vehicles,
        vehicle_capacity=instance.vehicle_capacity,
        horizon=(float(atfs.horizon[0]), float(atfs.horizon[1])),
        time_windows=time_windows,
        demands=list(instance.demands),
        service_times=[float(s) for s in instance.service_times],
        arcs=[(i, j, f.xs, f.ys) for (i, j), f in atfs.arcs.items()],
        fixed_route_cost=fixed_route_cost,
    )
