"""Load MAMUT TD artifacts and copy their binary64 operands into the compiled instance.

Vertices 1 through n are customers and vertex 0 is the depot. Each directed arc carries an arrival-time function over the instance horizon."""

from __future__ import annotations

from pathlib import Path

from mamut_routing_lib.td import LoadedTDInstance, load_td_instance

from kayros import _core

# Accepted spellings of the two MAMUT objective values.
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
    """Load a MAMUT TD instance and its ATF sidecar.

    With verify=True the loader checks the files and the digest of the materialized arrival-time functions against the instance declaration. This requires an additional pass over the arc data."""
    return load_td_instance(path, verify_sha256=verify)


def to_core(
    loaded: LoadedTDInstance, objective_function: object = "Duration"
) -> _core.Instance:
    """Build the native instance from the materialized MAMUT operands.

    The search fold uses binary64. The native incumbent evaluator prepares the original arc knots for exact GMP pricing. The Python library checks the final solution. Duration sets the per-route charge to zero. FleetCostDuration takes the charge from the instance."""
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
