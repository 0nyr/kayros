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


def load_instance(path: str | Path) -> LoadedTDInstance:
    """Load a MAMUT TD instance (``.vrp.json``) together with its ATF sidecar."""
    return load_td_instance(path)


def to_core(loaded: LoadedTDInstance) -> _core.Instance:
    """Build the compiled-core instance from a loaded MAMUT TD instance.

    Float coercions mirror the checker exactly (``_vertex_time_window`` and the
    ``service_time`` cast in ``compute_route_ready_time_function``), so core
    route pricing is bit-identical to ``check_td_solution``.
    """
    instance = loaded.instance
    atfs = loaded.atfs
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
    )
