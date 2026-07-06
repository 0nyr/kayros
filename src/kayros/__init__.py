"""KAYROS — exact & anytime solver for duration-minimization time-dependent vehicle routing (TDVRPTW / TDVRP).

Benchmarked on the canonical MAMUT-routing TD families; solutions are always
priced by the reference checker (``mamut_routing_lib.td.check_td_solution``).
"""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("kayros")
except PackageNotFoundError:  # editable/source tree without metadata
    __version__ = "0.0.0"

from kayros import _core  # noqa: F401  (compiled extension smoke-import)
from kayros.solve import (  # noqa: F401
    Incumbent,
    IncumbentHook,
    InfeasibleError,
    KayrosError,
    Params,
    Solution,
    solve,
)

__all__ = [
    "__version__",
    "solve",
    "Params",
    "Solution",
    "Incumbent",
    "IncumbentHook",
    "KayrosError",
    "InfeasibleError",
]
