"""Time-dependent vehicle routing under Duration and FleetCostDuration objectives.

The public heuristic returns route costs under the MAMUT exact evaluation profile.
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
