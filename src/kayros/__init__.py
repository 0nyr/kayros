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

__all__ = ["__version__"]
