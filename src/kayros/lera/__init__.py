"""Optional CPLEX-based exact BPC component (Lera-Romero et al. 2020).

Not shipped in the default build: it requires a local CPLEX installation and a
source build with ``-DKAYROS_WITH_LERA=ON``. Lands at milestone M3.7.
"""

raise ImportError(
    "kayros.lera is not available in this build. The exact BPC component "
    "requires CPLEX and a source build with -DKAYROS_WITH_LERA=ON; it is "
    "planned for a future kayros release (milestone M3.7)."
)
