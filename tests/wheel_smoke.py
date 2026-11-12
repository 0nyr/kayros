"""Smoke solve of an installed kayros wheel, used by the publish workflow.

Usage: python tests/wheel_smoke.py MAMUT_ROUTING_ROOT

Solves Dabia2013 C101 with 25 customers (TDVRPTW) twice: a short anytime
solve with ``kayros.solve`` and a cold exact solve with
``kayros.lera.solve_duration``. The exact solve must end with the status
``Optimum``, no checker-infeasible priced column, and a value that is not
above the anytime solution. The file is not named ``test_*.py``, so pytest
does not collect it.
"""

import sys
from pathlib import Path

import kayros
from kayros.lera import solve_duration
from mamut_routing_lib.td import load_td_instance


def main(root: str) -> int:
    path = Path(root) / "benchmarks/TDVRPTW/Dabia2013/n=25/C101.vrp.json"
    solution = kayros.solve(str(path), time_limit=2.0, seed=1, verify=True)
    print(f"kayros {kayros.__version__} anytime: {solution.duration} ({solution.num_routes} routes)")
    loaded = load_td_instance(str(path))
    result = solve_duration(loaded, time_limit_s=300.0)
    status = result["exact_log"]["status"]
    value = result.get("value")
    bad = result.get("checker_infeasible_columns")
    print(f"exact: status {status}, value {value}, checker_infeasible_columns {bad}")
    if status != "Optimum" or bad != 0 or value is None:
        return 1
    if value > solution.duration:
        return 1
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
