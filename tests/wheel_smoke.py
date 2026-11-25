"""Exercise both public solves and the native exact checker in an installed wheel."""

import sys
from fractions import Fraction
from pathlib import Path

import kayros
from kayros import _core
from kayros.exact import solve_exact
from kayros.io import to_core
from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.td import compute_solution_cost, load_td_instance


def main(root: str) -> int:
    path = Path(root) / "benchmarks/Dabia2013/TDVRPTW/n=25/C101.vrp.json"
    if not path.is_file():
        sys.exit(f"missing wheel smoke fixture: {path}")
    loaded = load_td_instance(str(path))
    heuristic = kayros.solve(str(path), time_limit=2.0, seed=1, verify=True)
    result = solve_exact(loaded, time_limit=300.0)
    print(f"kayros {kayros.__version__}: ILS {heuristic.cost}, BPC {result.cost} ({result.termination_reason})")
    if result.status != "feasible" or result.termination_reason != "search_closed" or result.certified_dual_bound is not None:
        return 1
    if result.raw["checker_infeasible_columns"] != 0 or not result.routes:
        return 1
    checked = compute_solution_cost(loaded.instance, loaded.atfs, result.routes, ObjectiveFunction.DURATION, backend="python")
    pricer = _core.ExactRoutePricer(to_core(loaded))
    native = pricer.solution_cost(result.routes)
    if native is None or Fraction(*native) != checked:
        return 1
    return 0 if checked == result.exact_cost and result.cost == float(checked) and checked <= heuristic.exact_cost else 1


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: python tests/wheel_smoke.py MAMUT_ROUTING_ROOT")
    sys.exit(main(sys.argv[1]))
