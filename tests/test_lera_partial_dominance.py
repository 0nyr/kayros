"""Regression gate for the partial-dominance right-piece defect repaired in 1.7.0.

``PWLDominationFunction::DominatePieces`` splits a piece when the dominated
interval lies strictly inside it. Upstream rebuilt the surviving right part with
``p1.image.right``, the maximum of the piece, as its value at the right end. On a
decreasing piece that over-estimates the label, and a later comparison could then
erase an undominated part of it: the Exact pricing level missed columns and a
node bound could exceed the optimum.

Reproducer: Dabia2013 n=50 RC107, on the pricing ladder of releases before 1.7.0.
With that ladder restarted at the first heuristic level after each productive
Exact pass (``KAYROS_BPC_DROPBACK_LEVEL=0``), the pre-fix solver closed at the root
at 10632.144948 (10625.511174 with cut-blind heuristics added), above the optimum
10614.011180 that the other ladders prove. The switches only change the column-generation trajectory. They
are sound by construction, so every setting must prove the same optimum.
"""

import pytest
from mamut_routing_lib.td import load_td_instance

from conftest import family_instances
from exact_reference import assert_exact_value, exact_duration_cost

RC107_50 = [p for p in family_instances("TDVRPTW", "Dabia2013", ["n=50"]) if p.name == "RC107.vrp.json"]
RC107_50_OPTIMUM = 10614.011179781175

@pytest.mark.parametrize("instance_path", RC107_50, ids=lambda p: "RC107-n50")
@pytest.mark.parametrize("verify_final_mask", [0, 16], ids=["production", "verify-partial-dominance"])
def test_rc107_n50_closes_at_the_optimum(instance_path, verify_final_mask) -> None:
    from kayros._bpc import routes_to_mamut, solve_duration

    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit=300.0, cut_limit=500, verify_final_mask=verify_final_mask)
    assert result["exact_log"]["status"] == "Optimum"
    assert result["closure_census"]["cg_unresolved_final_pricing"] == 0
    assert result["value"] == pytest.approx(RC107_50_OPTIMUM, rel=1e-9)
    routes = [r[1:-1] for r in routes_to_mamut(result["routes"], loaded.instance.num_customers)]
    assert_exact_value(result, exact_duration_cost(loaded.instance, loaded.atfs, routes))
    counters = result.get("pricing_counters", {})
    assert counters.get("verify_final_catches", 0) == 0
