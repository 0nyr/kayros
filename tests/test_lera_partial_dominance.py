"""Regression gate for the partial-dominance right-piece defect (ms61).

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
from mamut_routing_lib.td import compute_solution_cost, load_td_instance

from conftest import family_instances

RC107_50 = [p for p in family_instances("TDVRPTW", "Dabia2013", ["n=50"]) if p.name == "RC107.vrp.json"]
RC107_50_OPTIMUM = 10614.011179781175

# The pricing ladder of releases before 1.7.0, on which the defect was found.
LEGACY = {
    "KAYROS_BPC_NEAR_REL": "0",
    "KAYROS_BPC_HE_CUTBLIND": "0",
    "KAYROS_BPC_PACKED": "0",
    "KAYROS_BPC_MERGE_PREFILTER": "0",
    "KAYROS_BPC_LAM_ITERATE": "0",
}

SETTINGS = {
    "default": {},
    "legacy": dict(LEGACY),
    # The two trajectories that closed above the optimum before the fix.
    "legacy-dropback": {**LEGACY, "KAYROS_BPC_DROPBACK_LEVEL": "0"},
    "legacy-dropback-cutblind": {**LEGACY, "KAYROS_BPC_DROPBACK_LEVEL": "0", "KAYROS_BPC_HE_CUTBLIND": "1"},
    # Re-runs every empty final Exact pass without partial dominance: a column
    # found there is a column partial dominance lost.
    "legacy-dropback-verified": {**LEGACY, "KAYROS_BPC_DROPBACK_LEVEL": "0", "KAYROS_BPC_VERIFY_FINAL": "16"},
    "default-verified": {"KAYROS_BPC_VERIFY_FINAL": "16"},
}


@pytest.mark.parametrize("instance_path", RC107_50, ids=lambda p: "RC107-n50")
@pytest.mark.parametrize("setting", sorted(SETTINGS))
def test_rc107_n50_closes_at_the_optimum_under_every_ladder(instance_path, setting, monkeypatch) -> None:
    from kayros.lera import routes_to_mamut, solve_duration

    for name, value in SETTINGS[setting].items():
        monkeypatch.setenv(name, value)
    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit_s=300.0, cut_limit=500)
    assert result["exact_log"]["status"] == "Optimum"
    assert result["closure_census"]["cg_unresolved_final_pricing"] == 0
    assert result["value"] == pytest.approx(RC107_50_OPTIMUM, rel=1e-9)
    routes = [r[1:-1] for r in routes_to_mamut(result["routes"], loaded.instance.num_customers)]
    assert result["value"] == compute_solution_cost(loaded.instance, loaded.atfs, routes)
    counters = result.get("pricing_counters", {})
    assert counters.get("verify_final_catches", 0) == 0
