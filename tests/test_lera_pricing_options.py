"""The production pricing configuration and exact-closure diagnostic."""

import os

import pytest
from mamut_routing_lib.td import load_td_instance

from conftest import family_instances

R201_25 = [p for p in family_instances("TDVRPTW", "Dabia2013", ["n=25"]) if p.name == "R201.vrp.json"]
R201_25_OPTIMUM = 7932.168490728329


def _solve(instance_path, **kwargs):
    from kayros._bpc import solve_duration

    return solve_duration(load_td_instance(instance_path), time_limit_s=120.0, cut_limit=500, **kwargs)


@pytest.mark.parametrize("instance_path", R201_25, ids=lambda p: "R201-n25")
def test_production_pricing_ignores_historical_environment(instance_path, monkeypatch) -> None:
    monkeypatch.setenv("KAYROS_BPC_NEAR_REL", "0")
    monkeypatch.setenv("KAYROS_BPC_PACKED", "2")
    monkeypatch.setenv("KAYROS_STEP_EXACT", "external")
    result = _solve(instance_path)
    assert not result["stepwise_atfs"]
    assert os.environ["KAYROS_STEP_EXACT"] == "external"
    options = result["effective_algorithm_settings"]["pricing_options"]
    assert options["near_rel"] == pytest.approx(0.10)
    assert options["near_limit"] == 3000
    assert options["heuristic_cut_blind"]
    assert options["packed_filter"]
    assert options["merge_prefilter"]
    assert options["last_arc_merge_iterate"]
    assert options["verify_final_mask"] == 0
    assert result["exact_log"]["status"] == "Optimum"
    assert result["value"] == pytest.approx(R201_25_OPTIMUM, rel=1e-9)
    assert result["closure_census"]["cg_unresolved_final_pricing"] == 0
