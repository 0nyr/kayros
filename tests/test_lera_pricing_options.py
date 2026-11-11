"""Pricing options of the exact component (1.7.0): defaults, switches and overrides.

The options change how fast a node is solved, never what is proven: every setting
must certify the same optimum, and the result must record what was in force.
"""

import pytest
from mamut_routing_lib.td import load_td_instance

from conftest import family_instances

R201_25 = [p for p in family_instances("TDVRPTW", "Dabia2013", ["n=25"]) if p.name == "R201.vrp.json"]
R201_25_OPTIMUM = 7932.168490728329


def _solve(instance_path, **kwargs):
    from kayros.lera import solve_duration

    return solve_duration(load_td_instance(instance_path), time_limit_s=120.0, cut_limit=500, **kwargs)


@pytest.mark.parametrize("instance_path", R201_25, ids=lambda p: "R201-n25")
def test_default_pricing_options_are_recorded(instance_path) -> None:
    result = _solve(instance_path)
    options = result["effective_algorithm_settings"]["pricing_options"]
    assert options["near_rel"] == pytest.approx(0.10)
    assert options["near_limit"] == 3000
    assert options["he_cutblind"] == 1
    assert options["packed_filter"] == 1
    assert options["merge_prefilter"] is True
    assert options["lam_iterate"] is True
    # Experimental levers stay off.
    assert options["dropback_level"] == -1
    assert options["early_cuts"] == 0
    assert options["dssr"] == 0
    assert options["verify_final"] is False
    assert result["exact_log"]["status"] == "Optimum"
    assert result["value"] == pytest.approx(R201_25_OPTIMUM, rel=1e-9)
    assert result["closure_census"]["cg_unresolved_final_pricing"] == 0


@pytest.mark.parametrize("instance_path", R201_25, ids=lambda p: "R201-n25")
def test_options_do_not_change_the_optimum(instance_path) -> None:
    plain = _solve(instance_path, near_rel=0.0, heuristic_cut_blind=False)
    options = plain["effective_algorithm_settings"]["pricing_options"]
    assert options["near_rel"] == 0.0
    assert options["he_cutblind"] == 0
    assert plain["pricing_counters"]["near_columns_added"] == 0
    wide = _solve(instance_path, near_rel=0.5)
    for result in (plain, wide):
        assert result["exact_log"]["status"] == "Optimum"
        assert result["value"] == pytest.approx(R201_25_OPTIMUM, rel=1e-9)
        assert result["closure_census"]["cg_unresolved_final_pricing"] == 0


@pytest.mark.parametrize("instance_path", R201_25, ids=lambda p: "R201-n25")
def test_environment_overrides_the_parameter(instance_path, monkeypatch) -> None:
    monkeypatch.setenv("KAYROS_BPC_NEAR_REL", "0")
    monkeypatch.setenv("KAYROS_BPC_PACKED", "2")  # packed filter with its self-check
    result = _solve(instance_path, near_rel=0.3)
    options = result["effective_algorithm_settings"]["pricing_options"]
    assert options["near_rel"] == 0.0
    assert options["packed_filter"] == 2
    assert result["value"] == pytest.approx(R201_25_OPTIMUM, rel=1e-9)


def test_negative_near_rel_is_rejected() -> None:
    if not R201_25:
        pytest.skip("benchmarks not available")
    with pytest.raises(Exception):
        _solve(R201_25[0], near_rel=-1.0)
