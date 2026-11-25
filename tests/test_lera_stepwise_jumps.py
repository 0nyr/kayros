"""Regression gates for the value-jump defect repaired in 1.8.0 (``cpp/lera/NOTICE.md`` item 18).

On stepwise travel times every value jump lost its vertical piece in the JSON
round trip of the service-waiting preprocessing, and the inverse of the arrival
function bridged the bare jump with a chord. Departure functions were then too
late after each travel-time up-step and labels were under-estimated. Two things
followed. The labeling priced as negative routes whose checker reduced cost is
positive (phantom columns), and an under-estimated label could dominate a label
that is better in true arithmetic, so the Exact level missed improving columns.

Witness: Rifki2020 TDVRPTW n=10 ``Rifki-30``, cold start, symmetric labeling,
no cut. Releases 1.5.1 to 1.7.1 report a root LP value of 4194.142857142854. An
exhaustive enumeration of the 40,074 checker-feasible routes of the instance,
made with the primitives of mamut-routing-lib only, gives 4193.857142857141.

The other gates run instances on which a 1.7.1 Exact pass ended empty with an
improving column outstanding (Rifki-16 and Rifki-18 at n=10, Rifki-17 at n=50,
all in symmetric mode) or returned phantom columns (Rifki-23 at n=40). They need
the Rifki2020 trees of the benchmark store and are skipped without them.
"""

import json

import pytest
from mamut_routing_lib.td import load_td_instance

from conftest import family_instances
from exact_reference import assert_value_close, exact_duration_cost

ENUMERATED_ROOT_LP = 4193.857142857141

# Counters of ``pricing_integrity`` that a sound pricing leaves at 0.
INTEGRITY_COUNTERS = (
    "empty_negative_priced_routes",
    "phantom_columns",
    "phantom_duplicates",
    "labeling_below_checker_columns",
    "route_duration_mismatches",
    "verify_final_negative_columns",
    "negative_duplicates",
)


def _instance(problem_type: str, family: str, size_dir: str, name: str) -> list:
    return [p for p in family_instances(problem_type, family, [size_dir]) if p.name == f"{name}.vrp.json"]


RIFKI30_10 = _instance("TDVRPTW", "Rifki2020", "n=10", "Rifki-30")
# Historical value-jump witnesses and their independent objective values.
CLOSURES = [
    ("n=10", "Rifki-16", 5321.0),
    ("n=10", "Rifki-18", 4025.0),
    ("n=50", "Rifki-17", 21319.0),
    ("n=40", "Rifki-23", 16109.0),
]
CLOSURE_CASES = [
    pytest.param(path, optimum, id=f"{name}-{size}")
    for size, name, optimum in CLOSURES
    for path in _instance("TDVRPTW", "Rifki2020", size, name)
]
JUMP_FREE = _instance("TDVRPTW", "Dabia2013", "n=25", "C205")


def _assert_clean_pricing(result: dict) -> None:
    integrity = result["pricing_integrity"]
    for counter in INTEGRITY_COUNTERS:
        assert integrity[counter] == 0, f"{counter} = {integrity[counter]}"
    assert result["checker_infeasible_columns"] == 0


@pytest.mark.parametrize("instance_path", RIFKI30_10, ids=lambda p: "Rifki-30-n10")
def test_rifki30_n10_root_lp_equals_the_enumerated_value(instance_path) -> None:
    from kayros._bpc import solve_duration

    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit=120.0, cut_limit=0, node_limit=0)
    assert result["stepwise_atfs"] is True
    assert result["exact_log"]["root_lp_value"] == pytest.approx(ENUMERATED_ROOT_LP, abs=1e-6)
    # Under true duals an existing column never prices negative again.
    assert result["closure_census"]["duplicate_columns"] == 0
    _assert_clean_pricing(result)


@pytest.mark.parametrize("instance_path, optimum", CLOSURE_CASES)
def test_stepwise_closures_leave_no_column_behind(instance_path, optimum) -> None:
    from kayros._bpc import routes_to_mamut, solve_duration

    # Every empty final Exact pass is run again without partial dominance and
    # without label correction. On 1.7.1 that pass added columns on each of
    # these instances, improving ones on the three symmetric cases.
    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit=300.0, verify_final_mask=16)
    assert result["exact_log"]["status"] == "Optimum"
    assert result["value"] == pytest.approx(optimum, abs=1e-6)
    routes = [r[1:-1] for r in routes_to_mamut(result["routes"], loaded.instance.num_customers)]
    # M2: becomes exact equality once priced columns are evaluated exactly
    assert_value_close(result["value"], exact_duration_cost(loaded.instance, loaded.atfs, routes))
    counters = result["pricing_counters"]
    assert counters["verify_final_runs"] > 0
    assert counters["verify_final_catches"] == 0
    assert result["closure_census"]["duplicate_columns"] == 0
    assert result["closure_conditions"]["all_hold"] is True
    _assert_clean_pricing(result)


@pytest.mark.parametrize("instance_path", JUMP_FREE, ids=lambda p: "C205-n25")
def test_jump_free_solve_reports_clean_pricing_and_covered_closures(instance_path) -> None:
    from kayros._bpc import solve_duration

    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit=120.0)
    stored = json.loads(instance_path.with_name("C205.bks.Duration.json").read_text())
    assert result["stepwise_atfs"] is False
    assert result["exact_log"]["status"] == "Optimum"
    assert result["value"] == pytest.approx(stored["cost"], rel=1e-9)
    conditions = result["closure_conditions"]
    for name in ("status_optimum", "all_closures_certified", "no_unpriced_closure", "no_checker_infeasible_column", "no_empty_negative_priced_route", "all_hold"):
        assert conditions[name] is True, name
    # Near-zero columns ride along with productive passes on jump-free solves,
    # so duplicates are legitimate there. Phantoms are not.
    _assert_clean_pricing(result)
