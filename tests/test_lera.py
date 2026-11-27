"""Public branch-price-and-cut result and exact-family incumbent checks."""

import copy
from fractions import Fraction

import pytest
from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.td import compute_solution_cost, load_td_instance

from kayros.exact import ExactResult, solve_exact
from conftest import family_instances


def named(family, size, name):
    return [path for path in family_instances("TDVRPTW", family, [size]) if path.name == name]


C101 = named("Dabia2013", "n=25", "C101.vrp.json")
RIFKI = named("Rifki2020", "n=10", "Rifki-1.vrp.json")


def test_exact_backend_is_available():
    from kayros import _lera
    assert _lera.LP_BACKEND in ("HiGHS", "CPLEX")


@pytest.mark.parametrize("instance_path", C101 + RIFKI, ids=lambda p: p.stem)
def test_cold_result_and_callbacks(instance_path):
    events = []
    result = solve_exact(instance_path, verify=True, time_limit=120, on_incumbent=lambda event, routes: events.append((event, routes)))
    loaded = load_td_instance(instance_path)
    assert isinstance(result, ExactResult)
    # The ledger certified the run and the final gate passed (M2 step 5).
    assert result.status == "optimal" and result.termination_reason == "search_closed"
    assert result.raw["exact_log"]["status"] == "Optimum" and result.raw["ledger"]["certified"]
    assert result.certified_dual_bound == result.exact_cost and result.certified_gap == 0 and result.objective == "Duration"
    assert result.certified_dual_bound_binary64 == float(result.exact_cost) and result.certificate_gate is None
    assert result.routes == result.raw["ledger"]["incumbent"]
    assert result.reported_best_bound == result.raw["exact_log"].get("best_bound")
    assert result.pricing_integrity == result.raw["pricing_integrity"]
    assert result.closure_conditions == result.raw["closure_conditions"]
    assert [r["path"] for r in result.raw["routes"]] == result.routes
    assert result.raw["stepwise_atfs"] == ("Rifki" in instance_path.name)
    assert events and result.incumbents == [event for event, _ in events]
    costs = []
    for event, routes in events:
        assert sorted(c for route in routes for c in route) == list(range(1, loaded.instance.num_customers + 1))
        value = compute_solution_cost(loaded.instance, loaded.atfs, routes, ObjectiveFunction.DURATION)
        assert event.value == float(value) and event.exact_cost == value
        costs.append(value)
    assert all(b < a for a, b in zip(costs, costs[1:]))
    assert result.routes == events[-1][1]
    assert result.cost == float(result.exact_cost) and result.exact_cost == costs[-1]
    assert (result.exact_evaluation["objective"], result.exact_evaluation["profile"]) == ("Duration", "td-binary64-exact-v1")


@pytest.mark.parametrize("instance_path", C101, ids=["C101"])
def test_winning_warm_start_keeps_its_routes(instance_path):
    cold = solve_exact(instance_path, time_limit=120)
    events = []
    warm = solve_exact(load_td_instance(instance_path), time_limit=120, initial_routes=cold.routes,
                       on_incumbent=lambda event, routes: events.append((event, routes)))
    # The warm start is the incumbent of the ledger, whose routes the native record now returns.
    assert warm.raw["exact_log"]["status"] == "Optimum"
    assert [r["path"] for r in warm.raw["routes"]] == cold.routes
    assert warm.routes == cold.routes and warm.exact_cost == cold.exact_cost
    assert warm.incumbents[0].origin == "warm_start" and events[0][1] == cold.routes
    assert warm.status == "optimal" and warm.certified_dual_bound == cold.exact_cost


@pytest.mark.parametrize("instance_path", C101, ids=["C101"])
def test_invalid_or_depot_delimited_warm_start_does_not_start_the_solver(instance_path, monkeypatch):
    from kayros import _bpc
    monkeypatch.setattr(_bpc, "solve_duration", lambda *a, **kw: pytest.fail("invalid seed reached BPC"))
    with pytest.raises(ValueError, match="invalid initial_routes"):
        solve_exact(instance_path, initial_routes=[[1]])
    with pytest.raises(ValueError, match="invalid initial_routes"):
        solve_exact(instance_path, initial_routes=[[0, 1, 0]])
    with pytest.raises(ValueError, match="verify=True"):
        solve_exact(load_td_instance(instance_path), verify=True)


@pytest.mark.parametrize("instance_path", RIFKI, ids=["Rifki-1"])
def test_raw_closure_cannot_certify_an_invalid_incumbent(instance_path, monkeypatch):
    from kayros import _bpc
    loaded = load_td_instance(instance_path)
    bad = [{"path": [0, 1, loaded.instance.num_customers + 1]}]

    def raw(loaded, *, on_incumbent, **kwargs):
        event = {"routes": bad, "time": 1.0, "origin": "lp", "value": 1.0}
        on_incumbent(event)
        return {"routes": bad, "value": 1.0, "incumbents": [event],
                "exact_log": {"status": "Optimum", "time": 1.0, "best_bound": 1.0},
                "closure_conditions": {"all_hold": True}, "pricing_integrity": {}}

    monkeypatch.setattr(_bpc, "solve_duration", raw)
    result = solve_exact(loaded, on_incumbent=lambda *args: pytest.fail("published invalid incumbent"))
    # A native Optimum without a certified ledger is no claim.
    assert result.status == "no_incumbent" and result.termination_reason == "certificate_gate_failed"
    assert result.certificate_gate == "ledger_not_certified"
    assert result.routes == [] and not result.incumbents and result.rejected_incumbents == 1
    assert result.cost is result.exact_cost is result.certified_dual_bound is None


@pytest.mark.parametrize("instance_path", C101, ids=["C101"])
def test_memory_limit_is_independent_of_incumbent_status(instance_path):
    result = solve_exact(instance_path, time_limit=120, memory_limit_mb=1)
    assert result.termination_reason == "memory_limit"
    assert result.raw["memory"]["limit_reached"]
    assert result.certified_dual_bound is None
    if result.routes:
        assert result.status == "feasible" and isinstance(result.exact_cost, Fraction)
    else:
        assert result.status == "no_incumbent" and result.exact_cost is None


@pytest.mark.parametrize("instance_path", C101, ids=["C101"])
@pytest.mark.parametrize(("limits", "expected"), [
    ({"time_limit": 0.0}, "time_limit"),
    ({"memory_limit_mb": 1}, "memory_limit"),
])
def test_checked_warm_start_survives_limits(instance_path, limits, expected):
    seed = solve_exact(instance_path, time_limit=120)
    stopped = solve_exact(instance_path, initial_routes=seed.routes, **limits)
    assert stopped.status == "feasible" and stopped.termination_reason == expected
    assert stopped.routes == seed.routes and stopped.exact_cost == seed.exact_cost
    assert stopped.incumbents[0].origin == "warm_start"
    assert stopped.certified_dual_bound is stopped.certified_gap is None


@pytest.mark.parametrize("instance_path", C101, ids=["C101"])
def test_native_closure_with_no_routes_is_no_incumbent(instance_path, monkeypatch):
    from kayros import _bpc
    monkeypatch.setattr(_bpc, "solve_duration", lambda *args, **kwargs: {
        "routes": [], "incumbents": [], "value": 0.0,
        "exact_log": {"status": "Optimum", "best_bound": 0.0, "time": 0.1},
        "closure_conditions": {"all_hold": True}, "pricing_integrity": {},
    })
    result = solve_exact(instance_path)
    assert result.status == "no_incumbent" and result.termination_reason == "certificate_gate_failed"
    assert result.routes == [] and result.incumbents == []
    assert result.cost is result.exact_cost is result.certified_dual_bound is result.certified_gap is None
    assert result.reported_best_bound == 0.0 and result.closure_conditions["all_hold"]


@pytest.mark.parametrize("instance_path", C101, ids=["C101"])
def test_unresolved_lp_keeps_checked_warm_start_without_claim(instance_path, monkeypatch):
    seed = solve_exact(instance_path, time_limit=120)
    from kayros import _bpc
    monkeypatch.setattr(_bpc, "solve_duration", lambda *args, **kwargs: {
        "routes": [], "incumbents": [], "value": 0.0,
        "exact_log": {"status": "Unresolved", "time": 0.1},
        "closure_conditions": {"all_hold": False}, "pricing_integrity": {},
    })
    result = solve_exact(instance_path, initial_routes=seed.routes)
    assert result.status == "feasible" and result.termination_reason == "lp_unresolved"
    assert result.routes == seed.routes and result.exact_cost == seed.exact_cost
    assert result.reported_best_bound is None and result.certified_dual_bound is None


@pytest.mark.parametrize("instance_path", RIFKI, ids=["Rifki-1"])
def test_ledger_record_sidecar_and_header_check(instance_path, tmp_path):
    from kayros.exact import _header_mismatch
    loaded = load_td_instance(instance_path)
    path = tmp_path / "rifki.record"
    result = solve_exact(loaded, time_limit=120, record_path=path)
    assert result.status == "optimal" and result.record_path == str(path)
    record = path.read_text()
    assert record == result.raw["ledger"]["record"] and record.startswith("kayros-ledger-record 1\n")
    assert _header_mismatch(record, loaded) is None
    lines = record.split("\n")
    capacity = next(k for k, line in enumerate(lines) if line.startswith("capacity "))
    wrong = list(lines)
    wrong[capacity] = "capacity " + str(int(lines[capacity].split()[1]) + 1)
    assert _header_mismatch("\n".join(wrong), loaded) == "capacity"
    arc = next(k for k, line in enumerate(lines) if line.startswith("arc "))
    tokens = lines[arc].split()
    tokens[-1] = float(float.fromhex(tokens[-1]) + 1).hex()
    wrong = list(lines)
    wrong[arc] = " ".join(tokens)
    assert _header_mismatch("\n".join(wrong), loaded) == "arcs"
    assert _header_mismatch("garbage", loaded).startswith("unreadable header")


@pytest.mark.parametrize("instance_path", RIFKI, ids=["Rifki-1"])
def test_final_gate_refuses_a_certified_ledger_that_disagrees_with_the_checker(instance_path, monkeypatch):
    from kayros import _bpc
    loaded = load_td_instance(instance_path)
    real = _bpc.solve_duration(loaded, time_limit=120)
    assert real["ledger"]["certified"] and real["exact_log"]["status"] == "Optimum"
    for field, value in (("U", str(Fraction(real["ledger"]["U"]) - Fraction(1, 3))), ("record", "kayros-ledger-record 1\nfleet none\n")):
        tampered = copy.deepcopy(real)
        tampered["ledger"][field] = value
        monkeypatch.setattr(_bpc, "solve_duration", lambda *args, _t=tampered, **kwargs: _t)
        result = solve_exact(loaded)
        assert result.status == "feasible" and result.certified_dual_bound is None
        assert result.termination_reason == "certificate_gate_failed" and result.certificate_gate
