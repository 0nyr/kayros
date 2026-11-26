"""Exact incumbent decisions, restart state, and bounded final-only recovery."""
from fractions import Fraction
from types import SimpleNamespace

import pytest
from mamut_routing_lib.checker import SolutionCheckStatus
from mamut_routing_lib.exact_types import ExactRouteError

import kayros
from kayros.solve import _Acceptance

from conftest import family_instances, instance_path, require_benchmarks


def invalid():
    return ExactRouteError(SolutionCheckStatus.TIME_WINDOW_VIOLATED, "forced refusal")


def incumbent(value, i=0, squeeze=False):
    return SimpleNamespace(value=value, seconds=0.0, iteration=i, origin=2, from_squeeze=squeeze)


def test_fraction_gate_and_squeeze_rejections():
    policy = _Acceptance(None, "Duration", kayros.Params(), None)
    low = Fraction(1)
    high = low + Fraction(1, 2**60)
    assert float(low) == float(high)
    costs = iter([invalid(), high, high, low])

    def evaluate(routes):
        cost = next(costs)
        if isinstance(cost, Exception):
            raise cost
        return cost

    policy.proposal_cost = evaluate
    assert not policy.accept([[1]], True)
    assert policy.accept([[2]], False)
    policy.publish(incumbent(1.1), [[2]])
    assert not policy.accept([[3]], True)
    assert policy.accept([[4]], False)
    policy.publish(incumbent(1.0), [[4]])
    assert [e.exact_cost for e in policy.events] == [high, low]
    assert [e.value for e in policy.events] == [1.0, 1.0]
    assert policy.stats["infeasible"] == policy.stats["nonimproving"] == 1
    assert policy.stats["squeeze_rejections"] == 2


@pytest.mark.parametrize("capacity,expected", [(0, [1]), (2, [1, 2, 3]), (6, list(range(1, 8)))])
def test_final_only_retention_and_exhaustion(capacity, expected, caplog):
    params = kayros.Params(acceptance_mode="final_only", incumbent_history_size=capacity)
    policy = _Acceptance(None, "Duration", params, None)
    # Emit the same solution twice, with routes in different order.
    for value in range(10, 0, -1):
        routes = [[value], [11]]
        inc = incumbent(value, squeeze=value == 1)
        policy.publish(incumbent(value + 0.2), list(reversed(routes)))
        policy.publish(inc, routes)
    calls = []

    def reject(routes):
        calls.append(routes[0][0])
        raise invalid()

    policy.prepared = SimpleNamespace(evaluate=reject)
    result = SimpleNamespace(value=1, routes=[[1], [11]], incumbents=[inc])
    with pytest.raises(kayros.InfeasibleError, match=f"all {capacity + 1} retained"):
        policy.finish(result)
    assert calls == expected
    assert policy.stats["squeeze_rejections"] == 1
    assert policy.stats["final_rejections"] == capacity + 1
    assert policy.stats["fallback_attempts"] == capacity
    assert "rejected candidate 1" in caplog.text


def test_real_search_restarts_only_from_an_accepted_incumbent():
    from kayros.io import to_core, load_instance

    require_benchmarks()
    path = next(p for p in family_instances("TDVRPTW", "Dabia2013", ["n=25"]) if p.name == "R101.vrp.json")
    core = to_core(load_instance(path))
    params = kayros.Params(ils_max_iterations=12, restart_no_improvement=2)._to_ils_core()
    proposals = []
    published = []

    def gate(routes, squeezed):
        proposals.append(routes)
        return len(proposals) == 2  # refuse the raw greedy seed, then every later improvement

    result = kayros._core.solve_ils(core, params, 7, 0.0, lambda i, r: published.append(r), accept_incumbent=gate)
    assert len(proposals) > 2
    assert result.restarts > 0
    assert published == [proposals[1]]
    assert result.routes == proposals[1]
    assert result.value == kayros._core.solution_cost(core, result.routes)


@pytest.mark.parametrize("search_mode", ["binary64", "exact"])
@pytest.mark.parametrize("acceptance_mode", ["each_incumbent", "final_only"])
@pytest.mark.parametrize("receiver,donor,expected", [
    pytest.param([3, 13, 16, 5], [2, 12], 12798, id="before"),
    pytest.param([2, 3, 13, 16, 5], [12], 12778, id="after"),
])
def test_rifki13_relocation_checked_cost(receiver, donor, expected, search_mode, acceptance_mode):
    from kayros.io import load_instance
    from mamut_routing_lib.exact_td import prepare_td

    path = instance_path("TDVRPTW", "Rifki2020", "n=20", "Rifki-13")
    if not path.is_file():
        pytest.skip("Rifki-13 n=20 is not installed")
    loaded = load_instance(path, verify=True)
    singletons = [[c] for c in range(1, 21) if c not in receiver and c not in donor]
    routes = [receiver, donor, *singletons]
    oracle = prepare_td(loaded.instance, loaded.atfs, objective_function="Duration", backend="python")
    checked = oracle.evaluate(routes)
    assert checked.cost == expected

    # Let the deadline expire at initialization to exercise publication of this particular seed.
    events = []
    params = kayros.Params(search_mode=search_mode, acceptance_mode=acceptance_mode)
    result = kayros.solve(loaded, params, initial_routes=routes, time_limit=1e-12,
                          on_incumbent=lambda event, published: events.append((event, published)))
    assert result.routes == routes
    assert result.exact_cost == checked.cost and result.cost == checked.display_cost
    witness = dict(zip(checked.routes, checked.route_evaluations))
    assert result.exact_route_durations == [witness[tuple(r)].duration for r in result.routes]
    assert result.exact_route_departures == [witness[tuple(r)].departure_time for r in result.routes]

    (event, published), = events
    assert published == routes and event.origin == "warm_start"
    if search_mode == "binary64" and acceptance_mode == "final_only":
        assert event.exact_cost is None and event.value == event.search_value
    else:
        assert event.exact_cost == checked.cost and event.value == checked.display_cost
    if search_mode == "exact":
        assert event.search_value is None and result.search_value is None


def test_public_api_recovers_a_prior_incumbent(monkeypatch, caplog):
    from importlib import import_module
    from mamut_routing_lib.exact import prepare_exact

    module = import_module("kayros.solve")
    calls = []

    def prepare(instance, **kwargs):
        prepared = prepare_exact(instance, **kwargs)

        def evaluate(routes):
            calls.append([list(r) for r in routes])
            if len(calls) == 1:
                raise invalid()
            return prepared.evaluate(routes)

        return SimpleNamespace(evaluate=evaluate)

    monkeypatch.setattr(module, "prepare_exact", prepare)
    require_benchmarks()
    path = next(p for p in family_instances("TDVRPTW", "Dabia2013", ["n=25"]) if p.name == "R101.vrp.json")
    events = []
    params = kayros.Params(ils_max_iterations=25, acceptance_mode="final_only")
    solution = kayros.solve(path, params, seed=5, on_incumbent=lambda i,r: events.append((i, r)))
    assert len(calls) == 2
    assert calls == [events[-1][1], events[-2][1]]
    assert solution.routes == calls[1]
    assert solution.search_value == events[-2][0].value
    assert solution.cost == float(solution.exact_cost)
    assert all(i.exact_cost is None for i in solution.incumbents)
    assert solution.acceptance_stats["recovered"] == solution.acceptance_stats["final_rejections"] == 1
    assert "Recovered a checked incumbent" in caplog.text


@pytest.mark.parametrize("kwargs", [{"incumbent_history_size": -1}, {"incumbent_history_size": 1.5}, {"acceptance_mode": "off"}])
def test_invalid_acceptance_options(kwargs):
    with pytest.raises(ValueError):
        _Acceptance(None, "Duration", kayros.Params(**kwargs), None)


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
@pytest.mark.parametrize("warm", [False, True])
def test_finite_search_value_with_overflowing_exact_display(mode, warm):
    from pathlib import Path
    from sys import float_info
    from mamut_routing_lib.exact_td import prepare_td
    from mamut_routing_lib.td import InstanceATFs, LoadedTDInstance, NDCPWLF
    from mamut_routing_lib.td.models import BenchmarkInstanceTDVRPTW

    # Multiplying the fixed charge by five rounds down to max binary64.
    # The total duration fits inside half an ULP, but the exact sum crosses it.
    ulp = 2**971
    duration = float(ulp // 16)
    horizon = float(ulp)
    charge = float.fromhex("0x1.9999999999999p+1021")
    name = "finite-search-overflowing-exact"
    instance = BenchmarkInstanceTDVRPTW(
        instance_name=name, instance_origin="Blauth2024", benchmark_name="Blauth2024",
        num_customers=5, num_vehicles=5, vehicle_capacity=1, coordinates=[(i, 0) for i in range(6)],
        demands=[0, 1, 1, 1, 1, 1], service_times=[0] * 6, time_windows=[(0, horizon)] + [(0, 0)] * 5,
        horizon=(0, horizon), fleet_fixed_cost=charge, td={"model": "atf-ndcpwlf", "atf_path": "toy.atf.json"},
    )
    arcs = {}
    for tail in range(6):
        for head in range(6):
            if tail != head:
                travel = duration if head == 0 else 0
                arcs[tail, head] = NDCPWLF([0, horizon], [travel, horizon + travel])
    atfs = InstanceATFs(name, "Blauth2024", (0, horizon), 5, arcs)
    loaded = LoadedTDInstance(instance, atfs, Path("synthetic.vrp.json"), None)
    routes = [[i] for i in range(1, 6)]
    checked = prepare_td(instance, atfs, objective_function="FleetCostDuration", backend="python").evaluate(routes)
    assert checked.cost == Fraction(float_info.max) + 9 * (ulp // 16)
    assert checked.display_cost is None
    events = []
    result = kayros.solve(loaded, kayros.Params(acceptance_mode=mode, objective="FleetCostDuration", ils_max_iterations=1),
                          initial_routes=routes if warm else None, on_incumbent=lambda e, r: events.append((e, r)))
    assert result.routes == routes and result.exact_cost == checked.cost
    assert result.search_value == float_info.max and result.cost is None
    (event, published), = events
    assert published == routes and event.origin == ("warm_start" if warm else "greedy")
    assert event.search_value == float_info.max
    if mode == "each_incumbent":
        assert event.exact_cost == checked.cost and event.value is None
    else:
        assert event.exact_cost is None and event.value == event.search_value
