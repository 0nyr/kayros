"""Public warm starts whose exact departure interval is lost by the binary64 search fold."""

import json
from dataclasses import replace
from fractions import Fraction
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace
from time import sleep

import pytest
from mamut_routing_lib.artifacts import load_bks
from mamut_routing_lib.td.bks import save_td_solution_as_bks_if_improved
from mamut_routing_lib.checker import SolutionCheckStatus
from mamut_routing_lib.exact_types import ExactRouteError
from mamut_routing_lib.exact_td import prepare_td
from mamut_routing_lib.td import (
    InstanceATFs, LoadedTDInstance, NDCPWLF, compute_atf_sha256, save_instance_atfs,
)
from mamut_routing_lib.td.models import BenchmarkInstanceTDVRPTW

import kayros
from kayros.io import load_instance, to_core


def tight_instance(*, direct_has_knot=False, return_from_one=1, fixed_charge=1, capacity=2):
    # Inverting arrival 16 on the direct outbound arc rounds 15 down by one ULP.
    length, start = 22.0, 15.0
    name = "tight-depot-opening"
    instance = BenchmarkInstanceTDVRPTW(
        instance_name=name, instance_origin="Blauth2024", benchmark_name="Blauth2024",
        num_customers=2, num_vehicles=2, vehicle_capacity=capacity, coordinates=[(0, 0), (1, 0), (2, 0)],
        demands=[0, 1, 1], service_times=[0, 0, 0], time_windows=[(start, length), (0, 16), (0, length)],
        horizon=(0, length), fleet_fixed_cost=fixed_charge, td={"model": "atf-ndcpwlf", "atf_path": "toy.atf.json"},
    )
    arcs = {}
    for tail in range(3):
        for head in range(3):
            if head == tail:
                continue
            omit_start = (tail, head) == (0, 1) and not direct_has_knot
            xs = [0.0, length] if omit_start else [0.0, start, length]
            travel = {(0, 2): 0, (1, 0): return_from_one}.get((tail, head), 1)
            arcs[tail, head] = NDCPWLF(xs, [x + travel for x in xs])
    atfs = InstanceATFs(name, "Blauth2024", (0.0, length), 2, arcs)
    return LoadedTDInstance(instance, atfs, Path("synthetic.vrp.json"), None)


def params(mode, cap=2):
    return kayros.Params(objective="FleetCostDuration", acceptance_mode=mode, k_cap=cap, ils_max_iterations=1)


def assert_seed_event(event, cost):
    assert (event.origin, event.iteration, event.value) == ("warm_start", 0, float(cost))
    assert event.exact_cost == cost and event.search_value is None


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
@pytest.mark.parametrize("cap", [1, 2])
def test_retry_finds_a_better_checked_route(mode, cap):
    loaded = tight_instance()
    seed = [[1], [2]]
    gold = prepare_td(loaded.instance, loaded.atfs, backend="python", objective_function="FleetCostDuration")
    original = gold.evaluate(seed)
    assert original.cost == 5
    assert [(w.duration, w.departure_time) for w in original.route_evaluations] == [(2, 15), (1, 15)]
    assert not to_core(loaded).evaluate_route([1])[0]
    events = []
    result = kayros.solve(loaded, params(mode, cap), initial_routes=seed, time_limit=0,
                          on_incumbent=lambda event, routes: events.append((event, routes)))
    assert result.routes == [[2, 1]]
    assert result.exact_cost == gold.evaluate(result.routes).cost == 3
    assert result.search_value is not None
    if cap == 2:
        assert events[0][1] == seed
        assert_seed_event(events[0][0], original.cost)
    assert events[-1][1] == result.routes
    assert events[-1][0].exact_cost == (result.exact_cost if mode == "each_incumbent" else None)
    assert [event.origin for event, _ in events] == (["warm_start", "greedy"] if cap == 2 else ["greedy"])
    assert all(previous.seconds <= current.seconds for (previous, _), (current, _) in zip(events, events[1:]))
    assert result.k_stats["k_best_max"] == cap and result.k_stats["k_best_min"] == 1
    assert result.k_cap_stats["first_capped_work"] == 0
    assert result.incumbents == [event for event, _ in events]


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
def test_cold_failure_retains_only_an_eligible_checked_seed(mode, tmp_path):
    fixture = tight_instance(capacity=1)
    descriptor = fixture.instance.model_dump(mode="json", exclude_none=True)
    descriptor["td"]["atf_sha256"] = compute_atf_sha256(fixture.atfs)
    save_instance_atfs(fixture.atfs, tmp_path / descriptor["td"]["atf_path"])
    instance_path = tmp_path / "toy.vrp.json"
    instance_path.write_text(json.dumps(descriptor), encoding="utf-8")
    loaded = load_instance(instance_path, verify=True)
    seed = [[1], [2]]
    events = []

    def mutate_callback_copy(event, routes):
        events.append(event)
        routes[0].clear()

    result = kayros.solve(loaded, params(mode), initial_routes=seed, on_incumbent=mutate_callback_copy)
    assert seed == result.routes == [[1], [2]]
    assert result.exact_cost == 5 and result.cost == 5 and result.search_value is None
    assert result.exact_route_durations == [2, 1] and result.exact_route_departures == [15, 15]
    assert result.status == "finished" and result.iterations == result.work_units == 0
    assert len(events) == result.acceptance_stats["proposals"] == 1
    assert_seed_event(events[0], result.exact_cost)
    saved = save_td_solution_as_bks_if_improved(loaded, result.to_benchmark_solution(), authors="test",
                                                objective_function="FleetCostDuration")
    stored = load_bks(saved.path)
    assert stored.routes == result.routes and stored.cost == result.cost
    for field in ("authoritative_cost", "route_evaluations"):
        assert stored.metadata["exact_evaluation"][field] == result.exact_evaluation[field]
    with pytest.raises(kayros.InfeasibleError, match="k_cap=1"):
        kayros.solve(loaded, params(mode, 1), initial_routes=seed,
                     on_incumbent=lambda *_: pytest.fail("above-cap seed published"))


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
@pytest.mark.parametrize("return_from_one", [2, 4])
def test_equal_or_worse_cold_solution_cannot_replace_the_checked_seed(mode, return_from_one):
    loaded = tight_instance(return_from_one=return_from_one)
    seed = [[1, 2]]
    result = kayros.solve(loaded, params(mode, 1), initial_routes=seed)
    assert result.routes == seed and result.exact_cost == 4 and result.search_value is None
    assert result.exact_route_durations == [3] and result.exact_route_departures == [15]
    assert_seed_event(result.incumbents[0], result.exact_cost)
    if mode == "each_incumbent":
        assert len(result.incumbents) == 1 and result.acceptance_stats["nonimproving"] >= 1
    else:
        assert result.incumbents[1].origin == "greedy" and result.incumbents[1].exact_cost is None


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
def test_warm_callback_time_consumes_the_same_search_budget(mode):
    loaded = tight_instance()

    def callback(event, routes):
        assert event.origin == "warm_start"
        sleep(0.02)

    result = kayros.solve(loaded, params(mode), initial_routes=[[1], [2]], time_limit=0.005,
                          on_incumbent=callback)
    assert result.routes == [[1], [2]] and result.exact_cost == 5 and result.search_value is None
    assert result.status == "time_limit" and result.iterations == result.work_units == 0
    assert result.acceptance_stats["proposals"] == 1
    with pytest.raises(kayros.InfeasibleError, match="k_cap=1"):
        kayros.solve(loaded, params(mode, 1), initial_routes=[[1], [2]], time_limit=1e-12,
                     on_incumbent=lambda *_: pytest.fail("no time for a cold publication"))


@pytest.mark.parametrize("history_size", [0, 6])
def test_final_only_falls_back_after_all_cold_candidates_fail_the_final_check(monkeypatch, history_size):
    loaded = tight_instance()
    seed = [[1], [2]]
    module = import_module("kayros.solve")
    prepared = module.prepare_exact(loaded, objective_function="FleetCostDuration")
    calls = []

    def evaluate(routes):
        calls.append([list(route) for route in routes])
        if calls[-1] == [[2, 1]]:
            raise ExactRouteError(SolutionCheckStatus.TIME_WINDOW_VIOLATED, "forced cold rejection")
        return prepared.evaluate(routes)

    monkeypatch.setattr(module, "prepare_exact", lambda *_args, **_kwargs: SimpleNamespace(evaluate=evaluate))
    config = params("final_only")
    config.incumbent_history_size = history_size
    result = kayros.solve(loaded, config, initial_routes=seed)
    assert calls == [seed, [[2, 1]], seed]
    assert result.routes == seed and result.exact_cost == 5 and result.search_value is None
    assert result.acceptance_stats["fallback_attempts"] == result.acceptance_stats["recovered"] == 1
    assert result.acceptance_stats["final_rejections"] == 1


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
def test_zero_fleet_charge_enforces_the_cap_for_warm_and_cold_starts(mode):
    seed = [[1], [2]]
    for direct_has_knot in (True, False):
        loaded = tight_instance(fixed_charge=0, direct_has_knot=direct_has_knot)
        result = kayros.solve(loaded, params(mode, 1), initial_routes=seed)
        assert result.routes == [[2, 1]] and result.exact_cost == 2
        assert all(event.origin != "warm_start" for event in result.incumbents)
        assert result.k_stats["k_best_max"] == result.k_cap_stats["reached_cap"] == 1

    # Capacity forces two routes. These ordinary starts both fail the one-route fleet cap.
    loaded = tight_instance(fixed_charge=0, direct_has_knot=True, capacity=1)
    for initial in (None, seed):
        with pytest.raises(kayros.InfeasibleError, match="k_cap=1"):
            kayros.solve(loaded, params(mode, 1), initial_routes=initial,
                         on_incumbent=lambda *_: pytest.fail("zero-charge fleet cap bypassed"))
        duration = kayros.Params(acceptance_mode=mode, k_cap=1, ils_max_iterations=1)
        uncapped = kayros.solve(loaded, duration, initial_routes=initial)
        assert uncapped.num_routes == 2 and uncapped.k_cap_stats is None


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
def test_exact_warm_objective_exceeding_binary64_range(mode):
    loaded = tight_instance(fixed_charge=1e308, direct_has_knot=True)
    seed = [[1], [2]]
    assert all(to_core(loaded).evaluate_route(route)[0] for route in seed)
    result = kayros.solve(loaded, params(mode), initial_routes=seed, time_limit=1e-12)
    assert result.routes == seed and result.exact_cost == 2 * Fraction(1e308) + 3
    assert result.cost is None and result.search_value is None
    event, = result.incumbents
    assert event.origin == "warm_start" and event.exact_cost == result.exact_cost
    assert event.value is None and event.search_value is None

@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
@pytest.mark.parametrize("cap", [1, 2])
def test_reference_search_from_an_exactly_feasible_seed(mode, cap):
    loaded = tight_instance()
    seed = [[1], [2]]
    assert not to_core(loaded).evaluate_route([1])[0]
    settings = params(mode, cap)
    settings.search_mode = "exact"
    published = []
    result = kayros.solve(loaded, settings, initial_routes=seed,
                          on_incumbent=lambda event, routes: published.append((event, routes)))
    oracle = prepare_td(loaded.instance, loaded.atfs, objective_function="FleetCostDuration", backend="python")
    assert result.routes == [[2, 1]] and result.exact_cost == 3
    assert published[-1][1] == result.routes
    assert [event.exact_cost for event, _ in published] == ([5, 3] if cap == 2 else [3])
    for event, routes in published:
        assert len(routes) <= cap
        assert event.exact_cost == oracle.evaluate(routes).cost
        assert event.value == float(event.exact_cost) and event.search_value is None
    assert result.search_value is None and result.incumbents == [event for event, _ in published]
    assert result.acceptance_stats["proposals"] == len(published)
    assert result.exact_route_departures == [15] and result.exact_route_durations == [2]


def test_reference_search_distinguishes_two_costs_with_one_display():
    charge = 2**55
    loaded = tight_instance(fixed_charge=charge)
    oracle = prepare_td(loaded.instance, loaded.atfs, objective_function="FleetCostDuration", backend="python")
    before = oracle.evaluate([[1, 2]])
    after = oracle.evaluate([[2, 1]])
    assert before.cost == charge + 3 and after.cost == charge + 2
    assert before.display_cost == after.display_cost
    settings = kayros.Params(search_mode="exact", objective="FleetCostDuration", k_cap=1, ils_max_iterations=2)
    first = kayros.solve(loaded, settings, seed=17, initial_routes=[[1, 2]])
    second = kayros.solve(loaded, settings, seed=17, initial_routes=[[1, 2]])
    assert first.routes == second.routes == [[2, 1]]
    assert first.work_units == second.work_units > 0
    for run in (first, second):
        assert run.exact_cost == after.cost and run.cost == after.display_cost
        assert [event.exact_cost for event in run.incumbents] == [before.cost, after.cost]
        assert [event.value for event in run.incumbents] == [before.display_cost, after.display_cost]


def test_reference_search_deadline_keeps_the_checked_warm_start():
    loaded = tight_instance(capacity=1)
    settings = kayros.Params(search_mode="exact", objective="FleetCostDuration", k_cap=2)
    seed = [[1], [2]]
    result = kayros.solve(loaded, settings, initial_routes=seed, time_limit=1e-12)
    assert result.routes == seed and result.exact_cost == 5
    assert result.status == "time_limit"
    assert_seed_event(result.incumbents[0], result.exact_cost)


@pytest.mark.parametrize("changes", [{"sq_work_cap": 1}, {"sq_ladder": True}])
def test_reference_squeeze_parameters_are_explicit(changes):
    with pytest.raises(ValueError, match="sq_work_cap=0"):
        kayros.solve(tight_instance(), kayros.Params(search_mode="exact", **changes), time_limit=1)


def test_unknown_search_mode_rejected():
    with pytest.raises(ValueError, match="unknown search_mode"):
        kayros.solve(tight_instance(), kayros.Params(search_mode="other"), time_limit=1)

def test_empty_reference_instance_has_exact_zero_cost():
    loaded = tight_instance()
    empty = loaded.instance.model_copy(update={
        "num_customers": 0, "num_vehicles": 0, "demands": [0], "coordinates": [(0, 0)],
        "service_times": [0], "time_windows": [(0, 22)],
    })
    atfs = InstanceATFs(empty.instance_name, "Blauth2024", (0, 22), 0, {})
    result = kayros.solve(replace(loaded, instance=empty, atfs=atfs),
                          kayros.Params(search_mode="exact", objective="FleetCostDuration", ils_max_iterations=1))
    assert result.routes == result.exact_route_departures == result.exact_route_durations == []
    assert result.cost == result.exact_cost == result.incumbents[0].exact_cost == 0
    assert result.status == "finished" and result.search_value is None
