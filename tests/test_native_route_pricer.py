"""Exact route acceptance and witnesses through the shared MAMUT C++ header."""

from fractions import Fraction
from math import ulp

import pytest
from mamut_routing_lib.exact_td import prepare_td
from mamut_routing_lib.td import load_td_instance

import kayros
from conftest import family_instances, instance_path
from kayros import _core
from kayros.io import to_core


def simple_pricer(budget, *, windows=((0, 15), (6, 7)), charge=0.0):
    instance = _core.Instance(1, 1, 1, (0, 15), windows, [0, 1], [0, 1],
                              [(0, 1, [0, 15], [1, 23.5]), (1, 0, [0, 15], [1, 16])], charge)
    return _core.ExactRoutePricer(instance, cache_knots=budget)


@pytest.mark.parametrize("budget,retained,preparations", [(0, 0, 4), (1, 0, 4), (2, 2, 4), (4, 4, 2)])
def test_exact_fraction_reuse_and_eviction(budget, retained, preparations):
    pricer = simple_pricer(budget, charge=0.1)
    assert pricer.cached_knots == pricer.prepared_arcs == 0
    duration, departure = pricer.route_timing([1])
    assert (Fraction(*duration), Fraction(*departure)) == (Fraction(14, 3), Fraction(10, 3))
    assert Fraction(*pricer.solution_cost([[1]])) == Fraction(14, 3) + Fraction(0.1)
    assert (pricer.cached_knots, pricer.prepared_arcs) == (retained, preparations)


def test_unwindowed_return_and_infeasibility():
    plain = simple_pricer(4, windows=None)
    assert tuple(Fraction(*q) for q in plain.route_timing([1])) == (3, 0)
    closed = simple_pricer(4, windows=((0, 7), (6, 7)))
    assert closed.route_timing([1]) is None
    assert closed.solution_cost([[1]]) is None
    for invalid in [[], [0], [2]]:
        with pytest.raises(ValueError):
            plain.route_timing(invalid)


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
@pytest.mark.parametrize("path", [p for p in family_instances("TDVRPTW", "Rifki2020", ["n=30"])
                                  if p.name == "Rifki-10.vrp.json"])
def test_rifki_jump_witness_survives_public_warm_start(path, mode):
    loaded = load_td_instance(path)
    route = [6, 17, 12, 22, 16]
    core = to_core(loaded)
    native = _core.ExactRoutePricer(core)
    gold = prepare_td(loaded.instance, loaded.atfs, backend="python", objective_function="Duration")
    timing = gold.route_timing(route)
    assert tuple(Fraction(*q) for q in native.route_timing(route)) == (timing.duration, timing.departure_time)
    assert (timing.duration, timing.departure_time) == (2002, 26949)

    routes = [route] + [[v] for v in range(1, core.num_customers + 1) if v not in route]
    # A deadline shorter than one steady-clock tick allows only the supplied incumbent.
    result = kayros.solve(loaded, kayros.Params(acceptance_mode=mode), initial_routes=routes, time_limit=1e-12)
    i = result.routes.index(route)
    assert (result.exact_route_durations[i], result.exact_route_departures[i]) == (timing.duration, timing.departure_time)
    assert result.route_departures[i] == float(timing.departure_time)
    assert result.exact_cost == Fraction(*native.solution_cost(result.routes)) == gold.evaluate(result.routes).cost


def test_rifki_1_unwindowed_stepwise_minimum():
    path = instance_path("TDVRP", "Rifki2020", "n=30", "Rifki-1")
    if not path.is_file():
        pytest.skip(f"missing benchmark instance: {path}")
    loaded = load_td_instance(path)
    core = to_core(loaded)
    assert not core.has_time_windows
    route = [24, 12, 1, 8, 15]
    oracle = prepare_td(loaded.instance, loaded.atfs, backend="python", objective_function="Duration")
    timing = oracle.route_timing(route)
    assert (timing.duration, timing.departure_time) == (2507, 2253)
    native = _core.ExactRoutePricer(core).route_timing(route)
    assert tuple(Fraction(*q) for q in native) == (timing.duration, timing.departure_time)

    # Replay the fold's chosen departure directly on the original arc knots.
    feasible, duration, departure = core.evaluate_route(route)
    assert feasible
    assert loaded.instance.horizon[0] <= departure <= loaded.instance.horizon[1]
    ready = Fraction(departure)
    for tail, head in zip([0, *route], [*route, 0]):
        ready = loaded.atfs.arcs[tail, head].evaluate_exact(ready) + Fraction(loaded.instance.service_times[head])
    attained = ready - Fraction(departure)
    assert abs(Fraction(duration) - attained) <= ulp(duration)
    assert 0 <= attained - timing.duration <= 5


@pytest.mark.parametrize("path", [p for p in family_instances("TDVRP", "Rifki2020", ["n=30"])
                                  if p.name == "Rifki-11.vrp.json"])
def test_unwindowed_stepwise_witness(path):
    loaded = load_td_instance(path)
    native = _core.ExactRoutePricer(to_core(loaded))
    route = [3, 2]
    gold = prepare_td(loaded.instance, loaded.atfs, backend="python", objective_function="Duration")
    timing = gold.route_timing(route)
    assert (timing.duration, timing.departure_time) == (1383, 720)
    assert tuple(Fraction(*q) for q in native.route_timing(route)) == (timing.duration, timing.departure_time)


def test_missing_materialized_arc_is_an_input_error():
    instance = _core.Instance(1, 1, 1, (0, 15), None, [0, 1], [0, 0], [(0, 1, [0, 15], [1, 16])])
    pricer = _core.ExactRoutePricer(instance)
    with pytest.raises(ValueError, match=r"missing materialized TD arc \(1, 0\)"):
        pricer.route_timing([1])


@pytest.mark.parametrize("path", [p for p in family_instances("TDVRPTW", "Dabia2013", ["n=25"])
                                  if p.name == "C101.vrp.json"])
def test_rejected_proposal_and_independent_final_check(path, monkeypatch):
    from kayros.solve import _Acceptance

    loaded = load_td_instance(path)
    params = kayros.Params(ils_max_iterations=1)
    policy = _Acceptance(loaded, "Duration", params, None)
    assert not policy.accept([[1]], True)
    assert policy.stats["infeasible"] == policy.stats["squeeze_rejections"] == 1

    proposal = _Acceptance.proposal_cost
    monkeypatch.setattr(_Acceptance, "proposal_cost", lambda self, routes: proposal(self, routes) + 1)
    with pytest.raises(kayros.KayrosError, match="final exact evaluation disagrees"):
        kayros.solve(loaded, params)

    monkeypatch.setattr(_Acceptance, "proposal_cost", lambda *_: pytest.fail("final-only mode priced a proposal"))
    solution = kayros.solve(loaded, kayros.Params(acceptance_mode="final_only", ils_max_iterations=1))
    assert solution.exact_cost == policy.prepared.evaluate(solution.routes).cost
