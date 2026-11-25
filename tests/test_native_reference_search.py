"""Native exact search on directed instances with independently priced tours."""

from fractions import Fraction

import pytest

from kayros import _core


def directed_instance(target, capacity):
    n = sum(map(len, target))
    preferred = {(i, j) for route in target for i, j in zip([0, *route], [*route, 0])}
    weights = {(i, j): 1 if (i, j) in preferred else 9 for i in range(n + 1) for j in range(n + 1) if i != j}
    arcs = [(i, j, [0, 128], [t, 128 + t]) for (i, j), t in weights.items()]
    core = _core.Instance(n, len(target), capacity, (0, 128), None, [0] + [1] * n, [0] * (n + 1), arcs)
    return core, weights


def run(core, warm, *, iterations=0, seed=3, **overrides):
    params = _core.IlsParams()
    params.fd_attempts = 0
    params.ils_max_iterations = iterations
    for key, value in overrides.items():
        setattr(params, key, value)
    events = []

    def record(routes, cost, seconds, iteration, origin):
        events.append((routes, Fraction(*cost), iteration, origin))

    result = _core.solve_reference(core, params, seed, 0, record, warm)
    return result, events


# Each target is reachable by one move. Every route edge costs at least one.
@pytest.mark.parametrize("warm,target,capacity", [
    ([[1, 2, 3, 4, 5]], [[2, 3, 4, 1, 5]], 5),                         # relocate
    ([[1, 2, 3], [4]], [[1, 3], [4, 2]], 3),                          # relocate between routes
    ([[1, 2, 3, 4, 5]], [[1, 5, 3, 4, 2]], 5),                       # swap
    ([[1, 2, 3], [4, 5, 6]], [[1, 5, 3], [4, 2, 6]], 3),            # swap between routes
    ([[1, 2, 3, 4, 5, 6]], [[1, 5, 4, 3, 2, 6]], 6),                 # intra-route 2-opt
    ([[1, 2, 3, 4], [5, 6, 7, 8]], [[1, 2, 7, 8], [5, 6, 3, 4]], 4),  # 2-opt*
])
def test_descent_finds_the_directed_unit_edge_tours(warm, target, capacity):
    core, weights = directed_instance(target, capacity)
    result, events = run(core, warm)
    assert sorted(result.routes) == sorted(target)
    assert [event[3] for event in events] == [3, 2]
    for routes, cost, iteration, origin in events:
        assert cost == sum(weights[i, j] for route in routes for i, j in zip([0, *route], [*route, 0]))
    assert Fraction(*result.exact_cost) == sum(map(len, target)) + len(target)
    assert result.iterations_run == 0 and result.work_units > 1


def test_perturbation_repair_and_restart_are_deterministic():
    core, _ = directed_instance([[1, 3, 5], [2, 4, 6]], 3)
    args = dict(iterations=4, restart_no_improvement=2, min_perturbations=3, max_perturbations=3)
    first, logs = run(core, [], **args)
    second, replay = run(core, [], **args)
    assert first.routes == second.routes
    assert first.work_units == second.work_units
    assert first.restarts == second.restarts and first.restarts > 0
    assert logs == replay
    assert first.fleet_stats.kicks_applied == first.iterations_run == 4


@pytest.mark.parametrize("bad", [[[1]], [[1, 1]], [[1, 3]], [[], [1, 2]]])
def test_native_warm_coverage(bad):
    core, _ = directed_instance([[1, 2]], 2)
    with pytest.raises(ValueError, match="warm start"):
        run(core, bad)


@pytest.mark.parametrize("settings", [dict(sq_work_cap=1), dict(sq_ladder=True)])
def test_native_rejects_squeeze(settings):
    core, _ = directed_instance([[1, 2]], 2)
    with pytest.raises(ValueError, match="squeeze"):
        run(core, [], **settings)


def test_cold_construction_can_defer_a_customer_until_a_predecessor_exists():
    times = {(0, 1): 6, (0, 2): 1, (1, 0): 1, (1, 2): 1, (2, 0): 1, (2, 1): 1}
    arcs = [(i, j, [0, 20], [t, 20 + t]) for (i, j), t in times.items()]
    core = _core.Instance(2, 1, 2, (0, 20), [(0, 20), (0, 4), (0, 20)], [0, 1, 1], [0, 0, 0], arcs)
    assert _core.ExactRoutePricer(core).route_timing([1]) is None
    result, events = run(core, [])
    assert result.routes == [[2, 1]] and Fraction(*result.exact_cost) == 3
    assert events == [([[2, 1]], 3, 0, 0)]


def test_zero_charge_fleet_drain_respects_the_cap_and_work_budget():
    arcs = [(i, j, [0, 20], [1, 21]) for i in range(4) for j in range(4) if i != j]
    core = _core.Instance(3, 3, 3, (0, 20), None, [0, 1, 1, 1], [0] * 4, arcs)
    params = _core.IlsParams()
    params.ils_max_iterations = 0
    params.k_cap = 1
    for budget in (1, 200):
        params.fd_work_cap = budget
        events = []
        result = _core.solve_reference(core, params, 1, 0, lambda *args: events.append(args),
                                       [[1], [2, 3]], fleet_cost_objective=True)
        assert result.k_cap_stats.seed_drain_attempts == 1
        assert result.fd_stats.evaluated <= budget
        assert result.fd_stats.rollbacks_work == (budget == 1)
        assert result.fd_stats.successes == (budget == 200)
        assert len(result.routes) == 1 and Fraction(*result.exact_cost) == 4
        assert len(events) == 1 and len(events[0][0]) == 1
