"""Warm starts through the public heuristic API."""
import pytest

import kayros
from conftest import family_instances, require_benchmarks
from kayros.io import load_instance


@pytest.fixture(scope="module")
def blauth_seed():
    require_benchmarks()
    paths = family_instances("TDVRPTW", "Blauth2024", ["n=10"])
    loaded = load_instance(next(p for p in paths if "berlin" in p.name))
    solution = kayros.solve(loaded, kayros.Params(objective="FleetCostDuration", ils_max_iterations=10))
    return loaded, solution


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
def test_warm_start_polishes_supplied_routes(blauth_seed, mode):
    loaded, prior = blauth_seed
    params = kayros.Params(objective="FleetCostDuration", ils_max_iterations=20,
                           k_cap=prior.num_routes, acceptance_mode=mode)
    events = []
    result = kayros.solve(loaded, params, initial_routes=prior.routes,
                          on_incumbent=lambda event, routes: events.append((event, routes)))
    assert events[0][0].origin == "warm_start"
    assert events[0][1] == prior.routes
    if mode == "each_incumbent":
        assert events[0][0].exact_cost == prior.exact_cost
        assert result.exact_cost <= prior.exact_cost
    assert result.num_routes <= prior.num_routes


@pytest.mark.parametrize("mode", ["each_incumbent", "final_only"])
@pytest.mark.parametrize("bad_routes", [[], [[0, 1, 0]], [[1, 1]], [[1]]])
def test_bad_warm_start_is_rejected_before_any_publication(blauth_seed, mode, bad_routes):
    loaded, _ = blauth_seed
    with pytest.raises(ValueError, match="invalid initial_routes"):
        kayros.solve(loaded, kayros.Params(acceptance_mode=mode, ils_max_iterations=1),
                     initial_routes=bad_routes, on_incumbent=lambda *_: pytest.fail("invalid seed published"))


def test_above_cap_seed_is_valid_but_cannot_be_published(blauth_seed):
    loaded, prior = blauth_seed
    split = [[c] for route in prior.routes for c in route]
    assert len(split) > prior.num_routes
    published = []
    result = kayros.solve(
        loaded, kayros.Params(objective="FleetCostDuration", k_cap=prior.num_routes, ils_max_iterations=100),
        initial_routes=split, on_incumbent=lambda _, routes: published.append(len(routes)),
    )
    assert result.k_stats["k_seed"] == len(split)
    assert published and max(published) <= prior.num_routes
