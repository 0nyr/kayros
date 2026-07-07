"""M5.8: the exact BPC component (Lera bridge, HiGHS backend) ships in the default install.

Import must succeed on any default build; the solve gates certify a small instance
end-to-end with the M5.6 checker-exact contract (reported value == checker repricing,
bit-exactly) and exercise the M5.3 warm-start path.
"""

import pytest
from mamut_routing_lib.td import compute_solution_cost, load_td_instance

from conftest import family_instances


def pick(paths, names):
    return [p for p in paths if p.name.removesuffix(".vrp.json") in names]


C101_25 = pick(family_instances("TDVRPTW", "Dabia2013", ["n=25"]), {"C101"})


def test_lera_module_present() -> None:
    import kayros._lera  # noqa: F401
    from kayros import lera  # noqa: F401


@pytest.mark.parametrize("instance_path", C101_25, ids=lambda p: "C101")
def test_lera_certifies_c101(instance_path) -> None:
    from kayros.lera import routes_to_mamut, solve_duration

    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit_s=120.0)
    assert result["exact_log"]["status"] == "Optimum"
    routes = [r[1:-1] for r in routes_to_mamut(result["routes"], loaded.instance.num_customers)]
    checker_value = compute_solution_cost(loaded.instance, loaded.atfs, routes)
    # M5.6 stage A: the reported value IS the checker value, exactly.
    assert result["value"] == checker_value


@pytest.mark.parametrize("instance_path", C101_25, ids=lambda p: "C101")
def test_lera_warm_start_contract(instance_path) -> None:
    from kayros.lera import routes_to_mamut, solve_duration

    loaded = load_td_instance(instance_path)
    cold = solve_duration(loaded, time_limit_s=120.0)
    assert cold["exact_log"]["status"] == "Optimum"
    seed = [r[1:-1] for r in routes_to_mamut(cold["routes"], loaded.instance.num_customers)]
    warm = solve_duration(loaded, time_limit_s=120.0, initial_routes=seed)
    assert warm["exact_log"]["status"] == "Optimum"
    assert warm["value"] == cold["value"]
    # M5.3 contract: proving the seed optimal returns the proven value with
    # empty routes — the caller's own solution is the optimum.
    assert warm["routes"] == []
