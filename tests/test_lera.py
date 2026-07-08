"""The exact BPC component (Lera bridge, HiGHS backend) ships in the default install.

Import must succeed on any default build; the solve gates certify a small instance
end-to-end with the checker-exact contract (reported value == checker repricing,
bit-exactly) and exercise the warm-start path and the optimality-stamp helper.
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


def test_lera_exposes_lp_backend() -> None:
    import kayros._lera as _lera

    assert _lera.LP_BACKEND in ("HiGHS", "CPLEX")


@pytest.mark.parametrize("instance_path", C101_25, ids=lambda p: "C101")
def test_lera_certifies_c101(instance_path) -> None:
    from kayros.lera import CERTIFICATE, optimality_metadata, routes_to_mamut, solve_duration

    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit_s=120.0)
    assert result["exact_log"]["status"] == "Optimum"
    routes = [r[1:-1] for r in routes_to_mamut(result["routes"], loaded.instance.num_customers)]
    checker_value = compute_solution_cost(loaded.instance, loaded.atfs, routes)
    # The reported value IS the checker value, exactly.
    assert result["value"] == checker_value

    stamp = optimality_metadata(result, wall_time_s=1.0, time_limit_s=120.0, date="2026-07-07")
    assert stamp is not None
    assert stamp["proven"] is True
    assert stamp["certificate"] == CERTIFICATE
    assert stamp["proven_optimum"] == result["value"]
    assert stamp["dual_bound"] == result["exact_log"]["best_bound"]
    assert stamp["prover"].startswith("kayros ")
    assert "LP backend" in stamp["prover"]
    assert stamp["date"] == "2026-07-07"
    assert "campaign" not in stamp  # omitted when not provided


def test_optimality_metadata_none_unless_optimum() -> None:
    from kayros.lera import optimality_metadata

    assert optimality_metadata({"exact_log": {"status": "TimeLimitReached"}}) is None
    assert optimality_metadata({}) is None


RIFKI_10 = pick(family_instances("TDVRPTW", "Rifki2020", ["n=10"]), {"Rifki-1"})


@pytest.mark.parametrize("instance_path", RIFKI_10, ids=lambda p: "Rifki-1")
def test_no_optimality_stamp_on_stepwise_atfs(instance_path) -> None:
    # Stepwise (duplicate-x jump) ATFs: certificates refuted by counterexample
    # (Rifki2020 retraction, 2026-07-08; NOTICE item 9) — an "Optimum" status
    # there is warm-start-dependent, so the stamp helper must refuse it.
    from kayros.lera import optimality_metadata, solve_duration

    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit_s=120.0)
    assert result["stepwise_atfs"] is True
    assert result["exact_log"]["status"] == "Optimum"
    assert optimality_metadata(result, wall_time_s=1.0, time_limit_s=120.0) is None


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
    # Warm-start contract: proving the seed optimal returns the proven value
    # with empty routes — the caller's own solution is the optimum.
    assert warm["routes"] == []
