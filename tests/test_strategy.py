"""Iterated local search budgets, determinism and incumbent reporting."""

from dataclasses import fields

import pytest

import kayros

from conftest import family_instances, require_benchmarks


def pick_one(problem_type, family, size_dir, name=None):
    paths = family_instances(problem_type, family, [size_dir])
    if name is not None:
        paths = [p for p in paths if p.name.startswith(name)]
    return paths[0] if paths else None


def dabia(name, size_dir="n=25"):
    require_benchmarks()
    path = pick_one("TDVRPTW", "Dabia2013", size_dir, name)
    if path is None:
        pytest.skip("instance not present")
    return path


def test_default_strategy_is_ils() -> None:
    # ILS is the default strategy.
    assert kayros.Params().strategy == "ils"
    assert kayros.Params().search_mode == "binary64"


def test_ils_parameter_names_and_mapping() -> None:
    defaults = kayros.Params()
    public_only = {"strategy", "objective", "acceptance_mode", "incumbent_history_size", "search_mode"}
    names = {f.name for f in fields(defaults)} - public_only
    native_defaults = kayros._core.IlsParams()
    assert {n for n in dir(native_defaults) if not n.startswith("_")} == names

    def values(core):
        return {name: getattr(core, name) for name in names}

    expected = {name: getattr(defaults, name) for name in names}
    # The public zero sentinel lets solve() select a time or fallback iteration budget.
    expected["ils_max_iterations"] = (1 << 64) - 1
    assert values(defaults._to_ils_core()) == values(native_defaults) == expected
    # Distinct markers also catch accidental transposition between same-type budgets.
    custom = {}
    for index, name in enumerate(sorted(names), 1):
        default = getattr(defaults, name)
        if type(default) is bool:
            custom[name] = not default
        elif type(default) is float:
            custom[name] = index + 0.25
        else:
            custom[name] = index
    assert values(kayros.Params(**custom)._to_ils_core()) == custom


def test_unknown_strategy_rejected() -> None:
    path = dabia("R101")
    with pytest.raises(ValueError, match="unknown strategy"):
        kayros.solve(path, kayros.Params(strategy="hgs"), time_limit=1)


def test_ils_without_budget_falls_back_to_restart_windows() -> None:
    """No time limit + no ils_max_iterations: solve() bounds the run at five
    restart windows (5 * restart_no_improvement) instead of raising."""
    path = dabia("R101")
    solution = kayros.solve(
        path, kayros.Params(strategy="ils", restart_no_improvement=40), seed=7
    )
    assert solution.iterations == 200
    assert solution.cost > 0.0


def test_ils_strategy_end_to_end_deterministic() -> None:
    path = dabia("C101")
    params = kayros.Params(strategy="ils", ils_max_iterations=150)
    a = kayros.solve(path, params, seed=11)
    b = kayros.solve(path, params, seed=11)
    assert a.routes == b.routes
    assert a.cost == b.cost
    origins = {i.origin for i in a.incumbents}
    assert origins <= {"greedy", "ils"}
    assert a.incumbents[0].origin == "greedy"
    values = [i.exact_cost for i in a.incumbents]
    assert values == sorted(values, reverse=True)
    assert len(set(values)) == len(values)
    assert a.exact_cost == values[-1]
    assert a.search_value == a.incumbents[-1].search_value


def test_ils_strategy_hook_monotone() -> None:
    path = dabia("R101")
    events = []
    result = kayros.solve(
        path,
        kayros.Params(strategy="ils", ils_max_iterations=150),
        seed=3,
        on_incumbent=lambda inc, routes: events.append(inc),
    )
    assert events
    values = [e.exact_cost for e in events]
    assert values == sorted(values, reverse=True)
    assert len(set(values)) == len(values)
    assert values[-1] == result.exact_cost
    assert events[-1].search_value == result.search_value
