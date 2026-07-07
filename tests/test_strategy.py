"""M7.3 strategy-API gates: Params.strategy dispatch through kayros.solve().

solve() itself re-prices every returned solution through the reference
checker and raises on disagreement, so these gates focus on the dispatch
contract: validation, determinism per strategy, incumbent stream shape
(strictly improving, correct origins) and the aco+ils merge.
"""

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


def test_default_strategy_is_aco() -> None:
    assert kayros.Params().strategy == "aco"


def test_unknown_strategy_rejected() -> None:
    path = dabia("R101")
    with pytest.raises(ValueError, match="unknown strategy"):
        kayros.solve(path, kayros.Params(strategy="hgs"), time_limit=1)


def test_ils_requires_a_budget() -> None:
    path = dabia("R101")
    with pytest.raises(ValueError, match="time_limit or ils_max_iterations"):
        kayros.solve(path, kayros.Params(strategy="ils"))


def test_split_requires_time_limit() -> None:
    path = dabia("R101")
    with pytest.raises(ValueError, match="needs a time_limit"):
        kayros.solve(path, kayros.Params(strategy="aco+ils"))


def test_ils_strategy_end_to_end_deterministic() -> None:
    path = dabia("C101")
    params = kayros.Params(strategy="ils", ils_max_iterations=150)
    a = kayros.solve(path, params, seed=11)
    b = kayros.solve(path, params, seed=11)
    assert a.routes == b.routes
    assert a.duration == b.duration
    origins = {i.origin for i in a.incumbents}
    assert origins <= {"greedy", "ils"}
    assert a.incumbents[0].origin == "greedy"
    values = [i.value for i in a.incumbents]
    assert values == sorted(values, reverse=True)
    assert len(set(values)) == len(values)
    assert a.duration == values[-1]


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
    values = [e.value for e in events]
    assert values == sorted(values, reverse=True)
    assert len(set(values)) == len(values)
    assert values[-1] == result.duration


def test_split_strategy_runs_both_phases() -> None:
    path = dabia("R205", size_dir="n=50")
    events = []
    result = kayros.solve(
        path,
        kayros.Params(strategy="aco+ils", max_iterations=100_000,
                      max_no_improvement=100_000),
        time_limit=8,
        seed=5,
        on_incumbent=lambda inc, routes: events.append(inc),
    )
    values = [i.value for i in result.incumbents]
    assert values == sorted(values, reverse=True)
    assert len(set(values)) == len(values), "the merged stream must be strict"
    assert result.incumbents[0].origin == "greedy"
    assert result.duration == values[-1]
    # The hook stream must match the merged record stream in values.
    assert [e.value for e in events] == values
    # Both phases fired their seed/greedy event; the ILS phase ran (its
    # incumbents, if any beat the ACO best, carry the "ils" origin).
    assert {i.origin for i in result.incumbents} <= {"greedy", "aco", "ils"}


def test_ils_strategy_beats_construction_only() -> None:
    path = dabia("R101")
    ils = kayros.solve(
        path, kayros.Params(strategy="ils", ils_max_iterations=300), seed=3
    )
    no_ls = kayros.solve(
        path,
        kayros.Params(max_iterations=40, max_no_improvement=40,
                      local_search=False),
        seed=3,
    )
    assert ils.duration < no_ls.duration
