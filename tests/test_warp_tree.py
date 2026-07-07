"""P8.2 gates: warp-augmented segments in the LCA-BST.

Discipline per the P8.0 memo §5 (the P4.1 lesson transposed): warp dissolves
the domain wall, so BOTH channels are value channels under association dust —
trees rank, the sequential augmented fold accounts. Hence:

- ``update_leaf`` must equal a fresh rebuild BITWISE on every query range
  (same association — the td-route-trees P4.3 gate on both channels);
- full-range tree queries vs the sequential fold: zero-warp feasibility must
  agree exactly (the empirical P4.1 claim, now on the warp channel), values
  agree association-close (tight on Dabia, feasibility-only on Rifki steps);
- splice evaluations vs direct fold pricing of the spliced vector: same.
"""

import random

import pytest

from kayros import _core
from kayros.io import load_instance, to_core

from conftest import family_instances, require_benchmarks
from test_ls import GATE_PICKS, greedy_routes, load_core, pick_one, spliced

REL_TOL = 1e-9
PENALTY = 10.0


def gate_cases():
    require_benchmarks()
    cases = []
    for problem_type, family, size_dir, name, tight in GATE_PICKS:
        path = pick_one(problem_type, family, size_dir, name)
        if path is not None:
            cases.append((path, tight))
    assert cases, "no benchmark instances found"
    return cases


@pytest.fixture(scope="module", params=range(len(GATE_PICKS)))
def gate_case(request):
    cases = gate_cases()
    if request.param >= len(cases):
        pytest.skip("pick not present in this checkout")
    return cases[request.param]


def fold_eval(core, route, t_end):
    return core.evaluate_route_warp(route, PENALTY, t_end)


def test_tree_full_query_matches_fold(gate_case) -> None:
    """Full-range (rho, omega) tree query vs the sequential augmented fold."""
    path, tight = gate_case
    core = load_core(path)
    routes = [r for r in greedy_routes(core) if r]
    t_end = core.warp_horizon()
    rng = random.Random(11)

    checked = 0
    for _ in range(60):
        base = routes[rng.randrange(len(routes))]
        route = list(base)
        rng.shuffle(route)  # shuffles are mostly warped on TDVRPTW
        (rho_xs, rho_ys), (w_xs, w_ys) = _core.warp_tree_full(core, route, t_end)
        ref = fold_eval(core, route, t_end)
        if not rho_xs:
            assert not ref["total"], (path.name, route)
            continue
        assert ref["total"], (path.name, route)
        # Zero-warp feasibility: exact agreement across associations (the
        # P4.1 emptiness claim transposed to the warp channel).
        tree_feasible = w_ys[0] == 0.0
        assert tree_feasible == ref["feasible"], (path.name, route)
        if tight:
            assert w_ys[0] == pytest.approx(ref["min_warp"], rel=REL_TOL, abs=1e-9)
            # Zero-warp duration recomputed from the tree channels, mirroring
            # min_zero_warp_duration: breakpoints <= tau PLUS the boundary tau
            # itself (under safe dedup a flat run can straddle tau).
            if tree_feasible:
                tau = max(x for x, y in zip(w_xs, w_ys) if y == 0.0)
                candidates = [y - x for x, y in zip(rho_xs, rho_ys) if x <= tau]
                if tau not in rho_xs and rho_xs[0] <= tau <= rho_xs[-1]:
                    candidates.append(_core.pwlf_evaluate(rho_xs, rho_ys, tau) - tau)
                dur = min(candidates)
                assert dur == pytest.approx(ref["duration"], rel=REL_TOL)
        checked += 1
    assert checked > 20


def test_update_leaf_equals_rebuild(gate_case) -> None:
    """Localized update ≡ fresh rebuild, bitwise, both channels, all ranges."""
    path, _ = gate_case
    core = load_core(path)
    routes = [r for r in greedy_routes(core) if len(r) >= 2]
    t_end = core.warp_horizon()
    rng = random.Random(13)
    assert routes
    for _ in range(12):
        r1 = rng.randrange(len(routes))
        route = routes[r1]
        donor = routes[rng.randrange(len(routes))]
        new_customer = donor[rng.randrange(len(donor))]
        if new_customer in route:
            continue
        position = rng.randrange(len(route))
        assert _core.warp_tree_update_gate(core, route, t_end, position, new_customer), (
            path.name,
            route,
            position,
            new_customer,
        )


def test_splice_warp_matches_fold(gate_case) -> None:
    """Random penalised splices vs direct fold pricing of the spliced vector."""
    path, tight = gate_case
    core = load_core(path)
    routes = [r for r in greedy_routes(core) if r]
    t_end = core.warp_horizon()
    rng = random.Random(17)

    states = []
    for r in routes:
        st = _core.build_warp_route_state(core, r, PENALTY, t_end)
        assert st is not None, (path.name, r)
        states.append(st)

    checked = 0
    for _ in range(400):
        r1 = rng.randrange(len(routes))
        r2 = rng.randrange(len(routes))
        route1, route2 = routes[r1], routes[r2]
        m1, m2 = len(route1), len(route2)
        i1 = rng.randrange(0, m1 + 1)
        j1 = rng.randrange(i1 - 1, m1)
        if rng.random() < 0.3:
            i2, j2 = 1, 0
        else:
            i2 = rng.randrange(0, m2)
            j2 = rng.randrange(i2, m2)
        candidate = spliced(route1, i1, j1, route2, i2, j2)
        if not candidate:
            continue
        ev = _core.evaluate_splice_warp(
            core, states[r1], i1, j1, states[r2], i2, j2, PENALTY, t_end
        )
        ref = fold_eval(core, candidate, t_end)
        assert ev["total"] == ref["total"], (path.name, route1, i1, j1, route2, i2, j2)
        if not ev["total"]:
            continue
        assert ev["feasible"] == ref["feasible"], (
            path.name,
            route1,
            i1,
            j1,
            route2,
            i2,
            j2,
        )
        if tight:
            assert ev["min_warp"] == pytest.approx(ref["min_warp"], rel=REL_TOL, abs=1e-9)
            assert ev["penalised"] == pytest.approx(ref["penalised"], rel=REL_TOL)
            if ev["feasible"]:
                assert ev["duration"] == pytest.approx(ref["duration"], rel=REL_TOL)
        checked += 1
    assert checked > 100


def test_warp_state_accounting_is_fold(gate_case) -> None:
    """WarpRouteState stores fold-accounted values (repricing rule)."""
    path, _ = gate_case
    core = load_core(path)
    routes = [r for r in greedy_routes(core) if r]
    t_end = core.warp_horizon()
    for r in routes:
        st = _core.build_warp_route_state(core, r, PENALTY, t_end)
        ref = fold_eval(core, r, t_end)
        assert st is not None and ref["total"]
        assert st.min_warp == ref["min_warp"]
        expected = ref["duration"] if ref["feasible"] else ref["penalised"]
        assert st.duration == expected
