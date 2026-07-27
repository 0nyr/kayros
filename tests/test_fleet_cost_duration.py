"""M7 (Plan 11 Blauth2024 campaign): FleetCostDuration in the heuristic stack.

The lib checker stays the authority: every kayros FleetCostDuration value must
equal ``compute_solution_cost`` / ``check_td_solution`` under
``objective_function=FLEET_COST_DURATION`` with ``==``, never an epsilon.
These are the FleetCostDuration legs of the checker-equivalence and ILS gates:

- fold equivalence: canonical duration fold + one ``F * K`` multiply-add, on
  real instances with an injected ``fleet_fixed_cost``;
- Duration orthogonality: an instance CARRYING ``fleet_fixed_cost`` solved
  under Duration behaves bitwise like one without it (F ignored, rng streams
  untouched);
- LS emptying credit: relocates that empty a route are credited F, so
  duration-increasing merges worth up to F are taken under FleetCostDuration
  and refused under Duration;
- route-dissolve kick: armed only when the core prices routes; inert (bitwise)
  under Duration;
- solve() API: objective selection, misuse guards, checker equality by
  construction;
- fleet_stats counters (Plan 12 M1): dissolve-kick lifecycle diagnostics,
  self-consistent identities on a priced solve, None under Duration,
  deterministic per seed;
- Blauth-berlin n=10 smoke (skipped unless the converted family is reachable):
  the 10 h fleet penalty must actually buy route dissolves.
"""

import random

import pytest
from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.td import (
    LoadedTDInstance,
    compute_solution_cost,
    load_td_instance,
)

import kayros
from kayros import _core
from kayros.io import canonical_objective, to_core

from conftest import blauth2024_instances, family_instances

BLAUTH_F = 36000000.0  # the family's 10 h fixed cost, integer milliseconds

DABIA_25 = family_instances("TDVRPTW", "Dabia2013", ["n=25"])
BLAUTH_10 = blauth2024_instances("n=10")


def with_fleet_cost(
    loaded: LoadedTDInstance, f: float, *, unlimited_fleet: bool = False
) -> LoadedTDInstance:
    """The same loaded instance, carrying ``fleet_fixed_cost = f`` (and an
    unlimited fleet when asked — Blauth-style, lets partitions vary K)."""
    update: dict = {"fleet_fixed_cost": f}
    if unlimited_fleet:
        update["num_vehicles"] = None
    return LoadedTDInstance(
        instance=loaded.instance.model_copy(update=update),
        atfs=loaded.atfs,
        instance_path=loaded.instance_path,
        atf_path=loaded.atf_path,
        categories_path=None,
    )


def greedy_routes(core):
    ok, routes = _core.greedy_makespan(core)
    assert ok
    return routes


# --- objective plumbing -----------------------------------------------------


def test_canonical_objective_spellings() -> None:
    for spelling in ("duration", "Duration", ObjectiveFunction.DURATION):
        assert canonical_objective(spelling) == "Duration"
    for spelling in (
        "fleet_cost_duration",
        "FleetCostDuration",
        ObjectiveFunction.FLEET_COST_DURATION,
    ):
        assert canonical_objective(spelling) == "FleetCostDuration"
    with pytest.raises(ValueError):
        canonical_objective("makespan")


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_to_core_objective_selects_fixed_route_cost(instance_path) -> None:
    loaded = with_fleet_cost(load_td_instance(instance_path), BLAUTH_F)
    assert to_core(loaded).fixed_route_cost == 0.0  # Duration ignores F
    core = to_core(loaded, objective_function="fleet_cost_duration")
    assert core.fixed_route_cost == BLAUTH_F


# --- fold equivalence (checker-equivalence leg) -------------------------------


def random_split(rng: random.Random, routes: list[list[int]]) -> list[list[int]]:
    """A random refinement of a feasible solution: every output route is a
    contiguous subsequence of an input route, hence feasible (the perturb
    module's invariant), and K varies with the draws."""
    out: list[list[int]] = []
    for route in routes:
        i = 0
        while i < len(route):
            j = rng.randint(i + 1, len(route))
            out.append(route[i:j])
            i = j
    return out


@pytest.mark.parametrize(
    "instance_path", DABIA_25, ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_fold_equals_checker(instance_path) -> None:
    """Feasible partitions of varying K (random refinements of the greedy
    solution, fleet bound lifted): core FleetCostDuration == lib bitwise, and
    == the Duration fold + F * K exactly (one multiply, one add)."""
    loaded = with_fleet_cost(
        load_td_instance(instance_path), BLAUTH_F, unlimited_fleet=True
    )
    core_f = to_core(loaded, objective_function="fleet_cost_duration")
    core_0 = to_core(loaded)
    ok, greedy = _core.greedy_makespan(core_0)
    assert ok
    rng = random.Random(0xF1EE7)

    fleet_sizes = set()
    for trial in range(20):
        routes = greedy if trial == 0 else random_split(rng, greedy)
        fleet_sizes.add(len(routes))
        duration_total = _core.solution_duration(core_0, routes)
        assert duration_total != float("inf"), (instance_path, routes)
        got = _core.solution_duration(core_f, routes)
        want = compute_solution_cost(
            loaded.instance,
            loaded.atfs,
            routes,
            objective_function=ObjectiveFunction.FLEET_COST_DURATION,
        )
        assert got == want, (instance_path, routes)
        assert got == duration_total + BLAUTH_F * len(routes)
    assert len(fleet_sizes) > 1, "splits never varied K — generator regression"


# --- LS emptying credit -------------------------------------------------------


def _synthetic_core(fixed_route_cost: float):
    """Three customers, constant travel times, geometry chosen so the merge
    of the singleton [3] into [1, 2] costs +200 duration (2900 vs 2100 + 600):
    customers 1-2 sit in a far cluster (1000 out, 100 between), customer 3
    is near the depot (300) but far from the cluster (1500). Real instances
    never exhibit this cleanly (a removed client's own route is always the
    duration-best return), hence the synthetic isolation."""
    taus = {
        (0, 1): 1000, (1, 0): 1000, (0, 2): 1000, (2, 0): 1000,
        (1, 2): 100, (2, 1): 100,
        (0, 3): 300, (3, 0): 300,
        (1, 3): 1500, (3, 1): 1500, (2, 3): 1500, (3, 2): 1500,
    }
    h = 100000.0
    arcs = [
        (i, j, [0.0, h], [float(t), h + float(t)]) for (i, j), t in taus.items()
    ]
    return _core.Instance(
        num_customers=3,
        num_vehicles=None,
        vehicle_capacity=3,
        horizon=(0.0, h),
        time_windows=[(0.0, h)] * 4,
        demands=[0, 1, 1, 1],
        service_times=[0.0] * 4,
        arcs=arcs,
        fixed_route_cost=fixed_route_cost,
    )


def test_ls_emptying_credit_isolated() -> None:
    """The commit accountant's F·Δ(route count) term, isolated: the singleton
    merge is duration-INCREASING (+200), so the Duration descent must refuse
    it and the priced descent must take it (200 < F), crediting exactly one F
    in the final value."""
    start = [[1, 2], [3]]

    core_0 = _synthetic_core(0.0)
    kept, value_0, *_ = _core.ls_local_search(core_0, start, num_neighbours=0)
    assert value_0 == 2700.0  # 2100 + 600: the split IS the Duration optimum
    assert len(kept) == 2

    f = 1.0e6
    core_f = _synthetic_core(f)
    merged, value_f, *_ = _core.ls_local_search(core_f, start, num_neighbours=0)
    assert len(merged) == 1
    assert sorted(merged[0]) == [1, 2, 3]
    assert value_f == 2900.0 + f  # merged duration + exactly one F
    assert value_f == _core.solution_duration(core_f, merged)
    # And under the priced fold the split start was worth 2700 + 2 F.
    assert _core.solution_duration(core_f, start) == 2700.0 + 2 * f


# --- route-dissolve kick -------------------------------------------------------


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_perturb_dissolve_inert_under_duration(instance_path) -> None:
    """F == 0 arms nothing: no draw is consumed, kicks are bitwise the
    pre-M7 ones whatever dissolve_pct says."""
    core = to_core(load_td_instance(instance_path))
    routes = greedy_routes(core)
    a = _core.ls_perturb(core, routes, 123, dissolve_pct=0)
    b = _core.ls_perturb(core, routes, 123, dissolve_pct=100)
    assert a == b
    assert a[-1] is False  # never reported dissolved


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_perturb_dissolve_applies_on_priced_core(instance_path) -> None:
    loaded = with_fleet_cost(load_td_instance(instance_path), BLAUTH_F)
    core = to_core(loaded, objective_function="fleet_cost_duration")
    routes = greedy_routes(core)
    assert len(routes) >= 2, "pick must give a multi-route greedy seed"
    smallest = min(len(r) for r in routes)
    served = sorted(c for r in routes for c in r)
    new_routes, value, applied, removed, redraws, new_r, dissolved = _core.ls_perturb(
        core, routes, 7, dissolve_pct=100
    )
    assert applied and dissolved
    assert removed >= smallest, "the dissolve seed removes a whole route"
    assert sorted(c for r in new_routes for c in r) == served
    assert value == _core.solution_duration(core, new_routes)
    # Determinism across identical calls.
    again = _core.ls_perturb(core, routes, 7, dissolve_pct=100)
    assert again[0] == new_routes and again[1] == value


# --- solve() API ----------------------------------------------------------------


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_rejects_missing_fleet_fixed_cost(instance_path) -> None:
    with pytest.raises(kayros.KayrosError, match="fleet_fixed_cost"):
        kayros.solve(
            load_td_instance(instance_path),
            kayros.Params(objective="fleet_cost_duration", ils_max_iterations=5),
        )


def test_solve_rejects_unknown_objective() -> None:
    if not DABIA_25:
        pytest.skip("benchmarks not reachable")
    with pytest.raises(ValueError, match="objective"):
        kayros.solve(
            load_td_instance(DABIA_25[0]), kayros.Params(objective="makespan")
        )


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:2], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_fleet_cost_duration_end_to_end(instance_path) -> None:
    """ILS under FleetCostDuration: Solution carries the objective, the cost
    is the lib's bitwise (already asserted internally by construction), and
    the artifact metadata declares the objective for the BKS pipeline."""
    loaded = with_fleet_cost(load_td_instance(instance_path), BLAUTH_F)
    params = kayros.Params(objective="fleet_cost_duration", ils_max_iterations=150)
    solution = kayros.solve(loaded, params, seed=3)
    assert solution.objective == "FleetCostDuration"
    assert solution.duration == compute_solution_cost(
        loaded.instance,
        loaded.atfs,
        solution.routes,
        objective_function=ObjectiveFunction.FLEET_COST_DURATION,
    )
    artifact = solution.to_benchmark_solution()
    assert artifact.metadata["objective_function"] == "FleetCostDuration"
    assert artifact.cost == solution.duration


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_duration_solve_unchanged_by_carried_field(instance_path) -> None:
    """Regression guarantee: carrying fleet_fixed_cost changes NOTHING under
    Duration — same routes, same value, same incumbent stream (F is an exact
    fold no-op and the dissolve kick consumes no draw)."""
    plain = load_td_instance(instance_path)
    carrying = with_fleet_cost(plain, BLAUTH_F)
    params = kayros.Params(ils_max_iterations=120)
    a = kayros.solve(plain, params, seed=5)
    b = kayros.solve(carrying, params, seed=5)
    assert a.routes == b.routes
    assert a.duration == b.duration
    assert [i.value for i in a.incumbents] == [i.value for i in b.incumbents]
    assert b.objective == "Duration"
    assert "objective_function" not in b.to_benchmark_solution().metadata


# --- fleet_stats counters (Plan 12 M1) --------------------------------------------


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_fleet_stats_none_under_duration(instance_path) -> None:
    """Under Duration the dissolve is never armed, so the counters carry no
    signal and the Solution surface stays the pre-M1 one (None)."""
    solution = kayros.solve(
        load_td_instance(instance_path), kayros.Params(ils_max_iterations=50), seed=1
    )
    assert solution.fleet_stats is None


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_fleet_stats_self_consistent_on_priced_solve(instance_path) -> None:
    """Counter identities that hold by construction: one perturb per ILS
    iteration, every applied kick is dissolved xor normal, every dissolved
    kick is classified exactly once at both capture points, a new best is
    always LAHC-accepted, and the descent never raises K (so kick-drops
    survive to the judgment and kick-raises can only shrink)."""
    loaded = with_fleet_cost(load_td_instance(instance_path), BLAUTH_F)
    params = kayros.Params(objective="fleet_cost_duration", ils_max_iterations=300)
    solution = kayros.solve(loaded, params, seed=11)
    fks = solution.fleet_stats
    assert fks is not None
    assert all(v >= 0 for v in fks.values())
    assert fks["kicks_total"] == solution.iterations
    assert fks["kicks_applied"] <= fks["kicks_total"]
    assert fks["kicks_applied"] == fks["dissolved_armed"] + fks["normal_kicks"]
    assert fks["dissolved_armed"] > 0  # dissolve_pct=50 across 300 iterations
    assert fks["dissolved_armed"] == (
        fks["k_after_kick_lt"] + fks["k_after_kick_eq"] + fks["k_after_kick_gt"]
    )
    assert fks["dissolved_armed"] == (
        fks["k_after_descent_lt"]
        + fks["k_after_descent_eq"]
        + fks["k_after_descent_gt"]
    )
    assert fks["k_after_descent_lt"] >= fks["k_after_kick_lt"]
    assert fks["k_after_descent_gt"] <= fks["k_after_kick_gt"]
    assert fks["dissolved_accepted_lahc"] <= fks["dissolved_armed"]
    assert fks["dissolved_new_best"] <= fks["dissolved_accepted_lahc"]
    assert fks["dissolved_new_best_k_lt"] <= fks["dissolved_new_best"]
    assert fks["normal_accepted_lahc"] <= fks["normal_kicks"]
    assert fks["normal_new_best"] <= fks["normal_accepted_lahc"]


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_fleet_stats_deterministic(instance_path) -> None:
    loaded = with_fleet_cost(load_td_instance(instance_path), BLAUTH_F)
    params = kayros.Params(objective="fleet_cost_duration", ils_max_iterations=120)
    a = kayros.solve(loaded, params, seed=4)
    b = kayros.solve(loaded, params, seed=4)
    assert a.fleet_stats == b.fleet_stats


# --- Blauth-berlin n=10 smoke ----------------------------------------------------


def _blauth_pick(prefix: str):
    paths = [p for p in BLAUTH_10 if p.name.startswith(prefix)]
    if not paths:
        pytest.skip(f"{prefix}* not in the reachable Blauth2024 n=10 set")
    return paths[0]


def _solve_both(loaded):
    params_f = kayros.Params(objective="fleet_cost_duration", ils_max_iterations=400)
    params_d = kayros.Params(ils_max_iterations=400)
    return (
        kayros.solve(loaded, params_f, seed=0),
        kayros.solve(loaded, params_d, seed=0),
    )


def test_blauth_berlin_n10_smoke() -> None:
    """Kickstart-named smoke on the real family: exact contract facts (F is
    the 10 h upstream fixed cost, unlimited fleet), bitwise checker equality,
    and a fleet no worse than the Duration solve's. Berlin is fleet-minimal
    under BOTH objectives (session-41 probe: K = 1 each), so the strict
    trade-off legs live on cincinnati/san_francisco below."""
    loaded = load_td_instance(_blauth_pick("Blauth-berlin"))
    assert loaded.instance.fleet_fixed_cost == BLAUTH_F
    assert loaded.instance.num_vehicles is None

    seed_k = len(greedy_routes(to_core(loaded)))
    sol_f, sol_d = _solve_both(loaded)

    assert sol_f.objective == "FleetCostDuration"
    assert sol_f.duration == compute_solution_cost(
        loaded.instance,
        loaded.atfs,
        sol_f.routes,
        objective_function=ObjectiveFunction.FLEET_COST_DURATION,
    )
    assert sol_f.num_routes <= sol_d.num_routes
    assert sol_f.num_routes <= seed_k
    # Cross-scoring: the dedicated solve must not lose to the Duration
    # solve's routes under its own objective (400 iterations on n=10).
    assert sol_f.duration <= compute_solution_cost(
        loaded.instance,
        loaded.atfs,
        sol_d.routes,
        objective_function=ObjectiveFunction.FLEET_COST_DURATION,
    )


@pytest.mark.parametrize("prefix", ["Blauth-cincinnati", "Blauth-san_francisco"])
def test_blauth_n10_fleet_tradeoff_exercised(prefix) -> None:
    """The fleet trade-off, strictly: on these cities the Duration optimum
    uses MORE routes than the FleetCostDuration optimum (session-41 probe:
    cincinnati 2 vs 1, san_francisco 3 vs 2), so the FCD solve must have
    dissolved a route the Duration objective keeps — paying a duration
    penalty that stays under the 10 h fixed cost, exactly the mechanism the
    emptying credit + dissolve kick exist for."""
    loaded = load_td_instance(_blauth_pick(prefix))
    sol_f, sol_d = _solve_both(loaded)

    assert sol_f.num_routes < sol_d.num_routes
    # The merge sacrificed duration (the dissolve is duration-increasing) …
    duration_f = compute_solution_cost(
        loaded.instance, loaded.atfs, sol_f.routes
    )
    assert duration_f > sol_d.duration
    # … but by less than F per dissolved route, so it wins under FCD.
    saved_routes = sol_d.num_routes - sol_f.num_routes
    assert duration_f - sol_d.duration < BLAUTH_F * saved_routes
    assert sol_f.duration <= compute_solution_cost(
        loaded.instance,
        loaded.atfs,
        sol_d.routes,
        objective_function=ObjectiveFunction.FLEET_COST_DURATION,
    )


def test_blauth_preview_reachable_or_skipped() -> None:
    if not BLAUTH_10:
        pytest.skip(
            "Blauth2024 n=10 not reachable: set KAYROS_BLAUTH2024_DIR to the "
            "converted family directory (or mount it in the benchmarks tree)"
        )
    assert any(p.name.startswith("Blauth-berlin") for p in BLAUTH_10)
