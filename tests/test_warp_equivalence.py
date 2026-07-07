"""P8.1 hard gates: the warp-augmented fold must reduce bitwise to the checker.

Stream 8 (TD time-warp) exact-reduction gates, per the accepted P8.0 design
memo (workspace ``reports/design/p8.0-td-time-warp-design.md`` §4):

- **G1** — on every checker-feasible route, the zero-warp duration equals the
  checker's Delta* bitwise, the minimal warp is exactly 0.0, and the clamped
  ready-time function's breakpoint grid extends the checker's grid (bitwise
  prefix). Durations are asserted, argmins never (association-fragile).
- **G2** — the feasibility predicates agree exactly: checker domain non-empty
  ⇔ zero-warp set non-empty (exactly-zero warp, never a tolerance).
- **G3** — hand-built late routes get controlled warp values.

The C++ channels are additionally asserted bitwise against the pure-Python
reference twin (``tests/warp_reference.py``) built on the canonical lib
NDCPWLF algebra, mirroring the M3.2 checker-equivalence discipline.
"""

import math
import random
import zlib

import pytest
from mamut_routing_lib.td import (
    NDCPWLF,
    compute_route_duration,
    compute_route_ready_time_function,
    load_td_instance,
)

from kayros import _core
from kayros.io import to_core

import warp_reference as ref
from conftest import family_instances
from test_checker_equivalence import random_routes
from test_pwlf_equivalence import random_ndcpwlf

INSTANCES_PER_CASE = 3
EQUIVALENCE_DRAWS = 8
GATE_PENALTY = 10.0

FAMILY_CASES = [
    ("TDVRPTW", "Dabia2013", ["n=25", "n=50"]),
    ("TDVRP", "Dabia2013", ["n=25", "n=50"]),
    ("TDVRPTW", "Ari2018", ["n=15", "n=40"]),
    ("TDVRP", "Ari2018", ["n=15", "n=40"]),
    ("TDVRPTW", "Vu2020", ["n=59"]),
    ("TDVRP", "Vu2020", ["n=59"]),
    ("TDVRPTW", "Rifki2020", ["n=10", "n=30"]),
    ("TDVRP", "Rifki2020", ["n=10", "n=30"]),
]


def cases():
    for problem_type, family, sizes in FAMILY_CASES:
        for size in sizes:
            paths = family_instances(problem_type, family, [size])
            for path in paths[:INSTANCES_PER_CASE]:
                name = path.name.removesuffix(".vrp.json")
                yield pytest.param(path, id=f"{problem_type}-{family}-{size}-{name}")


ALL_CASES = list(cases())

# Aggregate route-outcome counts across the whole parametrized run, so the
# suite can assert the warped (penalised) path was actually exercised without
# requiring lateness on every single instance (some Ari TWs are very loose).
OUTCOME_TOTALS = {"feasible": 0, "warped": 0, "walled": 0}


def assert_route_gates(loaded, core, t_end: float, route: list[int], ctx) -> str:
    """G1 + G2 + twin bit-identity on one route. Returns the outcome class."""
    reference = compute_route_duration(loaded.instance, loaded.atfs, route)
    (rho_xs, rho_ys), (w_xs, w_ys) = core.route_warp_functions(route, t_end)
    twin_rho, twin_warp = ref.warp_route_functions(
        loaded.instance, loaded.atfs, route, t_end
    )
    assert rho_xs == twin_rho.xs and rho_ys == twin_rho.ys, (ctx, route)
    assert w_xs == twin_warp.xs and w_ys == twin_warp.ys, (ctx, route)

    ev = core.evaluate_route_warp(route, GATE_PENALTY, t_end)

    # Safe-dedup neutrality: the dedup'd fold (the search/accounting path)
    # must agree BITWISE on every accounting value, and the dedup'd functions
    # must reproduce every removed breakpoint by interpolation exactly.
    evd = core.evaluate_route_warp(route, GATE_PENALTY, t_end, True)
    assert evd["total"] == ev["total"] and evd["feasible"] == ev["feasible"], (ctx, route)
    if ev["total"]:
        assert evd["min_warp"] == ev["min_warp"], (ctx, route)
        if ev["feasible"]:
            assert evd["duration"] == ev["duration"], (ctx, route)
        (d_rho_xs, d_rho_ys), (d_w_xs, d_w_ys) = core.route_warp_functions(
            route, t_end, True
        )
        assert len(d_rho_xs) <= len(rho_xs) and len(d_w_xs) <= len(w_xs)
        for xs_full, ys_full, dxs, dys in (
            (rho_xs, rho_ys, d_rho_xs, d_rho_ys),
            (w_xs, w_ys, d_w_xs, d_w_ys),
        ):
            prev_x = None
            for x, y in zip(xs_full, ys_full):
                if x == prev_x:
                    continue  # vertical step: evaluate returns the lower value
                prev_x = x
                assert _core.pwlf_evaluate(dxs, dys, x) == y, (ctx, route, x)

    # G2: checker feasibility == (total AND exactly-zero achievable warp).
    assert ev["feasible"] == (ev["total"] and ev["min_warp"] == 0.0), (ctx, route)
    assert ev["feasible"] == reference.feasible, (ctx, route)

    if not reference.feasible:
        if ev["total"]:
            assert ev["min_warp"] > 0.0, (ctx, route)
            # The penalised objective is finite on infeasible routes: this is
            # the whole point of the warp channel (the wall became a value).
            assert math.isfinite(ev["penalised"]), (ctx, route)
            return "warped"
        return "walled"

    # G1: zero-warp duration == checker duration, bitwise.
    assert ev["min_warp"] == 0.0, (ctx, route)
    assert ev["duration"] == reference.duration, (ctx, route)
    # Phi* can only improve on Delta* (larger candidate set, exact at the
    # checker's argmin breakpoint since W there is exactly 0.0).
    assert ev["penalised"] <= reference.duration, (ctx, route)

    # G1 grid form: the checker's delta grid is a bitwise prefix of rho's.
    delta = compute_route_ready_time_function(loaded.instance, loaded.atfs, route)
    n = len(delta.xs)
    assert rho_xs[:n] == delta.xs, (ctx, route)
    assert rho_ys[:n] == delta.ys, (ctx, route)
    return "feasible"


@pytest.mark.parametrize("instance_path", ALL_CASES)
def test_warp_gate_random_routes(instance_path) -> None:
    loaded = load_td_instance(instance_path)
    core = to_core(loaded)
    t_end = core.warp_horizon()
    assert t_end == ref.warp_horizon(loaded.instance, loaded.atfs), instance_path
    rng = random.Random(zlib.crc32(b"warp:" + str(instance_path).encode()))

    outcomes = {"feasible": 0, "warped": 0, "walled": 0}
    for _ in range(EQUIVALENCE_DRAWS):
        for route in random_routes(rng, loaded.instance):
            outcomes[assert_route_gates(loaded, core, t_end, route, instance_path)] += 1
    assert outcomes["feasible"] > 0, (instance_path, outcomes)
    for key, count in outcomes.items():
        OUTCOME_TOTALS[key] += count


def test_warped_path_exercised() -> None:
    """Runs after the parametrized gates: the penalised (warped) regime must
    have been hit many times across the corpus."""
    if not ALL_CASES:
        pytest.skip("benchmarks not available")
    assert OUTCOME_TOTALS["warped"] > 100, OUTCOME_TOTALS


@pytest.mark.parametrize("instance_path", ALL_CASES)
def test_warp_gate_bks_and_deadline_riders(instance_path) -> None:
    """G1 on stored BKS routes; G2 + twin identity on adjacent-swap riders.

    BKS routes ride time windows tightly, and swapping two adjacent customers
    usually produces a *slightly* late route — small positive warp, the regime
    the penalised search lives in.
    """
    from mamut_routing_lib.bks import get_bks_path_for_instance, load_bks
    from mamut_routing_lib.enums import ObjectiveFunction

    bks_path = get_bks_path_for_instance(instance_path, ObjectiveFunction.DURATION)
    if not bks_path.exists():
        pytest.skip("no stored BKS for this instance")
    bks = load_bks(bks_path)
    loaded = load_td_instance(instance_path)
    core = to_core(loaded)
    t_end = core.warp_horizon()

    total = 0.0
    for route in sorted(bks.routes, key=lambda r: r[0]):
        outcome = assert_route_gates(loaded, core, t_end, route, instance_path)
        assert outcome == "feasible", (instance_path, route)
        total += core.evaluate_route_warp(route, GATE_PENALTY, t_end)["duration"]
    assert total == bks.cost, (instance_path, total, bks.cost)

    rng = random.Random(zlib.crc32(b"riders:" + str(instance_path).encode()))
    for route in bks.routes:
        if len(route) < 2:
            continue
        for _ in range(min(4, len(route) - 1)):
            i = rng.randrange(len(route) - 1)
            rider = list(route)
            rider[i], rider[i + 1] = rider[i + 1], rider[i]
            assert_route_gates(loaded, core, t_end, rider, instance_path)


def test_add_equivalence() -> None:
    """C++ pwlf add == Python twin add, bitwise, incl. steps and plateaus."""
    rng = random.Random(20260707)
    non_empty = 0
    for _ in range(500):
        f = random_ndcpwlf(rng)
        g = random_ndcpwlf(rng)
        xs, ys = _core.pwlf_add(f.xs, f.ys, g.xs, g.ys)
        h = ref.add(f, g)
        assert xs == h.xs and ys == h.ys, (f.xs, f.ys, g.xs, g.ys)
        if xs:
            non_empty += 1
    assert non_empty > 100


def test_builders_equivalence() -> None:
    rng = random.Random(20260708)
    for _ in range(200):
        e = rng.uniform(0.0, 100.0)
        latest = e + rng.uniform(0.0, 100.0)
        s = rng.uniform(0.0, 30.0)
        t_end = latest + rng.uniform(0.0, 500.0)
        (t_xs, t_ys), (w_xs, w_ys) = _core.pwlf_make_theta_warp(e, latest, s, t_end)
        theta, warp = ref.make_theta_warp(e, latest, s, t_end)
        assert t_xs == theta.xs and t_ys == theta.ys
        assert w_xs == warp.xs and w_ys == warp.ys
        # Below the deadline theta~ is make_theta bit for bit.
        base_xs, base_ys = _core.pwlf_make_theta(e, latest, s)
        assert t_xs[: len(base_xs)] == base_xs and t_ys[: len(base_ys)] == base_ys
        c_xs, c_ys = _core.pwlf_make_return_clamp(latest, t_end)
        clamp = ref.make_return_clamp(latest, t_end)
        assert c_xs == clamp.xs and c_ys == clamp.ys
        r_xs, r_ys = _core.pwlf_make_return_warp(latest, t_end)
        rwarp = ref.make_return_warp(latest, t_end)
        assert r_xs == rwarp.xs and r_ys == rwarp.ys


# --- G3: hand-built instance with controlled lateness ---------------------

def _toy_instance(customer_tw: tuple[float, float], depot_due: float = 100.0):
    """Depot + one customer, constant travel time 10 in both directions,
    service 5, horizon [0, 100]."""
    arc = ([0.0, 100.0], [10.0, 110.0])  # alpha(t) = t + 10
    return _core.Instance(
        num_customers=1,
        num_vehicles=None,
        vehicle_capacity=100,
        horizon=(0.0, 100.0),
        time_windows=[(0.0, depot_due), customer_tw],
        demands=[0, 1],
        service_times=[0.0, 5.0],
        arcs=[(0, 1, *arc), (1, 0, *arc)],
    )


def test_g3_feasible_boundary() -> None:
    inst = _toy_instance((0.0, 20.0))
    t_end = inst.warp_horizon()
    ev = inst.evaluate_route_warp([1], GATE_PENALTY, t_end)
    assert ev["total"] and ev["feasible"] and ev["min_warp"] == 0.0
    # 10 travel + 5 service + 10 travel back, no waiting anywhere.
    assert ev["duration"] == 25.0
    # Base evaluation agrees (the instance is checker-feasible).
    feasible, duration, _ = inst.evaluate_route([1])
    assert feasible and duration == 25.0
    # The zero-warp set ends exactly where the checker domain ends (t = 10:
    # arrival hits the deadline 20).
    (rho_xs, _), (w_xs, w_ys) = inst.route_warp_functions([1], t_end)
    base_xs, _ = inst.route_ready_time_function([1])
    assert base_xs[-1] == 10.0
    zero_end = max(x for x, y in zip(w_xs, w_ys) if y == 0.0)
    assert zero_end == 10.0
    assert rho_xs[: len(base_xs)] == base_xs


def test_g3_one_unit_late() -> None:
    # Deadline 9: earliest arrival is 10 — every departure is at least 1 late.
    inst = _toy_instance((0.0, 9.0))
    t_end = inst.warp_horizon()
    ev = inst.evaluate_route_warp([1], GATE_PENALTY, t_end)
    assert ev["total"] and not ev["feasible"]
    assert math.isclose(ev["min_warp"], 1.0, rel_tol=0.0, abs_tol=1e-9)
    assert ev["min_warp"] > 0.0
    feasible, _, _ = inst.evaluate_route([1])
    assert not feasible
    # Penalised objective at the earliest departure: clamped duration
    # (10 -> clamp 9 + 5 service + 10 return = 24) + P * 1.
    assert math.isclose(
        ev["penalised"], 24.0 + GATE_PENALTY * 1.0, rel_tol=0.0, abs_tol=1e-8
    )


def test_g3_depot_return_warp() -> None:
    # Customer window is wide, depot closes at 20: return arrives at >= 25.
    inst = _toy_instance((0.0, 90.0), depot_due=20.0)
    t_end = inst.warp_horizon()
    ev = inst.evaluate_route_warp([1], GATE_PENALTY, t_end)
    assert ev["total"] and not ev["feasible"]
    assert math.isclose(ev["min_warp"], 5.0, rel_tol=0.0, abs_tol=1e-9)
    feasible, _, _ = inst.evaluate_route([1])
    assert not feasible


def test_g3_warp_grows_with_lateness() -> None:
    # Tightening the deadline by k adds exactly k to the minimal warp.
    warps = []
    for deadline in (9.0, 8.0, 5.0, 2.0):
        inst = _toy_instance((0.0, deadline))
        t_end = inst.warp_horizon()
        warps.append(inst.evaluate_route_warp([1], GATE_PENALTY, t_end)["min_warp"])
    for prev, cur in zip(warps, warps[1:]):
        assert cur > prev
    assert math.isclose(warps[-1], 8.0, rel_tol=0.0, abs_tol=1e-9)
