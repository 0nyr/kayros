"""Regression tests for BPC pricing on stepwise and continuous inputs.

Rifki supplies checked solutions below previously reported numerical optima. The tests exercise vertical value jumps, reverse-side normalization and pricing on each build. The randomized cases in test_prover_fuzz_soundness.py complement these fixed witnesses. See cpp/lera/NOTICE.md, item 9, for the history.
"""

from __future__ import annotations

import pytest
from mamut_routing_lib.models import BenchmarkSolution
from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.td import check_td_solution, load_td_instance

from conftest import benchmarks_root, family_instances, instance_path, require_benchmarks
from exact_reference import assert_exact_value, exact_duration_cost


def _pick(paths, name):
    for p in paths:
        if p.name.removesuffix(".vrp.json") == name:
            return p
    return None


def _need(path, label):
    """Skip (do not fail) when a specific instance is absent from the checkout.

    ``require_benchmarks()`` only asserts the benchmarks *root* exists; CI and
    sparse checkouts may carry only a subset (e.g. Dabia n=25), so the
    stepwise reproducers (Rifki2020) can still be missing. These gates then
    skip, consistent with every other benchmark-dependent test in the suite.
    """
    if path is None or not path.exists():
        pytest.skip(f"{label} instance not in the benchmark checkout")
    return path


# Decisive minimal reproducer -------------------------------------------------
RIFKI16_N20 = _pick(family_instances("TDVRPTW", "Rifki2020", ["n=20"]), "Rifki-16")

# Checked routes costing 8,361 undercut the historical native closures at 8,485 and 8,376.
RIFKI16_N20_COUNTEREXAMPLE = [
    [5],
    [8, 13],
    [10, 16, 12, 18, 2, 19],
    [17, 6, 20, 9, 3],
    [7, 4, 15, 11, 14, 1],
]
RIFKI16_N20_COUNTEREXAMPLE_COST = 8361.0

# Jump-free regression guard --------------------------------------------------
C101_N25 = _pick(family_instances("TDVRPTW", "Dabia2013", ["n=25"]), "C101")


def _cold(loaded, tl=60.0):
    from kayros._bpc import solve_duration

    return solve_duration(loaded, time_limit=tl)


def _warm(loaded, routes, tl=60.0):
    from kayros._bpc import solve_duration

    return solve_duration(loaded, time_limit=tl, initial_routes=routes)


def test_reproducer_counterexample_is_checker_valid():
    """Sanity: the counterexample really is a valid, cheaper solution.

    This anchors the soundness gates below — if this fails the reproducer data
    is stale, not the solver.
    """
    require_benchmarks()
    _need(RIFKI16_N20, "Rifki-16 n=20 TDVRPTW")
    loaded = load_td_instance(RIFKI16_N20)
    sol = BenchmarkSolution(
        instance_name="Rifki-16",
        routes=sorted(RIFKI16_N20_COUNTEREXAMPLE, key=lambda r: r[0]),
    )
    chk = check_td_solution(loaded, sol, ObjectiveFunction.DURATION)
    assert chk.is_valid()
    assert chk.routing_cost == RIFKI16_N20_COUNTEREXAMPLE_COST


def test_stepwise_certified_value_is_sound():
    """The native closure value must be at most the checked feasible cost of 8,361."""
    require_benchmarks()
    _need(RIFKI16_N20, "Rifki-16 n=20 TDVRPTW")
    loaded = load_td_instance(RIFKI16_N20)
    res = _cold(loaded)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= RIFKI16_N20_COUNTEREXAMPLE_COST


def test_stepwise_certification_is_warm_start_independent():
    """Cold and warm native searches must reach the same numerical closure on this witness."""
    require_benchmarks()
    _need(RIFKI16_N20, "Rifki-16 n=20 TDVRPTW")
    loaded = load_td_instance(RIFKI16_N20)
    cold = _cold(loaded)
    warm = _warm(loaded, RIFKI16_N20_COUNTEREXAMPLE)
    assert cold["exact_log"]["status"] == "Optimum"
    assert warm["exact_log"]["status"] == "Optimum"
    assert cold["value"] == warm["value"]


# --- The pinned SYMMETRIC-merge defect (target of the exact value-jump work) --


def test_rifki25_stepwise_is_sound():
    """Rifki-25 retains the independently verified objective value.

    History: unsound (uninitialized-`symmetric` UB era, 4820) -> fixed by the
    13.2 tags -> apparently regressed (4820) -> ROOT CAUSE was the pricing
    ladder's unsound termination (memo 13.8: a heuristic level re-pricing an
    existing column stalled column generation before the Exact level ever ran);
    fixed by added-driven escalation in the bridge. Hard gate.
    """
    require_benchmarks()
    src = instance_path("TDVRPTW", "Rifki2020", "n=10", "Rifki-25")
    _need(src, "Rifki-25 n=10 TDVRPTW")
    loaded = load_td_instance(src)
    res = _cold(loaded, tl=60.0)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] == pytest.approx(4357.0, abs=1e-6)


# Shrunk from Rifki-14 n=30 (cold 10207 vs warm 10188 on both platforms, the
# 2026-07-10 two-platform gate's one material failure). k=15 subset, locally
# minimal under single drops. The checker-valid solution below (cost 5572,
# found and certified by the SYMMETRIC tagged-vertical path, cold == warm)
# refutes BOTH asymmetric arms: cold 5606 AND ILS-warm 5580 — so warm-start
# agreement alone cannot certify the asymmetric path, and mode-variation
# (asymmetric vs symmetric) joins the gate family.
RIFKI14_K15_SRC = ("Rifki2020", "n=30", "Rifki-14")
RIFKI14_K15_KEEP = [2, 5, 6, 7, 9, 10, 11, 12, 13, 15, 20, 22, 23, 25, 30]
RIFKI14_K15_TRUE_UB = 5572.0
RIFKI14_K15_WITNESS = [[8, 9, 7, 13, 3, 1], [6, 2, 4, 10, 15], [12, 11], [5, 14]]


def _load_rifki14_k15():
    from td_fuzz import subsample

    fam, size, name = RIFKI14_K15_SRC
    src = instance_path("TDVRPTW", fam, size, name)
    _need(src, "Rifki-14 n=30 TDVRPTW")
    return subsample(load_td_instance(src), RIFKI14_K15_KEEP, "Rifki-14-k15")


def test_rifki14_k15_witness_is_checker_valid():
    """Anchor: the 5572 witness is a valid solution of the k=15 subset."""
    require_benchmarks()
    inst = _load_rifki14_k15()
    cost = exact_duration_cost(inst.instance, inst.atfs, RIFKI14_K15_WITNESS)
    assert float(cost) == pytest.approx(RIFKI14_K15_TRUE_UB, abs=1e-6)


def test_asymmetric_rifki14_k15_is_sound():
    """Asymmetric cold must not certify above the checker-valid 5572.

    FIXED by the added-driven ladder escalation (memo 13.8): the label-trace
    harness showed the witness column's labels were only ever killed at
    HEURISTIC pricing levels, and the level logs showed the Exact level never
    ran — the ladder broke on a non-empty pricing pool whose columns were all
    fingerprint-duplicates, and the BCP read added == 0 as node-optimal. Cold
    and warm now certify 5572 with Exact iterations in the log. Hard gate.
    """
    require_benchmarks()
    inst = _load_rifki14_k15()
    res = _cold(inst, tl=120.0)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= RIFKI14_K15_TRUE_UB + 1e-6


# These three Rifki witnesses exercise waiting on a value jump, path-based deduplication and a stacked departure choice at the end of a piecewise function. The historical reference values are costs of feasible solutions. Rifki-18 may reach its time limit at the test budget.
M130_EXACT_CASES = {
    "Rifki-2": ("n=20", 7378.0, 300.0),
    "Rifki-18": ("n=20", 7127.0, 300.0),
    "Rifki-17": ("n=50", 21319.0, 600.0),
}


def _exact_cold_result(name):
    require_benchmarks()
    size, truth, tl = M130_EXACT_CASES[name]
    src = instance_path("TDVRPTW", "Rifki2020", size, name)
    _need(src, f"{name} {size} TDVRPTW")
    loaded = load_td_instance(src)
    return _cold(loaded, tl=tl), truth


def test_exact_path_rifki2_n20_certifies_truth():
    res, truth = _exact_cold_result("Rifki-2")
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= truth + 1e-6


def test_exact_path_rifki17_n50_certifies_truth():
    res, truth = _exact_cold_result("Rifki-17")
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= truth + 1e-6


def test_exact_path_rifki18_n20_is_sound():
    res, truth = _exact_cold_result("Rifki-18")
    status = res["exact_log"]["status"]
    if status == "Optimum":
        assert res["value"] <= truth + 1e-6
    else:
        # Honest TL: the incumbent must have reached the true optimum and the
        # dual bound must not contradict it (an above-truth certificate is the
        # defect this gate exists for).
        assert status == "TimeLimitReached"
        assert res["exact_log"]["best_int_value"] == pytest.approx(truth, abs=1e-6)
        assert res["exact_log"]["best_bound"] <= truth + 1e-6


# Fuzzer-pinned jump-free guard (2026-07-10): the dominator-side choice-min
# experiment over-certified this subset (7667.4 > 7526); it must stay at the
# ILS-confirmed optimum. Fast (seconds).
C102_K7_SRC = ("Dabia2013", "n=25", "C102")
C102_K7_KEEP = [5, 7, 9, 12, 13, 15, 17]
C102_K7_TRUE = 7525.999728501456  # reference solution cost at display precision


def test_jumpfree_c102_k7_guard():
    from td_fuzz import subsample

    require_benchmarks()
    fam, size, name = C102_K7_SRC
    src = instance_path("TDVRPTW", fam, size, name)
    inst = subsample(load_td_instance(src), C102_K7_KEEP, "C102-k7")
    res = _cold(inst, tl=60.0)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] == pytest.approx(C102_K7_TRUE, abs=1e-9)


# A CHOICE vertical can occur on a jump-free instance. Using the stepwise extension rules on Vu-A5-pA-d90-w40 led to closure about 4.27 above a feasible route set. The route cost below is the regression threshold.
VU_A5PA_D90_W40 = ("Vu2020", "n=59", "Vu-A5-pA-d90-w40")
VU_A5PA_D90_W40_TRUE = 2760.1098340282865


def test_jumpfree_vu2020_certificate_is_not_above_the_truth():
    """Check that cold BPC closure stays below the known feasible Vu2020 cost and that its returned routes attain the reported cost to the test tolerance."""
    require_benchmarks()
    fam, size, name = VU_A5PA_D90_W40
    src = instance_path("TDVRPTW", fam, size, name)
    _need(src, f"{name} {size} TDVRPTW")
    loaded = load_td_instance(src)
    res = _cold(loaded, tl=600.0)
    assert res["stepwise_atfs"] is False  # the premise this gate is about
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= VU_A5PA_D90_W40_TRUE + 1e-6
    from kayros._bpc import routes_to_mamut

    routes = [
        r[1:-1] for r in routes_to_mamut(res["routes"], loaded.instance.num_customers)
    ]
    assert_exact_value(res, exact_duration_cost(loaded.instance, loaded.atfs, routes))


# --- The reverse-side discriminators (2026-08-05) ---------------------------
#
# These two are the instances that catch a regression in the REVERSE instance
# construction, which the Vu-A5-pA-d90-w40 gate above does not: when the
# reverse-side normalizer ``continuize_value_jumps`` was deleted as presumed
# mollifier residue, Vu-A5-pA-d90-w40 kept certifying its exact stored value
# while these two rose above checker-valid stored solutions (1278.2616 against
# 1275.8316, and 1852.9437 against 1846.8070), deterministically and with the
# tree closed. The mechanism is that the reflected reverse arrival carries
# CHOICE verticals (``Inverse`` of a departure plateau, carried through
# ``FlipTime``/``FlipValue``), which the normalizer flattens and which the
# legacy jump-free arithmetic cannot handle. Truths are the stored MAMUT BKS
# costs, each attained by an independently checker-valid solution.
VU_REVERSE_SIDE_CASES = {
    "Vu-A2-pB-d98-w60": 1275.8316212247312,
    "Vu-A2-pB-d98-w100": 1846.8070188618972,
}


@pytest.mark.parametrize("name", sorted(VU_REVERSE_SIDE_CASES))
def test_jumpfree_reverse_side_certificate_is_not_above_the_truth(name):
    """A jump-free cold certificate must not exceed a checker-valid solution.

    Hard soundness gate on the reverse instance construction. Companion to
    ``test_jumpfree_vu2020_certificate_is_not_above_the_truth``, which pins the
    forward/extension side: that one passes on a build whose reverse arrivals
    are wrong, so these two carry the reverse-side signal on their own. As
    there, the returned solution must also price at the certified value under
    the reference checker, so a "sound" value backed by an infeasible route
    cannot pass.
    """
    require_benchmarks()
    truth = VU_REVERSE_SIDE_CASES[name]
    src = instance_path("TDVRPTW", "Vu2020", "n=59", name)
    _need(src, f"{name} n=59 TDVRPTW")
    loaded = load_td_instance(src)
    res = _cold(loaded, tl=600.0)
    assert res["stepwise_atfs"] is False  # the premise this gate is about
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= truth + 1e-6
    from kayros._bpc import routes_to_mamut

    routes = [
        r[1:-1] for r in routes_to_mamut(res["routes"], loaded.instance.num_customers)
    ]
    assert_exact_value(res, exact_duration_cost(loaded.instance, loaded.atfs, routes))


# --- Regression guards: jump-free family proofs must stay correct/stable -----


def test_jumpfree_certification_is_warm_start_independent():
    """Dabia2013 C101 (jump-free): cold and warm certify the same optimum."""
    require_benchmarks()
    assert C101_N25 is not None
    loaded = load_td_instance(C101_N25)
    cold = _cold(loaded, tl=120.0)
    assert cold["exact_log"]["status"] == "Optimum"
    routes = [
        r[1:-1]
        for r in __import__("kayros._bpc", fromlist=["routes_to_mamut"]).routes_to_mamut(
            cold["routes"], loaded.instance.num_customers
        )
    ]
    warm = _warm(loaded, routes, tl=120.0)
    assert warm["exact_log"]["status"] == "Optimum"
    assert cold["value"] == warm["value"]
    # Check the native value against an independent exact evaluation.
    assert_exact_value(cold, exact_duration_cost(loaded.instance, loaded.atfs, routes))


def test_jumpfree_certification_is_deterministic():
    """Two cold solves of the jump-free guard agree bit-for-bit."""
    require_benchmarks()
    assert C101_N25 is not None
    loaded = load_td_instance(C101_N25)
    a = _cold(loaded, tl=120.0)
    b = _cold(loaded, tl=120.0)
    assert a["value"] == b["value"]
    assert a["exact_log"]["status"] == b["exact_log"]["status"] == "Optimum"
