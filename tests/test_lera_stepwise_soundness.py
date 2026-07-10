"""Soundness gates for exact-BPC certification on stepwise (value-jump) ATFs.

Context (M5.9). The vendored Lera BPC is *unsound* on stepwise travel-time
functions (Rifki2020 and any future step-carrying family): value jumps are
mollified into 1e-3-wide steep bridge segments, whose ~1000*step-height slope
turns the labeling's epsilon arithmetic and domain clamps into O(step-height)
merge/dominance mispricing. Negative-reduced-cost columns are silently dropped
and the branch-and-price closes at a wrong, warm-start-dependent "optimum".
See ``cpp/lera/NOTICE.md`` item 9 and the 2026-07-08 Rifki retraction report.

These tests are the *specification* of the fix. The M5.9 numerics fix (exact
coordinate-swap ``Inverse`` for non-decreasing functions) made them pass on the
local build: the pricing incompleteness that certified 8376 > 8361 is gone
here, so the gates below are hard gates. **They are necessary, not sufficient**:
the 2026-07-10 g5k re-certification showed the residual mispricing is
BUILD-DEPENDENT (bit-identical payloads certify differently under gcc-13/Debian
vs the local NixOS toolchain; e.g. Rifki-25 n=10 cold 4820 vs true 4357 on g5k
only). These gates therefore pin local-build soundness against regressions;
they do not certify the prover on stepwise ATFs. See NOTICE item 9 and the
2026-07-10 campaign report. The guard tests (jump-free family) must stay green
throughout. Randomized coverage: ``test_prover_fuzz_soundness.py``.

The decisive minimal reproducer is TDVRPTW/Rifki2020/n=20/Rifki-16: cold solve
certifies ``Optimum 8376`` while the checker-valid solution below carries cost
8361 (a 124-unit / 1.46% refutation), and the certified value is warm-start
dependent (8485 / 8376 / 8361 from stored-BKS / cold / counterexample starts).
"""

from __future__ import annotations

import pytest
from mamut_routing_lib.models import BenchmarkSolution
from mamut_routing_lib.td import (
    check_td_solution,
    compute_solution_cost,
    load_td_instance,
)

from conftest import benchmarks_root, family_instances, require_benchmarks


def _pick(paths, name):
    for p in paths:
        if p.name.removesuffix(".vrp.json") == name:
            return p
    return None


# Decisive minimal reproducer -------------------------------------------------
RIFKI16_N20 = _pick(family_instances("TDVRPTW", "Rifki2020", ["n=20"]), "Rifki-16")

# A checker-valid solution strictly below the M5.7 "proven optimum" 8485 and
# below the cold-start "optimum" 8376 (M7.4 ILS counterexample; retraction
# report 2026-07-08). Its exact checker cost is asserted in-test.
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
    from kayros.lera import solve_duration

    return solve_duration(loaded, time_limit_s=tl)


def _warm(loaded, routes, tl=60.0):
    from kayros.lera import solve_duration

    return solve_duration(loaded, time_limit_s=tl, initial_routes=routes)


def test_reproducer_counterexample_is_checker_valid():
    """Sanity: the counterexample really is a valid, cheaper solution.

    This anchors the soundness gates below — if this fails the reproducer data
    is stale, not the solver.
    """
    require_benchmarks()
    assert RIFKI16_N20 is not None, "Rifki-16 n=20 TDVRPTW instance missing"
    loaded = load_td_instance(RIFKI16_N20)
    sol = BenchmarkSolution(
        instance_name="Rifki-16",
        routes=sorted(RIFKI16_N20_COUNTEREXAMPLE, key=lambda r: r[0]),
    )
    chk = check_td_solution(loaded, sol)
    assert chk.is_valid()
    assert chk.routing_cost == RIFKI16_N20_COUNTEREXAMPLE_COST


def test_stepwise_certified_value_is_sound():
    """A certified optimum must not exceed a known checker-valid solution.

    Cold solve of the reproducer: the certified value must be <= 8361 (the
    checker cost of a feasible solution). Fixed in M5.9 (exact non-decreasing
    ``Inverse`` removed the pricing incompleteness that made the mollified path
    certify 8376); was strict-xfail before the fix.
    """
    require_benchmarks()
    assert RIFKI16_N20 is not None
    loaded = load_td_instance(RIFKI16_N20)
    res = _cold(loaded)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= RIFKI16_N20_COUNTEREXAMPLE_COST


def test_stepwise_certification_is_warm_start_independent():
    """A sound proof cannot depend on the warm start.

    Cold and counterexample-warm solves must both certify Optimum at the same
    value. Before M5.9: 8376 (cold) vs 8361 (warm) -- warm-start dependence was
    the decisive soundness signature; now both certify 8361.
    """
    require_benchmarks()
    assert RIFKI16_N20 is not None
    loaded = load_td_instance(RIFKI16_N20)
    cold = _cold(loaded)
    warm = _warm(loaded, RIFKI16_N20_COUNTEREXAMPLE)
    assert cold["exact_log"]["status"] == "Optimum"
    assert warm["exact_log"]["status"] == "Optimum"
    assert cold["value"] == warm["value"]


# --- The pinned SYMMETRIC-merge defect (target of the exact value-jump work) --


@pytest.mark.xfail(
    reason="M5.9: symmetric-mode regression on Rifki-25 (cold 4820 vs 4357) "
    "introduced by the vertical-aware Max/Min iteration that made the "
    "PRODUCTION (asymmetric) path sound on every pinned reproducer. Symmetric "
    "is an opt-in gate mode; fixing it restores the mode-variation gate's "
    "second witness. k=15 stays sound in symmetric mode.",
    strict=True,
)
def test_symmetric_merge_stepwise_is_sound(monkeypatch):
    """Under KAYROS_LBL_SYMMETRIC=1 the prover must certify the true optimum.

    History: unsound (uninitialized-`symmetric` UB era, 4820) -> fixed by the
    13.2 tags (4357) -> regressed by the Max/Min iteration (4820 again) while
    the same iteration made production asymmetric sound on both pinned
    reproducers. Strict-xfail pins the regression for the follow-up.
    """
    require_benchmarks()
    from conftest import benchmarks_root

    src = benchmarks_root() / "TDVRPTW" / "Rifki2020" / "n=10" / "Rifki-25.vrp.json"
    assert src.exists()
    monkeypatch.setenv("KAYROS_LBL_SYMMETRIC", "1")
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
    src = benchmarks_root() / "TDVRPTW" / fam / size / f"{name}.vrp.json"
    return subsample(load_td_instance(src), RIFKI14_K15_KEEP, "Rifki-14-k15")


def test_rifki14_k15_witness_is_checker_valid():
    """Anchor: the 5572 witness is a valid solution of the k=15 subset."""
    require_benchmarks()
    inst = _load_rifki14_k15()
    from mamut_routing_lib.td import compute_solution_cost

    cost = compute_solution_cost(inst.instance, inst.atfs, RIFKI14_K15_WITNESS)
    assert cost == pytest.approx(RIFKI14_K15_TRUE_UB, abs=1e-6)


@pytest.mark.xfail(
    reason="M5.9: the open ASYMMETRIC production defect — k=15 certifies 5606 "
    "cold / 5608 warm vs the checker-valid 5572. A dominator-side choice-min "
    "briefly made this pass but was itself unsound (over-dominated jump-free "
    "C102) and was reverted; the pass was an artifact. Still the pinned target.",
    strict=True,
)
def test_asymmetric_rifki14_k15_is_sound():
    """Asymmetric cold must not certify above the checker-valid 5572."""
    require_benchmarks()
    inst = _load_rifki14_k15()
    res = _cold(inst, tl=120.0)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= RIFKI14_K15_TRUE_UB + 1e-6


def test_symmetric_rifki14_k15_is_sound():
    """The symmetric tagged-vertical path certifies the true 5572 (hard gate)."""
    require_benchmarks()
    import os as _os

    _os.environ["KAYROS_LBL_SYMMETRIC"] = "1"
    try:
        inst = _load_rifki14_k15()
        res = _cold(inst, tl=120.0)
    finally:
        _os.environ.pop("KAYROS_LBL_SYMMETRIC", None)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] == pytest.approx(RIFKI14_K15_TRUE_UB, abs=1e-6)


# Fuzzer-pinned jump-free guard (2026-07-10): the dominator-side choice-min
# experiment over-certified this subset (7667.4 > 7526); it must stay at the
# ILS-confirmed optimum. Fast (seconds).
C102_K7_SRC = ("Dabia2013", "n=25", "C102")
C102_K7_KEEP = [5, 7, 9, 12, 13, 15, 17]
C102_K7_TRUE = 7525.999728501456  # checker-exact; ILS matches at display precision


def test_jumpfree_c102_k7_guard():
    from td_fuzz import subsample

    require_benchmarks()
    fam, size, name = C102_K7_SRC
    src = benchmarks_root() / "TDVRPTW" / fam / size / f"{name}.vrp.json"
    inst = subsample(load_td_instance(src), C102_K7_KEEP, "C102-k7")
    res = _cold(inst, tl=60.0)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] == pytest.approx(C102_K7_TRUE, abs=1e-9)


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
        for r in __import__("kayros.lera", fromlist=["routes_to_mamut"]).routes_to_mamut(
            cold["routes"], loaded.instance.num_customers
        )
    ]
    warm = _warm(loaded, routes, tl=120.0)
    assert warm["exact_log"]["status"] == "Optimum"
    assert cold["value"] == warm["value"]
    # And the reported value is checker-exact.
    assert cold["value"] == compute_solution_cost(loaded.instance, loaded.atfs, routes)


def test_jumpfree_certification_is_deterministic():
    """Two cold solves of the jump-free guard agree bit-for-bit."""
    require_benchmarks()
    assert C101_N25 is not None
    loaded = load_td_instance(C101_N25)
    a = _cold(loaded, tl=120.0)
    b = _cold(loaded, tl=120.0)
    assert a["value"] == b["value"]
    assert a["exact_log"]["status"] == b["exact_log"]["status"] == "Optimum"
