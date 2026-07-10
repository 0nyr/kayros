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

from conftest import family_instances, require_benchmarks


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
    reason="M5.9 section 12: the symmetric bidirectional merge (t_m = T/2) "
    "misprices stepwise ATFs — cold certifies 4820 > 4357 on Rifki-25 n=10. "
    "This is the pinned reproducer for the exact value-jump labeling work; "
    "production uses symmetric=false (sound on every gate) until it lands.",
    strict=True,
)
def test_symmetric_merge_stepwise_is_sound(monkeypatch):
    """Under KAYROS_LBL_SYMMETRIC=1 the prover must still certify true optima.

    The uninitialized-`symmetric` UB (fixed in commit 8/n) made this path the
    silent default on some builds (the 2026-07-10 g5k campaign); it is now
    opt-in and unsound until the labeling handles value jumps exactly.
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
