"""Randomized differential soundness of the exact prover vs the ILS heuristic.

Instance-level analogue of the ``pwlf_compare`` random-curve tests: random small
TD instances (subsampled from real benchmarks, value-jump ATFs preserved) are
solved by both the exact BPC prover and the ILS heuristic, and the two soundness
invariants S1/S2 (see ``td_fuzz``) are asserted.

- ``test_rifki22_reproducer_*`` pin a *fixed* minimal reproducer discovered by
  the fuzzer (Rifki-22 n=20, customers {4,6,8,10,12,13,15,19}): before M5.9 cold
  certified ``Optimum 2504`` while the true optimum is ``2496`` (ILS and warm
  both reach it). Strict-xfail until the exact-``Inverse`` fix landed; now hard
  gates. NOTE (2026-07-10): these gates pin LOCAL-BUILD soundness only; the g5k
  re-certification showed the residual mispricing is build-dependent (identical
  payloads certify differently under another toolchain). See NOTICE item 9.
- ``test_fuzz_batch`` is the broad sweep. It is opt-in (set ``KAYROS_FUZZ=1``)
  because each case runs a full solve; it prints every unsound case so new
  reproducers can be pinned.

Run the sweep with:
    KAYROS_FUZZ=1 KAYROS_STEP_EXACT=1 pytest tests/test_prover_fuzz_soundness.py -s
"""
from __future__ import annotations

import os
import random

import pytest

from conftest import benchmarks_root, require_benchmarks
from td_fuzz import random_case, run_exact, run_ils, soundness, subsample

# Pinned minimal reproducer (fuzzer seed 7, case 06).
RIFKI22_SRC = ("Rifki2020", "n=20", "Rifki-22")
RIFKI22_KEEP = [4, 6, 8, 10, 12, 13, 15, 19]
RIFKI22_TRUE_OPT = 2496.0
RIFKI22_COLD_WRONG = 2504.0


def _load_rifki22():
    from mamut_routing_lib.td import load_td_instance

    root = benchmarks_root()
    fam, size, name = RIFKI22_SRC
    src = root / "TDVRPTW" / fam / size / f"{name}.vrp.json"
    return subsample(load_td_instance(src), RIFKI22_KEEP, "Rifki-22-repro")


def test_rifki22_reproducer_ils_finds_true_optimum():
    """Anchor: ILS reaches 2496 -- the certified 2504 is provably too high."""
    require_benchmarks()
    inst = _load_rifki22()
    cost, _ = run_ils(inst, seed=1, tl=8.0)
    assert cost == pytest.approx(RIFKI22_TRUE_OPT, abs=1e-3)


def test_rifki22_reproducer_cold_is_sound():
    """Cold certified optimum must not exceed the true optimum 2496.

    Fixed in M5.9 (exact non-decreasing ``Inverse``); cold certified 2504 before.
    """
    require_benchmarks()
    inst = _load_rifki22()
    res = run_exact(inst, tl=40.0)
    assert res["exact_log"]["status"] == "Optimum"
    assert res["value"] <= RIFKI22_TRUE_OPT + 1e-6


def test_rifki22_reproducer_warm_start_independent():
    """Cold and optimal-warm solves must certify the same value.

    Fixed in M5.9; was 2504 (cold) vs 2496 (warm) before.
    """
    require_benchmarks()
    inst = _load_rifki22()
    cold = run_exact(inst, tl=40.0)
    warm = run_exact(inst, tl=40.0, warm=[[4, 1, 6, 7, 8, 5], [3, 2]])
    assert cold["exact_log"]["status"] == warm["exact_log"]["status"] == "Optimum"
    assert cold["value"] == pytest.approx(warm["value"], abs=1e-6)


@pytest.mark.skipif(
    os.environ.get("KAYROS_FUZZ") != "1",
    reason="opt-in randomized sweep; set KAYROS_FUZZ=1 (slow: full solve per case)",
)
@pytest.mark.parametrize("seed0", [7, 11, 23])
def test_fuzz_batch(seed0):
    """Random subsampled instances must all be sound (S1 and S2)."""
    require_benchmarks()
    root = benchmarks_root()
    rng = random.Random(seed0)
    n_cases = int(os.environ.get("KAYROS_FUZZ_CASES", "8"))
    unsound = []
    for c in range(n_cases):
        case = random_case(rng, root)
        if case is None:
            continue
        family, size, name, inst = case
        r = soundness(inst, seed=seed0 + c, tl_ils=5.0, tl_exact=30.0)
        tag = "OK " if r["sound"] else "*** UNSOUND"
        print(f"[{seed0}:{c:02d}] {family:10s} k={inst.instance.num_customers} "
              f"{name[:36]:36s} ILS={r['ils_cost']:8.1f} "
              f"cold={r['cold_status'][:4]}:{r['cold_value']:8.1f} "
              f"warm={r['warm_status'][:4]}:{r['warm_value']:8.1f} {tag}")
        if not r["sound"]:
            unsound.append((name, r["ils_cost"], r["cold_value"], r["warm_value"]))
    assert not unsound, f"unsound instances: {unsound}"
