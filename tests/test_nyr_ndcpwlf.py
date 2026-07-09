"""Unit gates for the step-capable NDCPWLF primitives (M5.9).

These pin the exact-jump semantics of ``nyr::NDCPWLF`` before it replaces
``goc::PWLFunction`` inside the labeling (design memo
``reports/design/m5.9-exact-stepwise-labeling-design.md``). The authoritative
convention is the canonical checker's: evaluation is **left-continuous** at a
value jump — at the abscissa of an up-step the function takes the *lower*
(pre-jump) value, i.e. ``f(x) = lim_{t->x^-} f(t)``. Every new primitive
(Inverse, Max/Min, domination) must share this convention.

Test surface: ``kayros._lera.nyr.NDCPWLF`` (bound in ``cpp/lera/bridge/lera_module.cpp``).
"""

from __future__ import annotations

import pytest

nyr = pytest.importorskip("kayros._lera").nyr
NDCPWLF = nyr.NDCPWLF


def test_construct_and_basic_eval():
    # Simple sloped function on [0, 20], slope 0.5.
    f = NDCPWLF([0.0, 20.0], [0.0, 10.0])
    assert f.empty() is False
    assert f.evaluate(0.0) == 0.0
    assert f.evaluate(10.0) == 5.0
    assert f.evaluate(20.0) == 10.0
    assert f.min_domain == 0.0 and f.max_domain == 20.0
    assert f.min_image == 0.0 and f.max_image == 10.0


def test_out_of_domain_raises():
    f = NDCPWLF([0.0, 20.0], [0.0, 10.0])
    with pytest.raises(Exception):
        f.evaluate(-1.0)
    with pytest.raises(Exception):
        f.evaluate(21.0)


# --- The step (value-jump) convention: left-continuous ----------------------

# f: slope 0.5 on [0,10], an up-step 5 -> 12 at x=10, plateau 12 on [10,20].
STEP_XS = [0.0, 10.0, 10.0, 20.0]
STEP_YS = [0.0, 5.0, 12.0, 12.0]


def test_step_is_left_continuous():
    """At the step abscissa, evaluate returns the LOWER (pre-jump) value.

    This is the checker's authoritative convention. The pre-fix impl uses
    ``upper_bound`` and returns the upper value (12.0) — the bug this gate
    locks out.
    """
    f = NDCPWLF(STEP_XS, STEP_YS)
    assert f.evaluate(10.0) == 5.0  # lower value, NOT 12.0


def test_step_neighbourhood_values():
    f = NDCPWLF(STEP_XS, STEP_YS)
    assert f.evaluate(0.0) == 0.0
    assert f.evaluate(5.0) == 2.5  # mid of the first slope
    assert f.evaluate(9.0) == 4.5  # just left of the step
    assert f.evaluate(15.0) == 12.0  # on the plateau
    assert f.evaluate(20.0) == 12.0


def test_plateau_is_constant():
    # Duplicate-y (plateau): slope 0 on [10,20].
    f = NDCPWLF(STEP_XS, STEP_YS)
    assert f.evaluate(12.0) == 12.0
    assert f.evaluate(18.0) == 12.0


def test_multi_step_left_continuous():
    # Two up-steps: at x=10 (2->6) and x=10 is single; add one at x=30 (8->15).
    xs = [0.0, 10.0, 10.0, 30.0, 30.0, 40.0]
    ys = [0.0, 2.0, 6.0, 8.0, 15.0, 15.0]
    f = NDCPWLF(xs, ys)
    assert f.evaluate(10.0) == 2.0  # lower at first step
    assert f.evaluate(30.0) == 8.0  # lower at second step
    assert f.evaluate(20.0) == 7.0  # slope 0.1 midway on [10,30]: 6 + 0.1*10
    assert f.evaluate(40.0) == 15.0


# --- Inverse: exact coordinate swap, step<->plateau duality -----------------


def test_inverse_sloped():
    f = NDCPWLF([0.0, 20.0], [0.0, 10.0])  # slope 0.5
    g = f.inverse()
    assert g.get_xs() == [0.0, 10.0]  # domain of g = image of f
    assert g.get_ys() == [0.0, 20.0]
    assert g.evaluate(5.0) == 10.0  # slope 2
    assert g.min_domain == f.min_image and g.max_domain == f.max_image


def test_inverse_is_involution_sloped():
    f = NDCPWLF([0.0, 10.0, 25.0], [0.0, 5.0, 30.0])
    assert f.inverse().inverse() == f


def test_inverse_step_becomes_plateau_and_back():
    # f has an up-step at x=10 (5->12) and a plateau at 12 on [10,20].
    f = NDCPWLF(STEP_XS, STEP_YS)
    g = f.inverse()
    assert g.check_invariant()
    # Step at x=10 of f -> plateau of g at height 10 over [5,12].
    # Plateau of f at 12 -> step of g at abscissa 12 (10->20).
    assert g.get_xs() == [0.0, 5.0, 12.0, 12.0]
    assert g.get_ys() == [0.0, 10.0, 10.0, 20.0]
    # f^{-1}(5): the step maps arrival 5 back to departure 10 (left value).
    assert g.evaluate(5.0) == 10.0
    # f^{-1}(8): interior of the plateau-turned-flat -> constant 10.
    assert g.evaluate(8.0) == 10.0
    # f^{-1}(12): step of g, left-continuous lower value -> 10 (earliest x).
    assert g.evaluate(12.0) == 10.0
    # Double inverse is the exact identity (swap twice).
    assert g.inverse() == f


def test_inverse_empty():
    empty = NDCPWLF([], [])
    assert empty.empty()
    assert empty.inverse().empty()
