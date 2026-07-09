"""goc::PWLFunction step-capable retrofit gates (M5.9).

The labeling's general, non-monotone duration/cost functions live on
goc::PWLFunction. Today it cannot represent a genuine value jump: LinearFunction
collapses a zero-width piece to slope 0 with the UPPER value, and Value iterates
pieces back-to-front so it is right-continuous at shared breakpoints. Both are
wrong for the checker's left-continuous step semantics.

This file has two roles:
  * ``test_characterize_*`` — pin the CURRENT (buggy) behavior so the retrofit's
    effect is visible in the diff. These are xfail(strict) on the target
    behavior; they will flip when the fix lands.
  * ``test_jumpfree_*`` — continuous-function behavior that must stay identical
    through the retrofit (regression guard: no vertical => no new branch taken).

Test surface: ``kayros._lera.goc.PWLFunction``.
"""

from __future__ import annotations

import pytest

_lera = pytest.importorskip("kayros._lera")
PWL = _lera.goc.PWLFunction


# f: slope 0.5 on [0,10], up-step 5 -> 12 at x=10, plateau 12 on [10,20].
STEP_XS = [0.0, 10.0, 10.0, 20.0]
STEP_YS = [0.0, 5.0, 12.0, 12.0]


def test_jumpfree_value_is_stable():
    """Continuous function: Value stays as-is through the retrofit.

    Note: goc evaluates ``slope*x + intercept``, so it carries ulp dust at
    breakpoints (Value(10.0) == 4.9999999999999964, not exactly 5.0) even on a
    continuous function — unlike NDCPWLF, which interpolates exactly from stored
    breakpoints. The step-capable retrofit's left-continuous Value can return the
    exact stored value; this guard only pins that continuous behavior does not
    regress, so it compares with a tolerance.
    """
    f = PWL([0.0, 10.0, 25.0], [0.0, 5.0, 30.0])
    assert f.value(0.0) == pytest.approx(0.0)
    assert f.value(10.0) == pytest.approx(5.0)
    assert f.value(5.0) == pytest.approx(2.5)
    assert f.value(25.0) == pytest.approx(30.0)
    assert f.min_domain == 0.0 and f.max_domain == 25.0


def test_step_representable_at_all():
    """A goc PWL built from a step still spans the right domain/image."""
    f = PWL(STEP_XS, STEP_YS)
    assert f.min_domain == 0.0 and f.max_domain == 20.0
    assert f.min_image == 0.0 and f.max_image == 12.0
    assert f.value(5.0) == 2.5  # on the first slope, unambiguous
    assert f.value(15.0) == 12.0  # on the plateau, unambiguous


def test_step_value_is_left_continuous():
    """At the step abscissa, Value returns the LOWER (pre-jump) value.

    Left-continuous: the piece ending at x from the left is selected. (goc's
    slope*x+intercept eval carries ulp dust, so compare with tolerance.)
    """
    f = PWL(STEP_XS, STEP_YS)
    assert f.value(10.0) == pytest.approx(5.0)  # was 12.0 before the retrofit
    # And just off the abscissa the values are unambiguous.
    assert f.value(9.5) == pytest.approx(4.75)
    assert f.value(15.0) == pytest.approx(12.0)


# --- Inverse: exact coordinate swap for non-decreasing functions -------------


def test_inverse_jumpfree():
    f = PWL([0.0, 20.0], [0.0, 10.0])  # slope 0.5
    g = f.inverse()
    assert g.min_domain == pytest.approx(0.0) and g.max_domain == pytest.approx(10.0)
    assert g.value(5.0) == pytest.approx(10.0)  # slope 2


@pytest.mark.xfail(
    reason="M5.9: goc AddPiece merges a vertical into its neighbour at "
    "construction (both encode as slope 0), so the step degrades to image.left "
    "and Inverse cannot recover it. Needs a genuine vertical marker in "
    "LinearFunction (slope=INFTY) + non-merging AddPiece; also the plateau->step "
    "inverse convention (min vs max pre-image) must be pinned to the labeling.",
    strict=True,
)
def test_inverse_step_is_left_continuous_and_involutive():
    # arr-like: slope 0.5 on [0,10], up-step 5->12 at x=10, plateau 12 on [10,20].
    f = PWL(STEP_XS, STEP_YS)
    g = f.inverse()
    # Domain of the inverse is the image of f.
    assert g.min_domain == pytest.approx(0.0) and g.max_domain == pytest.approx(12.0)
    # f^{-1}(5) = 10 (step maps arrival 5 back to departure 10, lower/left value).
    assert g.value(5.0) == pytest.approx(10.0)
    # f^{-1}(8): interior of the plateau-turned-flat -> constant 10.
    assert g.value(8.0) == pytest.approx(10.0)
    # f^{-1}(12): step of g, left-continuous lower value -> 10.
    assert g.value(12.0) == pytest.approx(10.0)
    # Double inverse recovers f pointwise (exact swap twice).
    ff = g.inverse()
    for x in (0.0, 5.0, 9.0, 10.0, 15.0, 20.0):
        assert ff.value(x) == pytest.approx(f.value(x))
