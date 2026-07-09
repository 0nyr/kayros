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


@pytest.mark.xfail(
    reason="M5.9 goc retrofit not yet done: Value is right-continuous at a step "
    "(returns the upper 12.0) and slope-collapses verticals",
    strict=True,
)
def test_step_value_is_left_continuous():
    """Target: at the step abscissa, Value returns the LOWER (pre-jump) value."""
    f = PWL(STEP_XS, STEP_YS)
    assert f.value(10.0) == 5.0  # currently 12.0 (right-continuous / upper)
