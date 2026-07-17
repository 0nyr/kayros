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


def test_inverse_step_choice_semantics_and_involutive():
    # arr-like: slope 0.5 on [0,10], up-step 5->12 at x=10, plateau 12 on [10,20].
    f = PWL(STEP_XS, STEP_YS)
    g = f.inverse()
    # Domain of the inverse is the image of f.
    assert g.min_domain == pytest.approx(0.0) and g.max_domain == pytest.approx(12.0)
    # f^{-1}(5) = 10 (step maps arrival 5 back to departure 10, lower/left value).
    assert g.value(5.0) == pytest.approx(10.0)
    # f^{-1}(8): interior of the plateau-turned-flat -> constant 10.
    assert g.value(8.0) == pytest.approx(10.0)
    # f^{-1}(12): the plateau inverts into a CHOICE vertical (memo 13.2) whose
    # representative is the LATEST x = 20 (dep semantics: latest departure
    # arriving by 12; goc's historical max{x : f(x) = y}). Before 13.2 this
    # returned the lower endpoint 10 under the left-continuity convention.
    assert g.value(12.0) == pytest.approx(20.0)
    verts = [p for p in g.pieces() if p[4]]
    assert len(verts) == 1 and verts[0][5] == "choice"
    # Double inverse recovers f pointwise AWAY from the jump abscissa. AT the
    # jump abscissa the attainment side is lost: the jump inverts into a
    # plateau, and re-inverting any plateau yields a CHOICE vertical with the
    # latest representative (13.2) because plateaus carry no provenance tag.
    # Solver-safe: every solver inversion produces a dep-role function (arr ->
    # dep, r.arr -> r.dep), where plateau-inverse = choice is always correct;
    # nothing ever re-inverts a dep back into an arr.
    ff = g.inverse()
    for x in (0.0, 5.0, 9.0, 15.0, 20.0):
        assert ff.value(x) == pytest.approx(f.value(x))
    assert ff.value(10.0) == pytest.approx(12.0)  # latest representative, not 5


# --- Compose must preserve value jumps (f o g)(x) = f(g(x)) ------------------


def test_compose_identity_preserves_f_step():
    """f o id == f, including f's value jump."""
    f = PWL(STEP_XS, STEP_YS)  # step 5->12 at x=10
    ident = PWL([0.0, 20.0], [0.0, 20.0])
    fog = f.compose(ident)
    assert fog.value(9.0) == pytest.approx(4.5)
    assert fog.value(10.0) == pytest.approx(5.0)  # left-continuous lower value
    assert fog.value(15.0) == pytest.approx(12.0)


def test_compose_f_step_with_increasing_g():
    """g increasing maps x->2x on [0,10]; f steps at y=10. f(g(x)) steps at x=5."""
    # f: slope 1 on [0,10], up-step 10->30 at x=10, slope 1 on [10,20] (30..40).
    f = PWL([0.0, 10.0, 10.0, 20.0], [0.0, 10.0, 30.0, 40.0])
    g = PWL([0.0, 10.0], [0.0, 20.0])  # slope 2, image [0,20]
    fog = f.compose(g)
    # g(5)=10 -> f steps there. Left-continuous: fog(5)=f(10^-)=10.
    assert fog.value(4.0) == pytest.approx(8.0)  # f(g(4))=f(8)=8
    assert fog.value(5.0) == pytest.approx(10.0)  # lower side of the step
    assert fog.value(6.0) == pytest.approx(32.0)  # f(g(6))=f(12)=32


def test_compose_g_step_makes_fog_jump():
    """g itself has a value jump; f linear. f(g(x)) inherits the jump."""
    f = PWL([0.0, 40.0], [0.0, 40.0])  # identity on [0,40]
    # g: slope 1 on [0,10], up-step 10->30 at x=10, slope 1 on [10,20].
    g = PWL([0.0, 10.0, 10.0, 20.0], [0.0, 10.0, 30.0, 40.0])
    fog = f.compose(g)  # == g since f is identity
    assert fog.value(9.0) == pytest.approx(9.0)
    assert fog.value(10.0) == pytest.approx(10.0)  # left-continuous lower value of g's jump
    assert fog.value(11.0) == pytest.approx(31.0)


# --- Exact graph reflections (M5.9 section 12-13): flip_time / flip_value ----
# These replace the labeling's only decreasing-inner compositions
# (reverse_instance and Merge use T - Id). Each breakpoint moves by ONE IEEE
# subtraction: platform-stable, no epsilon logic, and attainment (the pointwise
# value at a jump abscissa) is preserved by construction (section 13.1).


def test_flip_time_continuous():
    f = PWL([0.0, 10.0, 25.0], [0.0, 5.0, 30.0])
    g = f.flip_time(25.0)  # g(x) = f(25 - x)
    assert g.min_domain == pytest.approx(0.0) and g.max_domain == pytest.approx(25.0)
    assert g.value(0.0) == pytest.approx(30.0)
    assert g.value(15.0) == pytest.approx(5.0)  # f(10)
    assert g.value(25.0) == pytest.approx(0.0)


def test_flip_time_step_preserves_attained_value():
    """g = flip_time(STEP, 20): the jump moves to x=10 and stays ATTAINED at 5.

    STEP is left-continuous (STEP(10) = 5, the lower value). g(10) = STEP(10)
    = 5 must hold even though g's jump is now right-continuous (its left
    limit at 10 is the plateau 12).
    """
    f = PWL(STEP_XS, STEP_YS)
    g = f.flip_time(20.0)
    assert g.value(0.0) == pytest.approx(12.0)   # f(20), plateau side
    assert g.value(5.0) == pytest.approx(12.0)   # f(15)
    assert g.value(10.0) == pytest.approx(5.0)   # f(10): ATTAINED value kept
    assert g.value(15.0) == pytest.approx(2.5)   # f(5)
    assert g.value(20.0) == pytest.approx(0.0)   # f(0)


def test_flip_time_is_involutive_on_steps():
    f = PWL(STEP_XS, STEP_YS)
    ff = f.flip_time(20.0).flip_time(20.0)
    for x in (0.0, 5.0, 9.5, 10.0, 12.0, 20.0):
        assert ff.value(x) == pytest.approx(f.value(x))


def test_flip_value_step():
    """h = flip_value(STEP, 20): h(x) = 20 - STEP(x), jump span reflected."""
    f = PWL(STEP_XS, STEP_YS)
    h = f.flip_value(20.0)
    assert h.value(0.0) == pytest.approx(20.0)
    assert h.value(10.0) == pytest.approx(15.0)  # 20 - 5, attained side kept
    assert h.value(15.0) == pytest.approx(8.0)   # 20 - 12
    assert h.min_image == pytest.approx(8.0) and h.max_image == pytest.approx(20.0)


def test_double_flip_is_the_reverse_arrival_reflection():
    """r(x) = T - f(T - x): non-decreasing again, attained value = T - f(T - x0).

    This is the reverse_instance construction. For STEP with T = 20 the jump
    lands at x = 10 spanning 8..15 with r(10) = 20 - STEP(10) = 15: the
    attained value is now the UPPER endpoint (a right-continuous-style jump),
    which is exactly what left-continuity-assuming code used to get wrong.
    """
    f = PWL(STEP_XS, STEP_YS)
    r = f.flip_time(20.0).flip_value(20.0)
    assert r.value(0.0) == pytest.approx(8.0)    # 20 - f(20)
    assert r.value(9.0) == pytest.approx(8.0)    # plateau
    assert r.value(10.0) == pytest.approx(15.0)  # 20 - f(10): attained UPPER
    assert r.value(15.0) == pytest.approx(17.5)  # 20 - f(5)
    assert r.value(20.0) == pytest.approx(20.0)  # 20 - f(0)


def test_compose_g_jump_produces_composite_jump_pieces():
    """A g-jump maps through continuous f to a genuine composite vertical.

    f = 2x on [0, 60]; g jumps at x=10 from 10 to 30 (attained 10). fog must
    jump at 10 from f(10)=20 (attained) to f(30)=60, with f's interior over
    (10, 30) discarded. NOTE (13.2): this discard rule applies to JUMP
    verticals only; dep-style CHOICE verticals (inverse of a plateau, every
    interior value attained by a real departure choice) must keep the legacy
    sweep semantics. The tagged-vertical Compose will carry both.
    """
    f = PWL([0.0, 60.0], [0.0, 120.0])
    g = PWL([0.0, 10.0, 10.0, 20.0], [0.0, 10.0, 30.0, 40.0])
    fog = f.compose(g)
    assert fog.value(9.0) == pytest.approx(18.0)
    assert fog.value(10.0) == pytest.approx(20.0)   # attained side preserved
    assert fog.value(11.0) == pytest.approx(62.0)   # f(31)
    verts = [p for p in fog.pieces() if p[4]]
    assert len(verts) == 1
    dl, dr, il, ir, _, kind = verts[0]
    assert kind == "jump"
    assert dl == dr == pytest.approx(10.0)
    assert (il, ir) == (pytest.approx(20.0), pytest.approx(60.0))


def test_compose_f_jump_through_increasing_g_keeps_span():
    """f jumps at y=10; g = 2x reaches it at x=5. fog jumps at 5, span kept.

    Implemented by the tagged-vertical Compose (13.2): JUMP verticals of f
    emit their span. Since M13.0, CHOICE verticals are preserved too (span +
    kind + representative; see test_compose_increasing_inner_preserves_choice_vertical).
    """
    f = PWL([0.0, 10.0, 10.0, 20.0], [0.0, 10.0, 30.0, 40.0])
    g = PWL([0.0, 10.0], [0.0, 20.0])
    fog = f.compose(g)
    verts = [p for p in fog.pieces() if p[4]]
    assert len(verts) == 1
    dl, _, il, ir, _, kind = verts[0]
    assert kind == "jump"
    assert dl == pytest.approx(5.0)
    assert (min(il, ir), max(il, ir)) == (pytest.approx(10.0), pytest.approx(30.0))
    assert fog.value(5.0) == pytest.approx(10.0)  # attained (f left-continuous)


# --- Tagged verticals (memo 13.2): jump vs choice --------------------------


def test_inverse_plateau_yields_choice_vertical_latest_representative():
    """arr plateau [10,20] -> 15 inverts to a CHOICE vertical at 15, rep = 20."""
    arr = PWL([0.0, 10.0, 20.0], [5.0, 15.0, 15.0])
    dep = arr.inverse()
    assert dep.value(15.0) == pytest.approx(20.0)  # latest departure
    verts = [p for p in dep.pieces() if p[4]]
    assert len(verts) == 1
    dl, dr, il, ir, _, kind = verts[0]
    assert kind == "choice"
    assert dl == pytest.approx(15.0)
    assert (il, ir) == (pytest.approx(10.0), pytest.approx(20.0))


def test_compose_through_choice_vertical_sweeps_outer():
    """A CHOICE vertical in g sweeps f over the span (prices the choice).

    f is V-shaped (min interior); g = dep-like with a choice vertical at x=15
    spanning [10, 20]. fog at 15 must contain f's values over the WHOLE span
    (min included), unlike a jump, which would keep only the endpoints.
    """
    f = PWL([0.0, 15.0, 30.0], [30.0, 0.0, 30.0])  # V shape, min 0 at 15
    arr = PWL([0.0, 10.0, 20.0], [5.0, 15.0, 15.0])
    dep = arr.inverse()  # choice vertical at 15 spanning [10, 20]
    fog = f.compose(dep)
    # The sweep must expose f's interior minimum f(15) = 0 at x0 = 15.
    assert fog.min_image == pytest.approx(0.0)
    swept = [p for p in fog.pieces() if p[0] == pytest.approx(15.0) and p[0] == p[1]]
    assert len(swept) >= 2  # f's two slopes over the span, stacked at x0
    assert all(p[5] == "choice" for p in swept if p[4])


def test_flip_time_preserves_choice_tag():
    arr = PWL([0.0, 10.0, 20.0], [5.0, 15.0, 15.0])
    dep = arr.inverse()
    flipped = dep.flip_time(20.0)
    verts = [p for p in flipped.pieces() if p[4]]
    assert len(verts) == 1 and verts[0][5] == "choice"
    # Representative preserved through the reflection: value at the reflected
    # abscissa (20 - 15 = 5) is still the latest departure 20.
    assert flipped.value(5.0) == pytest.approx(20.0)


# --- Max/Min and Restrict ops with verticals (memo 12.4) ---------------------


def test_max_with_constant_preserves_jump():
    """Max(STEP, const) must keep the jump structure (it used to drop it)."""
    f = PWL(STEP_XS, STEP_YS)  # slope to 5, jump 5->12 at 10, plateau 12
    c = PWL([0.0, 20.0], [8.0, 8.0])
    m = f.max(c)
    assert m.value(0.0) == pytest.approx(8.0)
    assert m.value(10.0) == pytest.approx(8.0)   # max(attained 5, 8)
    assert m.value(15.0) == pytest.approx(12.0)
    verts = [p for p in m.pieces() if p[4]]
    assert len(verts) == 1
    dl, _, il, ir, _, kind = verts[0]
    assert kind == "jump" and dl == pytest.approx(10.0)
    assert (min(il, ir), max(il, ir)) == (pytest.approx(8.0), pytest.approx(12.0))


def test_min_with_constant_preserves_jump():
    """Min(STEP, const 8): jump span clips to [5, 8]; plateau clamps to 8."""
    f = PWL(STEP_XS, STEP_YS)
    c = PWL([0.0, 20.0], [8.0, 8.0])
    m = f.min(c)
    assert m.value(10.0) == pytest.approx(5.0)   # min(attained 5, 8)
    assert m.value(15.0) == pytest.approx(8.0)
    verts = [p for p in m.pieces() if p[4]]
    assert len(verts) == 1
    _, _, il, ir, _, kind = verts[0]
    assert kind == "jump"
    assert (min(il, ir), max(il, ir)) == (pytest.approx(5.0), pytest.approx(8.0))


def test_max_passes_choice_vertical_through():
    """A choice vertical outside the other function's reach survives with tag."""
    arr = PWL([0.0, 10.0, 20.0], [5.0, 15.0, 15.0])
    dep = arr.inverse()  # choice vertical at 15 spanning [10, 20]
    low = PWL([0.0, 15.0], [0.0, 0.0])  # constant 0 over [0, 15]
    m = dep.max(low)
    verts = [p for p in m.pieces() if p[4]]
    assert len(verts) == 1 and verts[0][5] == "choice"
    assert m.value(15.0) == pytest.approx(20.0)  # representative kept


def test_scalar_ops_preserve_tags():
    arr = PWL([0.0, 10.0, 20.0], [5.0, 15.0, 15.0])
    dep = arr.inverse()
    neg = dep.__mul__(-1.0)
    verts = [p for p in neg.pieces() if p[4]]
    assert len(verts) == 1 and verts[0][5] == "choice"
    f = PWL(STEP_XS, STEP_YS)
    neg2 = f.__mul__(-1.0)
    verts2 = [p for p in neg2.pieces() if p[4]]
    assert len(verts2) == 1 and verts2[0][5] == "jump"


# --- M13.0 exact value-jump completeness fixes (session 24) -----------------
# Root cause of the 3 exact-arm incompleteness reproducers: the labeling's
# composed/added functions lost attained values and choice spans in three
# spots, letting backward-label durations under-estimate mandatory waiting
# (Rifki-2 dominator 21,4,2,10,3 claimed ~1959 where the checker-exact suffix
# duration is ~12703; the witness column's label was then erased at the EXACT
# level and the certificate closed 73 units above the optimum).


def test_compose_plateau_inner_uses_attained_value_at_f_vertical():
    """Constant-inner Compose must evaluate f at the ATTAINED side of a jump.

    fr = FlipTime of an up-step: its vertical at x=3 attains 3 (13.1), while
    the piece scanned first (ending at x=3) carries the unattained limit 9.
    Composing with a plateau at value 3 must produce the constant 3, not 9.
    """
    f0 = PWL([0.0, 3.0, 3.0, 6.0], [0.0, 3.0, 9.0, 12.0])
    fr = f0.flip_time(6.0)
    assert fr.value(3.0) == pytest.approx(3.0)  # 13.1: attained preserved
    g = PWL([0.0, 4.0], [3.0, 3.0])  # plateau at the vertical's abscissa
    fog = fr.compose(g)
    assert fog.value(0.0) == pytest.approx(3.0)
    assert fog.value(2.0) == pytest.approx(3.0)
    assert fog.value(4.0) == pytest.approx(3.0)


def test_compose_plateau_inner_at_choice_vertical_takes_span_min():
    """Constant-inner Compose at a CHOICE vertical of f: pointwise minimum.

    Durations are set-valued at choice abscissae with min semantics
    (MinValueAt, 21/n); a plateau inner pinning that abscissa must expose the
    attainable minimum, not an arbitrary side.
    """
    arr = PWL([0.0, 10.0, 20.0], [5.0, 15.0, 15.0])
    dep = arr.inverse()  # choice vertical at 15 spanning [10, 20], rep 20
    g = PWL([0.0, 4.0], [15.0, 15.0])  # plateau at the choice abscissa
    fog = dep.compose(g)
    assert fog.value(2.0) == pytest.approx(10.0)  # span minimum, attainable


def test_compose_increasing_inner_preserves_choice_vertical():
    """A CHOICE vertical of f survives composition with an increasing g.

    Collapsing it to the representative (the pre-M13.0 behavior) loses the
    span minimum: downstream MinValueAt/domination then over-estimate the
    label's duration and price columns out — the incompleteness class.
    """
    arr = PWL([0.0, 10.0, 20.0], [5.0, 15.0, 15.0])
    dep = arr.inverse()  # choice vertical at 15 spanning [10, 20]
    g = PWL([0.0, 60.0], [0.0, 30.0])  # slope 0.5: reaches y=15 at x=30
    fog = dep.compose(g)
    verts = [p for p in fog.pieces() if p[4]]
    assert len(verts) == 1
    dl, dr, il, ir, _, kind = verts[0]
    assert kind == "choice"
    assert dl == dr == pytest.approx(30.0)
    assert (min(il, ir), max(il, ir)) == (pytest.approx(10.0), pytest.approx(20.0))
    assert fog.value(30.0) == pytest.approx(20.0)  # representative kept


def test_add_pairs_attained_endpoints_on_down_jump():
    """operator+ must pair attained-with-attained (13.1), like operator*.

    A down-jump attains its HIGH endpoint (9); adding a constant 1 must keep
    the attained value 9+1=10. The lo/hi image pairing (pre-M13.0) mistagged
    the sum's attained value as 3+1=4.
    """
    f = PWL([0.0, 3.0, 3.0, 6.0], [12.0, 9.0, 3.0, 0.0])
    assert f.value(3.0) == pytest.approx(9.0)  # attained = incoming
    one = PWL([0.0, 6.0], [1.0, 1.0])
    s = f.__add__(one)
    assert s.value(3.0) == pytest.approx(10.0)
    verts = [p for p in s.pieces() if p[4]]
    assert len(verts) == 1 and verts[0][5] == "jump"


def test_add_keeps_stacked_vertical_tail_at_operand_exhaustion():
    """operator+ must not drop stacked verticals when the other operand ends.

    The r6 (Rifki-17) mechanism: a swept composite carries STACKED choice
    verticals at one abscissa; adding a function that ends exactly there
    advanced both operands after the first vertical and silently dropped the
    rest of the stack (the label kept one of eight departure choices and
    over-estimated its duration by ~1100 units).
    """
    arr = PWL([9335.0, 11495.0], [11843.0, 11843.0])
    dep = arr.inverse()  # choice vertical at 11843 spanning [9335, 11495]
    f = PWL([9335.0, 10365.0, 11000.0, 11495.0], [732.0, 754.0, 500.0, 764.0])
    comp = f.compose(dep)  # >= 3 stacked choice verticals at 11843
    stacked = [p for p in comp.pieces() if p[4]]
    assert len(stacked) >= 3
    assert comp.min_image == pytest.approx(500.0)
    g = PWL([11000.0, 11843.0], [0.0, 0.0])  # ends exactly at the stack
    s = comp.__add__(g)
    kept = [p for p in s.pieces() if p[4]]
    assert len(kept) == len(stacked)  # the whole stack survives
    assert s.min_image == pytest.approx(500.0)
    assert s.max_image == pytest.approx(comp.max_image)
