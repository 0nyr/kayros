#pragma once

#include "pwlf/pwlf.h"

namespace kayros {

// TD time-warp primitives (Stream 8, P8.0 design memo).
//
// The exact core encodes a time window as a DOMAIN wall: make_theta caps the
// vertex function's domain at the deadline, and compose's img∩dom truncation
// erases late arrivals. The warp-augmented builders below replace that wall by
// a clamp-at-deadline plus a separate non-negative warp channel:
//
//   theta~_j(a) = max(e_j, min(a, l_j)) + s_j   (non-decreasing, then FLAT)
//   w_j(a)     = max(0, a - l_j)                (zero, then a +1 ramp)
//
// Everything stays inside the NDCPWLF algebra (no decreasing "back-in-time"
// functions). On arrivals a <= l_j the clamped builder executes the exact same
// arithmetic as make_theta, and compose processes its interior breakpoint at
// l_j with the same event/inversion arithmetic that today processes theta's
// domain end — this is what makes the augmented fold reduce BITWISE to the
// checker fold on the feasible region (gate G1/G2 in test_warp_equivalence).

// Warp-augmented vertex pair (theta~_j, w_j), both total on [0, t_end].
// Preconditions: earliest <= latest <= t_end.
struct ThetaWarp {
    Pwlf theta;  // clamped ready-time function
    Pwlf warp;   // lateness at this vertex as a function of the arrival time
};
ThetaWarp make_theta_warp(double earliest, double latest, double service_time,
                          double t_end);

// Depot-return pair: the route ends upon arrival (no waiting, no service),
// clamped at the due date instead of domain-restricted to [0, due]:
//   rho_0(a) = min(a, due),  w_0(a) = max(0, a - due),  both on [0, t_end].
// Precondition: due <= t_end.
Pwlf make_return_clamp(double due, double t_end);
Pwlf make_return_warp(double due, double t_end);

// h = f + g on dom(f) ∩ dom(g); empty when the intersection is empty.
// Two-pointer event merge over the union of breakpoints; foreign grid points
// are filled by the same interpolation arithmetic as evaluate(). Vertical
// steps sum their lower and upper values at the shared x. Exactness contract:
// where both operands are exactly 0.0 the sum is exactly 0.0 (warp zero
// region); elsewhere the sum is a value channel (repricing-rule territory).
Pwlf add(PwlfView f, PwlfView g);

// End of the zero prefix of a non-decreasing, non-negative PWLF: the largest
// breakpoint x with value exactly 0.0 (the function is zero on [xs[0], x] and
// strictly positive beyond). Returns false when w(xs[0]) > 0 (no zero set).
bool zero_prefix_end(PwlfView w, double* end);

// Duration minimisation restricted to the exactly-zero-warp set:
// min (rho.ys[k] - rho.xs[k]) over breakpoints with rho.xs[k] <= zero-warp end.
// This is the accounting evaluator: on checker-feasible routes it must equal
// the checker's Delta* bitwise (gate G1). Returns false when the zero-warp set
// is empty (checker-infeasible route).
bool min_zero_warp_duration(PwlfView rho, PwlfView warp, MinShift* out);

// Penalised objective Phi(t) = (rho(t) - t) + penalty * warp(t), minimised
// over the union of both breakpoint grids. SEARCH-ONLY value (ranking):
// interpolation at foreign grid points carries ulp dust, so this must never
// be used for accounting — accepted moves are repriced by the fold and
// min_zero_warp_duration (repricing rule, extended to the warp channel).
// Precondition: rho and warp non-empty with identical domains.
MinShift min_penalised(PwlfView rho, PwlfView warp, double penalty);

}  // namespace kayros
