#pragma once

#include "pwlf/pwlf.h"

namespace kayros {

// The ordinary window function ends at the deadline. To price lateness, the warp fold extends it with a constant ready time and records a second, nonnegative warp function.
// For arrival a, clamped ready time is max(e_j, min(a, l_j)) + s_j and lateness is max(0, a - l_j). Through the deadline the uncompressed augmented fold uses the same binary64 arithmetic as the sequential unwarped reference.

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

// Deduplicate interiors of exactly flat and vertical runs, preserving their endpoints. This preserves the binary64 minimum duration and the end of the zero-warp prefix. The full sequential reference retains every knot.
void dedup_safe_runs(Pwlf& f);

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

// Find the minimum duration among exactly zero-warp departures on the sequential binary64 curve. Return the earliest minimizing departure on that curve.
bool min_zero_warp_duration(PwlfView rho, PwlfView warp, MinShift* out);

// Penalised objective Phi(t) = (rho(t) - t) + penalty * warp(t), minimised
// over the union of both breakpoint grids. SEARCH-ONLY value (ranking):
// interpolation at foreign grid points carries ulp dust, so this must never
// be used for accounting — accepted moves are repriced by the fold and
// min_zero_warp_duration (repricing rule, extended to the warp channel).
// Precondition: rho and warp non-empty with identical domains.
MinShift min_penalised(PwlfView rho, PwlfView warp, double penalty);

}  // namespace kayros
