#pragma once

#include <cstdint>
#include <vector>

namespace kayros {

// Non-decreasing continuous piecewise-linear function (NDCPWLF).
// Parallel breakpoint arrays: duplicate xs encode a vertical step (evaluation
// returns the smallest value), duplicate ys encode a plateau. Empty arrays
// mean the empty function.
//
// Exact IEEE-754 double arithmetic, no epsilons: this is a bit-identical port
// of the reference implementation in mamut_routing_lib.td.pwlf (the canonical
// Duration checker). Any change to the arithmetic here must keep the
// checker-equivalence test suite exactly green.
struct Pwlf {
    std::vector<double> xs;
    std::vector<double> ys;
};

// Non-owning view over breakpoint arrays (e.g. a slice of an instance's ATF pool).
struct PwlfView {
    const double* xs = nullptr;
    const double* ys = nullptr;
    std::int64_t n = 0;
};

inline PwlfView view(const Pwlf& f) {
    return {f.xs.data(), f.ys.data(), static_cast<std::int64_t>(f.xs.size())};
}

inline bool is_empty(PwlfView f) { return f.n == 0; }

// f(x) = x on [low, high]; single breakpoint when low == high.
// Precondition: low <= high.
Pwlf identity(double low, double high);

// f(x). Precondition: f non-empty and xs[0] <= x <= xs[n-1] (throws
// std::out_of_range otherwise). At a vertical step returns the smallest value.
double evaluate(PwlfView f, double x);

// h = f ∘ g restricted to {x in dom(g) : g(x) in dom(f)}.
// Two-pointer event merge over dom(f) ∩ img(g), O(f.n + g.n).
Pwlf compose(PwlfView f, PwlfView g);

// Fold study (session 136), arm F3: h = f ∘ g written into h (its capacity is reused), with the same events, arithmetic and output as compose, and no temporary vectors. h must not alias f or g.
void compose_into(PwlfView f, PwlfView g, Pwlf& h);

struct MinShift {
    double value;     // min_k (ys[k] - xs[k])
    double argmin_x;  // xs[k*] with k* the earliest argmin
};

// Precondition: f non-empty. For a route ready-time function this is the
// optimal duration and the associated optimal depot departure time.
MinShift min_shifted_image(PwlfView f);

// Vertex time-window ready-time function over arrival times in [0, latest]:
// theta(t) = max(t, earliest) + service_time. Precondition: earliest <= latest.
Pwlf make_theta(double earliest, double latest, double service_time);

// Fold study (session 136), arm F1: the td-fold/2 vertex transforms of mamut_routing_lib.td.pwlf, with the generic interpolation (no slope-one rule). Neither composes a ready-time function theta.

// f restricted to [low, high] ∩ dom(f): breakpoints inside the window are kept verbatim, a window end strictly inside a piece is interpolated. Replaces compose(f, identity(low, high)). Empty when the window misses.
Pwlf restrict_domain(PwlfView f, double low, double high);

// The vertex map t -> max(t, earliest) + service_time applied to the image of acc, with images above `latest` cut (crossings inserted by inverse interpolation, the fold stops at the first image above latest). Replaces compose(theta, acc). has_earliest = false means no waiting, has_latest = false means no cut. Empty when acc.ys[0] > latest.
Pwlf apply_ready_time(PwlfView acc, bool has_earliest, double earliest,
                      bool has_latest, double latest, double service_time);

}  // namespace kayros
