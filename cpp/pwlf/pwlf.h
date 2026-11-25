#pragma once

#include <cstdint>
#include <vector>

namespace kayros {

// Non-decreasing continuous piecewise-linear function (NDCPWLF).
// Parallel breakpoint arrays: duplicate xs encode a vertical step (evaluation
// returns the smallest value), duplicate ys encode a plateau. Empty arrays
// mean the empty function.
//
// The search evaluates these functions in binary64. The MAMUT library computes the exact objective.
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

inline Pwlf copy_pwlf(PwlfView f) {
    if (f.n == 0) return {};
    return {{f.xs, f.xs + f.n}, {f.ys, f.ys + f.n}};
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
// Reuse the capacity of out. Neither input may refer to its storage.
void compose_into(PwlfView f, PwlfView g, Pwlf& out);

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

}  // namespace kayros
