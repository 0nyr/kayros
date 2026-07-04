#include "pwlf/pwlf.h"

#include <algorithm>
#include <stdexcept>

namespace kayros {

Pwlf identity(double low, double high) {
    if (low == high) return {{low}, {low}};
    return {{low, high}, {low, high}};
}

double evaluate(PwlfView f, double x) {
    if (f.n == 0 || x < f.xs[0] || x > f.xs[f.n - 1]) {
        throw std::out_of_range("pwlf evaluate: x outside the function domain");
    }
    const double* it = std::lower_bound(f.xs, f.xs + f.n, x);
    std::int64_t i = it - f.xs;
    if (i < f.n && f.xs[i] == x) return f.ys[i];
    const double x_lo = f.xs[i - 1], x_hi = f.xs[i];
    const double y_lo = f.ys[i - 1], y_hi = f.ys[i];
    const double t = (x - x_lo) / (x_hi - x_lo);
    return y_lo + t * (y_hi - y_lo);
}

Pwlf compose(PwlfView f, PwlfView g) {
    if (f.n == 0 || g.n == 0) return {};
    const double lo = std::max(f.xs[0], g.ys[0]);
    const double hi = std::min(f.xs[f.n - 1], g.ys[g.n - 1]);
    if (lo > hi) return {};

    const double* fx = f.xs;
    const double* fy = f.ys;
    const double* gx = g.xs;
    const double* gy = g.ys;
    const std::int64_t nf = f.n, ng = g.n;

    std::int64_t i = 0;
    while (fx[i] < lo) ++i;
    std::int64_t j = 0;
    while (gy[j] < lo) ++j;

    Pwlf h;
    h.xs.reserve(static_cast<std::size_t>(nf + ng));
    h.ys.reserve(static_cast<std::size_t>(nf + ng));
    // Clamp monotone so rounding can never break the non-decreasing invariant,
    // then drop exact duplicates (identical to the reference emit()).
    auto emit = [&h](double x, double y) {
        if (!h.xs.empty()) {
            if (x < h.xs.back()) x = h.xs.back();
            if (y < h.ys.back()) y = h.ys.back();
            if (x == h.xs.back() && y == h.ys.back()) return;
        }
        h.xs.push_back(x);
        h.ys.push_back(y);
    };

    std::vector<double> f_ys;  // f-breakpoint values at event u (steps yield several)
    std::vector<double> g_xs;  // g-breakpoint positions with image u (plateaus yield several)
    while (true) {
        const bool has_f = i < nf && fx[i] <= hi;
        const bool has_g = j < ng && gy[j] <= hi;
        if (!has_f && !has_g) break;
        double u;
        if (!has_g) {
            u = fx[i];
        } else if (!has_f) {
            u = gy[j];
        } else {
            u = fx[i] <= gy[j] ? fx[i] : gy[j];
        }

        f_ys.clear();
        g_xs.clear();
        while (i < nf && fx[i] == u) {
            f_ys.push_back(fy[i]);
            ++i;
        }
        while (j < ng && gy[j] == u) {
            g_xs.push_back(gx[j]);
            ++j;
        }

        if (f_ys.empty()) {
            // u lies strictly inside an f piece: fx[i-1] < u < fx[i].
            const double x_lo = fx[i - 1], x_hi = fx[i];
            const double t = (u - x_lo) / (x_hi - x_lo);
            f_ys.push_back(fy[i - 1] + t * (fy[i] - fy[i - 1]));
        }
        if (g_xs.empty()) {
            // u lies strictly inside a g piece image: gy[j-1] < u < gy[j].
            const double y_lo = gy[j - 1], y_hi = gy[j];
            const double t = (u - y_lo) / (y_hi - y_lo);
            g_xs.push_back(gx[j - 1] + t * (gx[j] - gx[j - 1]));
        }

        for (double x_val : g_xs) emit(x_val, f_ys[0]);
        for (std::size_t k = 1; k < f_ys.size(); ++k) emit(g_xs.back(), f_ys[k]);
    }

    return h;
}

MinShift min_shifted_image(PwlfView f) {
    if (f.n == 0) {
        throw std::out_of_range("pwlf min_shifted_image: empty function");
    }
    double best = f.ys[0] - f.xs[0];
    double best_x = f.xs[0];
    for (std::int64_t k = 1; k < f.n; ++k) {
        const double value = f.ys[k] - f.xs[k];
        if (value < best) {
            best = value;
            best_x = f.xs[k];
        }
    }
    return {best, best_x};
}

Pwlf make_theta(double earliest, double latest, double service_time) {
    const double xs[3] = {0.0, earliest, latest};
    const double ys[3] = {earliest + service_time, earliest + service_time,
                          latest + service_time};
    Pwlf out;
    for (int k = 0; k < 3; ++k) {
        if (!out.xs.empty() && xs[k] == out.xs.back() && ys[k] == out.ys.back()) {
            continue;
        }
        out.xs.push_back(xs[k]);
        out.ys.push_back(ys[k]);
    }
    return out;
}

}  // namespace kayros
