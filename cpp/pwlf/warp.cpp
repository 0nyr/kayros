#include "pwlf/warp.h"

#include <algorithm>
#include <stdexcept>

namespace kayros {

namespace {

// Append (x, y) dropping exact duplicates of the previous point (the
// make_theta dedup, shared by all builders here).
inline void push_dedup(Pwlf& f, double x, double y) {
    if (!f.xs.empty() && x == f.xs.back() && y == f.ys.back()) return;
    f.xs.push_back(x);
    f.ys.push_back(y);
}

}  // namespace

ThetaWarp make_theta_warp(double earliest, double latest, double service_time,
                          double t_end) {
    if (!(earliest <= latest) || !(latest <= t_end)) {
        throw std::invalid_argument("make_theta_warp: need earliest <= latest <= t_end");
    }
    ThetaWarp out;
    // theta~: identical points to make_theta below the deadline (same
    // arithmetic, bit-for-bit), then flat at latest + service_time.
    push_dedup(out.theta, 0.0, earliest + service_time);
    push_dedup(out.theta, earliest, earliest + service_time);
    push_dedup(out.theta, latest, latest + service_time);
    push_dedup(out.theta, t_end, latest + service_time);
    // w: exactly 0.0 up to the deadline (no arithmetic on the zero branch),
    // then a slope-1 ramp.
    push_dedup(out.warp, 0.0, 0.0);
    push_dedup(out.warp, latest, 0.0);
    push_dedup(out.warp, t_end, t_end - latest);
    return out;
}

Pwlf make_return_clamp(double due, double t_end) {
    if (!(due <= t_end)) {
        throw std::invalid_argument("make_return_clamp: need due <= t_end");
    }
    Pwlf f;
    push_dedup(f, 0.0, 0.0);
    push_dedup(f, due, due);
    push_dedup(f, t_end, due);
    return f;
}

Pwlf make_return_warp(double due, double t_end) {
    if (!(due <= t_end)) {
        throw std::invalid_argument("make_return_warp: need due <= t_end");
    }
    Pwlf f;
    push_dedup(f, 0.0, 0.0);
    push_dedup(f, due, 0.0);
    push_dedup(f, t_end, t_end - due);
    return f;
}

void dedup_safe_runs(Pwlf& f) {
    const std::size_t n = f.xs.size();
    if (n < 3) return;
    std::size_t out = 1;
    for (std::size_t i = 1; i + 1 < n; ++i) {
        const bool flat_interior =
            f.ys[out - 1] == f.ys[i] && f.ys[i] == f.ys[i + 1];
        const bool vert_interior =
            f.xs[out - 1] == f.xs[i] && f.xs[i] == f.xs[i + 1];
        if (flat_interior || vert_interior) continue;
        f.xs[out] = f.xs[i];
        f.ys[out] = f.ys[i];
        ++out;
    }
    f.xs[out] = f.xs[n - 1];
    f.ys[out] = f.ys[n - 1];
    ++out;
    f.xs.resize(out);
    f.ys.resize(out);
}

Pwlf add(PwlfView f, PwlfView g) {
    if (f.n == 0 || g.n == 0) return {};
    const double lo = std::max(f.xs[0], g.xs[0]);
    const double hi = std::min(f.xs[f.n - 1], g.xs[g.n - 1]);
    if (lo > hi) return {};

    // Interpolated value of h strictly inside a piece, evaluate()'s formula.
    auto interp = [](PwlfView h, std::int64_t i, double x) {
        const double x_lo = h.xs[i - 1], x_hi = h.xs[i];
        const double y_lo = h.ys[i - 1], y_hi = h.ys[i];
        const double t = (x - x_lo) / (x_hi - x_lo);
        return y_lo + t * (y_hi - y_lo);
    };

    std::int64_t i = 0;
    while (f.xs[i] < lo) ++i;
    std::int64_t j = 0;
    while (g.xs[j] < lo) ++j;

    Pwlf h;
    h.xs.reserve(static_cast<std::size_t>(f.n + g.n));
    h.ys.reserve(static_cast<std::size_t>(f.n + g.n));
    // Monotone clamp + exact-duplicate drop, identical to compose's emit.
    auto emit = [&h](double x, double y) {
        if (!h.xs.empty()) {
            if (x < h.xs.back()) x = h.xs.back();
            if (y < h.ys.back()) y = h.ys.back();
            if (x == h.xs.back() && y == h.ys.back()) return;
        }
        h.xs.push_back(x);
        h.ys.push_back(y);
    };

    bool emitted_lo = false;
    while (true) {
        const bool has_f = i < f.n && f.xs[i] <= hi;
        const bool has_g = j < g.n && g.xs[j] <= hi;
        if (!has_f && !has_g) break;
        double u;
        if (!has_g) {
            u = f.xs[i];
        } else if (!has_f) {
            u = g.xs[j];
        } else {
            u = f.xs[i] <= g.xs[j] ? f.xs[i] : g.xs[j];
        }

        // Lower/upper values at u (vertical steps carry several breakpoints).
        double f_lo_val, f_hi_val;
        if (i < f.n && f.xs[i] == u) {
            f_lo_val = f.ys[i];
            std::int64_t k = i;
            while (k + 1 < f.n && f.xs[k + 1] == u) ++k;
            f_hi_val = f.ys[k];
            i = k + 1;
        } else {
            f_lo_val = f_hi_val = interp(f, i, u);
        }
        double g_lo_val, g_hi_val;
        if (j < g.n && g.xs[j] == u) {
            g_lo_val = g.ys[j];
            std::int64_t k = j;
            while (k + 1 < g.n && g.xs[k + 1] == u) ++k;
            g_hi_val = g.ys[k];
            j = k + 1;
        } else {
            g_lo_val = g_hi_val = interp(g, j, u);
        }

        emit(u, f_lo_val + g_lo_val);
        emit(u, f_hi_val + g_hi_val);
        emitted_lo = true;
    }
    if (!emitted_lo) {
        // Degenerate: no breakpoint of either operand inside [lo, hi]
        // (single-point intersection strictly inside both pieces).
        double fv, gv;
        {
            std::int64_t k = 0;
            while (f.xs[k] < lo) ++k;
            fv = (f.xs[k] == lo) ? f.ys[k] : interp(f, k, lo);
        }
        {
            std::int64_t k = 0;
            while (g.xs[k] < lo) ++k;
            gv = (g.xs[k] == lo) ? g.ys[k] : interp(g, k, lo);
        }
        emit(lo, fv + gv);
    }
    return h;
}

bool zero_prefix_end(PwlfView w, double* end) {
    if (w.n == 0 || w.ys[0] != 0.0) return false;
    std::int64_t k = 0;
    while (k + 1 < w.n && w.ys[k + 1] == 0.0) ++k;
    *end = w.xs[k];
    return true;
}

bool min_zero_warp_duration(PwlfView rho, PwlfView warp, MinShift* out) {
    double tau;
    if (rho.n == 0 || !zero_prefix_end(warp, &tau)) return false;
    bool found = false;
    bool tau_is_breakpoint = false;
    double best = 0.0, best_x = 0.0;
    for (std::int64_t k = 0; k < rho.n && rho.xs[k] <= tau; ++k) {
        const double value = rho.ys[k] - rho.xs[k];
        if (rho.xs[k] == tau) tau_is_breakpoint = true;
        if (!found || value < best) {
            best = value;
            best_x = rho.xs[k];
            found = true;
        }
    }
    // The boundary tau itself is always a candidate: under the safe dedup a
    // flat run of rho can straddle tau (its interior breakpoint at tau was
    // removed), and the restricted minimum lives exactly there. Evaluation is
    // exact in that case (flat interpolation adds t * 0.0).
    if (!tau_is_breakpoint && tau >= rho.xs[0] && tau <= rho.xs[rho.n - 1]) {
        const double value = evaluate(rho, tau) - tau;
        if (!found || value < best) {
            best = value;
            best_x = tau;
            found = true;
        }
    }
    if (!found) return false;  // rho starts strictly after the zero-warp end
    out->value = best;
    out->argmin_x = best_x;
    return true;
}

MinShift min_penalised(PwlfView rho, PwlfView warp, double penalty) {
    if (rho.n == 0 || warp.n == 0) {
        throw std::out_of_range("min_penalised: empty function");
    }
    bool found = false;
    double best = 0.0, best_x = 0.0;
    auto consider = [&](double x, double value) {
        if (!found || value < best) {
            best = value;
            best_x = x;
            found = true;
        }
    };
    for (std::int64_t k = 0; k < rho.n; ++k) {
        const double x = rho.xs[k];
        if (x < warp.xs[0] || x > warp.xs[warp.n - 1]) continue;
        consider(x, rho.ys[k] - x + penalty * evaluate(warp, x));
    }
    for (std::int64_t k = 0; k < warp.n; ++k) {
        const double x = warp.xs[k];
        if (x < rho.xs[0] || x > rho.xs[rho.n - 1]) continue;
        consider(x, evaluate(rho, x) - x + penalty * warp.ys[k]);
    }
    if (!found) {
        throw std::out_of_range("min_penalised: disjoint domains");
    }
    return {best, best_x};
}

}  // namespace kayros
