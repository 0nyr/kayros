#include "core/warp_eval.h"

#include <algorithm>

namespace kayros {

namespace {

// The zero function on [lo, hi] (single point when lo == hi).
Pwlf zero_on(double lo, double hi) {
    if (lo == hi) return {{lo}, {0.0}};
    return {{lo, hi}, {0.0, 0.0}};
}

}  // namespace

double warp_horizon(const Instance& inst) {
    double t_end = inst.horizon_end;
    for (double y : inst.atf_ys) {
        if (y > t_end) t_end = y;
    }
    if (inst.has_time_windows) {
        const std::int32_t nv = inst.num_vertices();
        for (std::int32_t v = 0; v < nv; ++v) {
            const double done = inst.tw_latest[v] + inst.service_times[v];
            if (done > t_end) t_end = done;
        }
    }
    return t_end;
}

WarpFunctions warp_route_functions(const Instance& inst, const std::int32_t* route,
                                   std::int64_t len, double t_end, bool dedup) {
    double dep_lo = inst.horizon_start;
    double dep_hi = inst.horizon_end;
    if (inst.has_time_windows) {
        dep_lo = std::max(dep_lo, inst.tw_earliest[0]);
        dep_hi = std::min(dep_hi, inst.tw_latest[0]);
    }
    if (dep_lo > dep_hi) return {};

    Pwlf acc = identity(dep_lo, dep_hi);
    Pwlf warp = zero_on(dep_lo, dep_hi);
    std::int32_t prev = 0;
    for (std::int64_t k = 0; k < len; ++k) {
        const std::int32_t v = route[k];
        // A_k: pre-clamp arrival at v as a function of the depot departure.
        Pwlf arrival = compose(inst.arc(prev, v), view(acc));
        if (arrival.xs.empty()) return {};  // hard horizon wall
        const double service_time = inst.service_times[v];
        if (inst.has_time_windows) {
            const ThetaWarp tw = make_theta_warp(
                inst.tw_earliest[v], inst.tw_latest[v], service_time, t_end);
            acc = compose(view(tw.theta), view(arrival));
            warp = add(view(warp), view(compose(view(tw.warp), view(arrival))));
        } else {
            // TDVRP: pure service shift, exactly the base fold (zero warp).
            const double upper = arrival.ys.back();
            const Pwlf theta{{0.0, upper}, {service_time, upper + service_time}};
            acc = compose(view(theta), view(arrival));
        }
        if (acc.xs.empty()) return {};
        if (dedup) {
            dedup_safe_runs(acc);
            dedup_safe_runs(warp);
        }
        prev = v;
    }

    Pwlf arrival = compose(inst.arc(prev, 0), view(acc));
    if (arrival.xs.empty()) return {};
    if (inst.has_time_windows) {
        // The route ends upon arrival: clamp at the depot due date and book
        // the excess as warp (base fold: identity(0, due) domain restriction).
        const double due = inst.tw_latest[0];
        acc = compose(view(make_return_clamp(due, t_end)), view(arrival));
        warp = add(view(warp), view(compose(view(make_return_warp(due, t_end)),
                                            view(arrival))));
    } else {
        acc = std::move(arrival);
    }
    if (acc.xs.empty()) return {};

    // Horizon truncations after a warp term was added may have shrunk rho's
    // domain: align the warp channel onto it (adding zero is exact).
    warp = add(view(warp), view(zero_on(acc.xs.front(), acc.xs.back())));
    if (dedup) {
        dedup_safe_runs(acc);
        dedup_safe_runs(warp);
    }
    return {std::move(acc), std::move(warp)};
}

WarpRouteEval evaluate_route_warp(const Instance& inst, const std::int32_t* route,
                                  std::int64_t len, double penalty, double t_end,
                                  bool dedup) {
    WarpRouteEval out;
    const WarpFunctions wf = warp_route_functions(inst, route, len, t_end, dedup);
    if (wf.rho.xs.empty()) return out;
    out.total = true;
    out.min_warp = wf.warp.ys.front();

    MinShift zero;
    if (min_zero_warp_duration(view(wf.rho), view(wf.warp), &zero)) {
        out.feasible = true;
        out.duration = zero.value;
        out.departure = zero.argmin_x;
    }
    const MinShift pen = min_penalised(view(wf.rho), view(wf.warp), penalty);
    out.penalised = pen.value;
    out.penalised_departure = pen.argmin_x;
    return out;
}

}  // namespace kayros
