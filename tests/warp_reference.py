"""Pure-Python reference twin of the warp-augmented fold (Stream 8, P8.1).

Mirrors ``cpp/pwlf/warp.cpp`` + ``cpp/core/warp_eval.cpp`` operation for
operation on top of the canonical ``mamut_routing_lib.td.pwlf.NDCPWLF``
algebra: every float expression here is the same expression as in the C++
(the equivalence gates assert bitwise identity of both channels). Exact
doubles, no epsilons — checker discipline.
"""

from __future__ import annotations

from mamut_routing_lib.td.pwlf import NDCPWLF


def _push_dedup(xs: list[float], ys: list[float], x: float, y: float) -> None:
    if xs and x == xs[-1] and y == ys[-1]:
        return
    xs.append(x)
    ys.append(y)


def make_theta_warp(
    earliest: float, latest: float, service_time: float, t_end: float
) -> tuple[NDCPWLF, NDCPWLF]:
    assert earliest <= latest <= t_end
    txs: list[float] = []
    tys: list[float] = []
    _push_dedup(txs, tys, 0.0, earliest + service_time)
    _push_dedup(txs, tys, earliest, earliest + service_time)
    _push_dedup(txs, tys, latest, latest + service_time)
    _push_dedup(txs, tys, t_end, latest + service_time)
    wxs: list[float] = []
    wys: list[float] = []
    _push_dedup(wxs, wys, 0.0, 0.0)
    _push_dedup(wxs, wys, latest, 0.0)
    _push_dedup(wxs, wys, t_end, t_end - latest)
    return NDCPWLF(txs, tys, validate=False), NDCPWLF(wxs, wys, validate=False)


def make_return_clamp(due: float, t_end: float) -> NDCPWLF:
    assert due <= t_end
    xs: list[float] = []
    ys: list[float] = []
    _push_dedup(xs, ys, 0.0, 0.0)
    _push_dedup(xs, ys, due, due)
    _push_dedup(xs, ys, t_end, due)
    return NDCPWLF(xs, ys, validate=False)


def make_return_warp(due: float, t_end: float) -> NDCPWLF:
    assert due <= t_end
    xs: list[float] = []
    ys: list[float] = []
    _push_dedup(xs, ys, 0.0, 0.0)
    _push_dedup(xs, ys, due, 0.0)
    _push_dedup(xs, ys, t_end, t_end - due)
    return NDCPWLF(xs, ys, validate=False)


def add(f: NDCPWLF, g: NDCPWLF) -> NDCPWLF:
    """f + g on dom(f) ∩ dom(g) — the C++ ``add`` bit for bit."""
    if f.is_empty() or g.is_empty():
        return NDCPWLF.empty()
    lo = max(f.xs[0], g.xs[0])
    hi = min(f.xs[-1], g.xs[-1])
    if lo > hi:
        return NDCPWLF.empty()

    def interp(h: NDCPWLF, i: int, x: float) -> float:
        x_lo, x_hi = h.xs[i - 1], h.xs[i]
        y_lo, y_hi = h.ys[i - 1], h.ys[i]
        t = (x - x_lo) / (x_hi - x_lo)
        return y_lo + t * (y_hi - y_lo)

    nf, ng = len(f.xs), len(g.xs)
    i = 0
    while f.xs[i] < lo:
        i += 1
    j = 0
    while g.xs[j] < lo:
        j += 1

    out_xs: list[float] = []
    out_ys: list[float] = []

    def emit(x: float, y: float) -> None:
        if out_xs:
            if x < out_xs[-1]:
                x = out_xs[-1]
            if y < out_ys[-1]:
                y = out_ys[-1]
            if x == out_xs[-1] and y == out_ys[-1]:
                return
        out_xs.append(x)
        out_ys.append(y)

    emitted = False
    while True:
        has_f = i < nf and f.xs[i] <= hi
        has_g = j < ng and g.xs[j] <= hi
        if not has_f and not has_g:
            break
        if not has_g:
            u = f.xs[i]
        elif not has_f:
            u = g.xs[j]
        else:
            u = f.xs[i] if f.xs[i] <= g.xs[j] else g.xs[j]

        if i < nf and f.xs[i] == u:
            f_lo_val = f.ys[i]
            k = i
            while k + 1 < nf and f.xs[k + 1] == u:
                k += 1
            f_hi_val = f.ys[k]
            i = k + 1
        else:
            f_lo_val = f_hi_val = interp(f, i, u)
        if j < ng and g.xs[j] == u:
            g_lo_val = g.ys[j]
            k = j
            while k + 1 < ng and g.xs[k + 1] == u:
                k += 1
            g_hi_val = g.ys[k]
            j = k + 1
        else:
            g_lo_val = g_hi_val = interp(g, j, u)

        emit(u, f_lo_val + g_lo_val)
        emit(u, f_hi_val + g_hi_val)
        emitted = True

    if not emitted:
        k = 0
        while f.xs[k] < lo:
            k += 1
        fv = f.ys[k] if f.xs[k] == lo else interp(f, k, lo)
        k = 0
        while g.xs[k] < lo:
            k += 1
        gv = g.ys[k] if g.xs[k] == lo else interp(g, k, lo)
        emit(lo, fv + gv)
    return NDCPWLF(out_xs, out_ys, validate=False)


def zero_prefix_end(w: NDCPWLF) -> float | None:
    if w.is_empty() or w.ys[0] != 0.0:
        return None
    k = 0
    while k + 1 < len(w.ys) and w.ys[k + 1] == 0.0:
        k += 1
    return w.xs[k]


def min_zero_warp_duration(rho: NDCPWLF, warp: NDCPWLF) -> tuple[float, float] | None:
    tau = zero_prefix_end(warp)
    if rho.is_empty() or tau is None:
        return None
    best = None
    best_x = None
    for x, y in zip(rho.xs, rho.ys):
        if x > tau:
            break
        value = y - x
        if best is None or value < best:
            best = value
            best_x = x
    if best is None:
        return None
    return best, best_x


def _zero_on(lo: float, hi: float) -> NDCPWLF:
    if lo == hi:
        return NDCPWLF([lo], [0.0], validate=False)
    return NDCPWLF([lo, hi], [0.0, 0.0], validate=False)


def warp_horizon(instance, atfs) -> float:
    """Mirror of the C++ ``warp_horizon`` over the lib's loaded objects."""
    t_end = float(atfs.horizon[1])
    for atf in atfs.arcs.values():
        if atf.ys and atf.ys[-1] > t_end:
            t_end = atf.ys[-1]
    time_windows = getattr(instance, "time_windows", None)
    if time_windows is not None:
        for v in range(instance.num_customers + 1):
            done = float(time_windows[v][1]) + float(instance.service_times[v])
            if done > t_end:
                t_end = done
    return t_end


def warp_route_functions(
    instance, atfs, route: list[int], t_end: float
) -> tuple[NDCPWLF, NDCPWLF]:
    """The warp-augmented left fold — mirror of the C++ ``warp_route_functions``.

    Returns (empty, empty) on a hard horizon wall.
    """
    horizon_start, horizon_end = atfs.horizon
    depot = instance.depot
    time_windows = getattr(instance, "time_windows", None)

    if time_windows is not None:
        dep_lo = max(horizon_start, float(time_windows[depot][0]))
        dep_hi = min(horizon_end, float(time_windows[depot][1]))
    else:
        dep_lo, dep_hi = horizon_start, horizon_end
    if dep_lo > dep_hi:
        return NDCPWLF.empty(), NDCPWLF.empty()

    acc = NDCPWLF.identity(dep_lo, dep_hi)
    warp = _zero_on(dep_lo, dep_hi)
    previous = depot
    for vertex in route:
        arrival = atfs.arcs[(previous, vertex)].compose(acc)
        if arrival.is_empty():
            return NDCPWLF.empty(), NDCPWLF.empty()
        service_time = float(instance.service_times[vertex])
        if time_windows is not None:
            theta, w = make_theta_warp(
                float(time_windows[vertex][0]),
                float(time_windows[vertex][1]),
                service_time,
                t_end,
            )
            acc = theta.compose(arrival)
            warp = add(warp, w.compose(arrival))
        else:
            upper = arrival.max_image
            theta = NDCPWLF([0.0, upper], [service_time, upper + service_time])
            acc = theta.compose(arrival)
        if acc.is_empty():
            return NDCPWLF.empty(), NDCPWLF.empty()
        previous = vertex

    arrival = atfs.arcs[(previous, depot)].compose(acc)
    if arrival.is_empty():
        return NDCPWLF.empty(), NDCPWLF.empty()
    if time_windows is not None:
        due = float(time_windows[depot][1])
        acc = make_return_clamp(due, t_end).compose(arrival)
        warp = add(warp, make_return_warp(due, t_end).compose(arrival))
    else:
        acc = arrival
    if acc.is_empty():
        return NDCPWLF.empty(), NDCPWLF.empty()

    warp = add(warp, _zero_on(acc.xs[0], acc.xs[-1]))
    return acc, warp
