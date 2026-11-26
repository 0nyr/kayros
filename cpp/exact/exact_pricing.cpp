// Exact pricing pass, port of the reviewed s139 prototype
// (tdvrptw-workspace src/experiments/s139_exact_pricer/exact_pricer.cpp).
// The algorithm is unchanged. See exact_pricing.h for the contract.
#include "exact/exact_pricing.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace kayros::exact {

Instance::Instance(long long capacity, double lo, double hi, std::optional<double> depot_close,
                   const std::vector<CustomerInput> &customers, const std::vector<ArcInput> &arcs)
    : n(static_cast<int>(customers.size())), cap(capacity) {
    if (n < 1 || n > max_customers) throw std::invalid_argument("customer count out of range for the exact pass");
    if (arcs.empty()) throw std::invalid_argument("no arc");
    const int N = n + 1;
    dem.assign(N, 0);
    cust.resize(N), has_win.assign(N, false), win_close.resize(N), service.resize(N);
    for (int v = 1; v <= n; ++v) {
        const CustomerInput &c = customers[v - 1];
        cust[v].emplace(c.window, c.service);
        dem[v] = c.demand;
        has_win[v] = c.window.has_value();
        win_close[v] = c.window ? td::exact(c.window->second) : Q(0);
        service[v] = td::exact(c.service);
    }
    arc.resize(N * N);
    std::vector<Q> m(N * N);
    for (const ArcInput &e : arcs) {
        if (e.from < 0 || e.from > n || e.to < 0 || e.to > n || e.from == e.to)
            throw std::invalid_argument("arc out of range");
        if (e.xs.size() != e.ys.size()) throw std::invalid_argument("ATF knot arrays of unequal length");
        auto &slot = arc[e.from * N + e.to];
        if (slot) throw std::invalid_argument("duplicate arc");
        slot.emplace(e.xs.data(), e.ys.data(), e.xs.size());
        // The minimum travel over a piecewise affine ATF is attained at a knot.
        Q best = td::exact(e.ys[0]) - td::exact(e.xs[0]);
        for (std::size_t t = 1; t < e.xs.size(); ++t) best = std::min(best, Q(td::exact(e.ys[t]) - td::exact(e.xs[t])));
        m[e.from * N + e.to] = best;
    }
    // td::arrival_curve refuses ATFs with unequal horizons. The pass checks it once.
    const td::Arc &domain = *arc[arcs.front().from * N + arcs.front().to];
    for (const auto &x : arc)
        if (x && (x->start() != domain.start() || x->end() != domain.end()))
            throw std::invalid_argument("ATFs have unequal horizons");
    hend = domain.end();
    low = std::max(td::exact(lo), domain.start());
    high = std::min(td::exact(hi), domain.end());
    if (depot_close) close = td::exact(*depot_close);
    // d(v, w) = shortest path with weights m(i, j) + s_j through customers only, minus s_w.
    // A lower bound of (arrival at w) - (ready time at v) along any path: travel >= m, waiting >= 0.
    std::vector<Q> D(N * N);
    std::vector<bool> fin(N * N, false);
    for (int i = 0; i <= n; ++i)
        for (int j = 1; j <= n; ++j)
            if (i != j && a(i, j)) D[i * N + j] = m[i * N + j] + service[j], fin[i * N + j] = true;
    for (int k = 1; k <= n; ++k)
        for (int i = 0; i <= n; ++i) {
            if (!fin[i * N + k]) continue;
            for (int j = 1; j <= n; ++j) {
                if (i == j || !fin[k * N + j]) continue;
                Q t = D[i * N + k] + D[k * N + j];
                if (!fin[i * N + j] || t < D[i * N + j]) D[i * N + j] = t, fin[i * N + j] = true;
            }
        }
    reach = fin;
    dmin.assign(N * N, Q(0));
    for (int i = 0; i <= n; ++i)
        for (int j = 1; j <= n; ++j)
            if (fin[i * N + j]) dmin[i * N + j] = D[i * N + j] - service[j];
}

namespace {

// Value of a nonempty curve at z in its domain: attained value, or right limit when right is set.
Q eval(const Map &f, const Q &z, bool right) {
    auto it = std::upper_bound(f.begin(), f.end(), z, [](const Q &x, const td::Knot &k) { return x < k.x; });
    const td::Knot &k = *(it - 1);
    if (k.x == z) return right ? k.right : k.at;
    const td::Knot &k1 = *it;
    return k.right + (k1.at - k.right) * (z - k.x) / (k1.x - k.x);
}

void check_strict(const Map &f) {
    for (std::size_t i = 1; i < f.size(); ++i)
        if (!(f[i - 1].x < f[i].x)) throw std::runtime_error("curve abscissas not strictly increasing");
}

// Binary64 shadow of a curve: for each knot, bounds lo <= x <= hi and lo <= at <= hi.
// It is used only to reject a dominance test early (see shadow_rejects).
struct Shadow {
    std::vector<double> x_lo, x_hi, at_lo, at_hi;
};

constexpr double inf = std::numeric_limits<double>::infinity();

// mpq_get_d truncates towards zero, so 1 step outwards gives a bound.
double down(const Q &q) {
    const double d = q.get_d();
    return std::isfinite(d) ? std::nextafter(d, -inf) : -inf;
}
double up(const Q &q) {
    const double d = q.get_d();
    return std::isfinite(d) ? std::nextafter(d, inf) : inf;
}

Shadow shadow(const Map &f) {
    Shadow s;
    for (const td::Knot &k : f) {
        s.x_lo.push_back(down(k.x)), s.x_hi.push_back(up(k.x));
        s.at_lo.push_back(down(k.at)), s.at_hi.push_back(up(k.at));
    }
    return s;
}

// True only if the exact test curve_dominated(f, g, delta) returns false. It looks for
// a knot x_i of f and a knot x_j of g with x_j <= x_i + delta and g_j.at > f_i.at, both
// certified by the bounds. Then either x_i + delta is beyond the last x of g, or, since g
// is nondecreasing (Claim 6), g(x_i + delta) >= g_j.at > f(x_i) at the breakpoint x_i,
// which the exact scan compares. A rejection is never turned into an acceptance.
bool shadow_rejects(const Shadow &f, const Shadow &g, double delta_lo) {
    std::size_t j = 0;
    bool have = false;  // g.x_hi[j] <= current lower bound of x_i + delta
    for (std::size_t i = 0; i < f.x_lo.size(); ++i) {
        // Round to nearest errs by at most half an ulp, so 1 step down bounds the sum.
        const double y = std::nextafter(f.x_lo[i] + delta_lo, -inf);
        if (!have && g.x_hi[0] <= y) have = true;
        if (!have) continue;
        while (j + 1 < g.x_hi.size() && g.x_hi[j + 1] <= y) ++j;
        if (g.at_lo[j] > f.at_hi[i]) return true;
    }
    return false;
}

struct Label {
    int v, parent, nvis;
    bool alive = true;
    long long q;
    Bits V, U;
    Q p, cp;
    std::vector<unsigned char> cnt;
    Map f;
    Shadow s;  // set when dominance is on
};

struct Pricer {
    const Instance &I;
    const Options &opt;
    std::vector<Q> pi, w;  // w = -sigma >= 0
    std::vector<std::array<int, 3>> cuts;
    std::vector<bool> forbid;  // (n+1)^2
    std::vector<Label> L;
    std::vector<std::vector<int>> bucket;
    std::vector<std::vector<int>> cuts_of;
    Result res;
    bool limit_hit = false;
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

    Pricer(const Instance &I, const Duals &d, const Options &o)
        : I(I), opt(o), pi(d.pi), cuts(d.cuts), forbid((I.n + 1) * (I.n + 1), false), bucket(I.n + 1), cuts_of(I.n + 1) {
        if (pi.size() != static_cast<std::size_t>(I.n + 1)) throw std::invalid_argument("pi must have n+1 entries");
        for (Q &x : pi) x.canonicalize();  // GMP compares canonical rationals only
        if (d.sigma.size() != cuts.size()) throw std::invalid_argument("one sigma per cut");
        for (std::size_t c = 0; c < cuts.size(); ++c) {
            const auto &S = cuts[c];
            for (int x : S)
                if (x < 1 || x > I.n) throw std::invalid_argument("cut vertex out of range");
            if (S[0] == S[1] || S[0] == S[2] || S[1] == S[2]) throw std::invalid_argument("cut vertices not distinct");
            Q sigma = d.sigma[c];
            sigma.canonicalize();
            if (sigma > 0) throw std::invalid_argument("positive cut dual sigma (expected sigma <= 0)");
            w.push_back(-sigma);
            for (int x : S) cuts_of[x].push_back(static_cast<int>(c));
        }
        for (auto [i, j] : d.forbidden) {
            if (i < 0 || i > I.n || j < 0 || j > I.n || i == j) throw std::invalid_argument("forbidden arc out of range");
            forbid[i * (I.n + 1) + j] = true;
        }
    }

    double elapsed() const { return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); }

    std::vector<int> path(int id) const {
        std::vector<int> r;
        for (; L[id].v != 0; id = L[id].parent) r.push_back(L[id].v);
        return {r.rbegin(), r.rend()};
    }

    // Claim 8: capacity, and t0 + d(v, w) > close_w with t0 the earliest ready time.
    void set_unreachable(Label &l) {
        l.U = l.V;
        const Q &t0 = l.f.front().at;
        for (int x = 1; x <= I.n; ++x) {
            if (l.V[x]) continue;
            const int e = l.v * (I.n + 1) + x;
            if (l.q + I.dem[x] > I.cap || (I.has_win[x] && I.reach[e] && t0 + I.dmin[e] > I.win_close[x])) l.U[x] = true;
        }
    }

    // Claim 7: M dominates L.
    bool dominates(const Label &M, const Label &L) {
        ++res.counters.dom_tests;
        if ((M.U & ~L.U).any() || M.q > L.q) return false;
        if (M.f.front().at > L.f.front().at) return false;  // necessary for (c) at the first x of L
        Q pen = 0;
        for (std::size_t c = 0; c < cuts.size(); ++c)
            if (M.cnt[c] == 1 && L.cnt[c] != 1) pen += w[c];
        const Q delta = (L.p - L.cp) - (M.p - M.cp - pen);
        if (shadow_rejects(L.s, M.s, down(delta))) { ++res.counters.shadow_rejections; return false; }
        return curve_dominated(L.f, M.f, delta, &res.counters.dom_scans);
    }

    void complete(int id) {
        if (forbid[L[id].v * (I.n + 1)] || !I.a(L[id].v, 0)) return;
        Map f = L[id].f;
        if (!extend_curve(I, f, L[id].v, 0)) return;
        ++res.counters.completed;
        const Q dur = min_duration(f);
        const Q rc = dur - L[id].p + L[id].cp;
        if (!res.min_rc || rc < *res.min_rc) res.min_rc = rc, res.best_route = path(id);
        if (rc < 0) {
            ++res.negative_count;
            if (res.negatives.size() < opt.negative_cap) res.negatives.push_back({path(id), rc});
        }
        if (opt.on_route) opt.on_route(path(id), dur, rc);
    }

    bool limits() {
        const long long created = res.counters.created;
        if (opt.label_limit > 0 && created >= opt.label_limit) return limit_hit = true;
        if ((created & 255) == 0 && opt.time_limit > 0 && elapsed() > opt.time_limit) return limit_hit = true;
        return false;
    }

    void run() {
        Label root;
        root.v = 0, root.parent = -1, root.nvis = 0, root.q = 0, root.p = 0, root.cp = 0;
        root.cnt.assign(cuts.size(), 0);
        root.f = initial_curve(I);
        if (root.f.empty()) return;
        L.push_back(std::move(root));
        std::vector<std::vector<int>> level(I.n + 2);
        level[0].push_back(0);
        Counters &st = res.counters;
        for (int k = 0; k <= I.n && !limit_hit; ++k) {
            for (std::size_t li = 0; li < level[k].size() && !limit_hit; ++li) {
                const int id = level[k][li];
                if (!L[id].alive) continue;
                ++st.extended;
                if (L[id].v != 0) complete(id);
                for (int x = 1; x <= I.n; ++x) {
                    const Label &P = L[id];
                    if (P.V[x] || P.q + I.dem[x] > I.cap || forbid[P.v * (I.n + 1) + x] || !I.a(P.v, x)) continue;
                    Label N;
                    N.f = P.f;
                    if (!extend_curve(I, N.f, P.v, x)) { ++st.empty; continue; }
                    check_strict(N.f);
                    N.v = x, N.parent = id, N.nvis = P.nvis + 1, N.q = P.q + I.dem[x];
                    N.V = P.V, N.V[x] = true, N.p = P.p + pi[x], N.cp = P.cp, N.cnt = P.cnt;
                    for (int c : cuts_of[x])
                        if (N.cnt[c] < 2 && ++N.cnt[c] == 2) N.cp += w[c];
                    ++st.created;
                    st.max_knots = std::max(st.max_knots, N.f.size());
                    set_unreachable(N);
                    if (opt.dominance) {
                        N.s = shadow(N.f);
                        bool out = false;
                        for (int s : bucket[x])
                            if (L[s].alive && dominates(L[s], N)) { out = true; break; }
                        if (out) { ++st.dominated; if (limits()) break; continue; }
                        auto &B = bucket[x];
                        std::size_t keep = 0;
                        for (int s : B) {
                            if (!L[s].alive) continue;
                            if (dominates(N, L[s])) { L[s].alive = false, Map().swap(L[s].f), L[s].s = {}, ++st.dominated; continue; }
                            B[keep++] = s;
                        }
                        B.resize(keep);
                        B.push_back(static_cast<int>(L.size()));
                    }
                    level[k + 1].push_back(static_cast<int>(L.size()));
                    L.push_back(std::move(N));
                    if (limits()) break;
                }
                if (!opt.dominance && L[id].v != 0) Map().swap(L[id].f);  // no later test reads it
            }
        }
    }
};

}  // namespace

Map initial_curve(const Instance &I) {
    if (I.high < I.low) return {};
    Map f{{I.low, I.low, I.low}};
    if (I.high > I.low) f.push_back({I.high, I.high, I.high});
    return f;
}

// One leg as in td::arrival_curve: restrict to the horizon end, compose, visit (customer) or close (depot).
bool extend_curve(const Instance &I, Map &f, int v, int w) {
    td::restrict_upper(f, I.hend);
    if (f.empty()) return false;
    f = td::compose(f, *I.a(v, w));
    if (w != 0) I.cust[w]->visit(f);
    else if (I.close) td::restrict_upper(f, *I.close);
    return !f.empty();
}

Q min_duration(const Map &f) {
    Q best = f.front().at - f.front().x;
    for (std::size_t i = 1; i < f.size(); ++i) best = std::min(best, Q(f[i].at - f[i].x));
    return best;
}

std::optional<Q> path_duration(const Instance &I, const std::vector<int> &customers) {
    Map f = initial_curve(I);
    int v = 0;
    for (std::size_t k = 0; k <= customers.size(); ++k) {
        if (f.empty()) return std::nullopt;
        const int w = k < customers.size() ? customers[k] : 0;
        if (w < 0 || w > I.n || !I.a(v, w)) return std::nullopt;
        if (!extend_curve(I, f, v, w)) return std::nullopt;
        v = w;
    }
    return min_duration(f);
}

bool curve_dominated(const Map &f, const Map &g, const Q &delta, long long *scans) {
    if (f.back().x + delta > g.back().x) return false;
    auto G = [&](const Q &b, bool right) {
        Q z = b + delta;
        return z < g.front().x ? g.front().at : eval(g, z, right);
    };
    if (G(f.front().x, false) > f.front().at) return false;  // cheap necessary test
    if (scans) ++*scans;
    std::vector<Q> bp;
    bp.reserve(f.size() + g.size());
    std::size_t i = 0, j = 0;
    while (i < f.size() || j < g.size()) {
        Q b;
        if (j == g.size() || (i < f.size() && f[i].x <= g[j].x - delta)) b = f[i++].x;
        else {
            b = g[j++].x - delta;
            if (!(f.front().x < b && b < f.back().x)) continue;
        }
        if (bp.empty() || bp.back() != b) bp.push_back(std::move(b));
    }
    for (std::size_t k = 0; k < bp.size(); ++k) {
        if (G(bp[k], false) > eval(f, bp[k], false)) return false;
        if (k + 1 < bp.size() && G(bp[k], true) > eval(f, bp[k], true)) return false;
    }
    return true;
}

bool shadow_rejects(const Map &f, const Map &g, const Q &delta) { return shadow_rejects(shadow(f), shadow(g), down(delta)); }

Result price(const Instance &instance, const Duals &duals, const Options &options) {
    Pricer P(instance, duals, options);
    P.run();
    P.res.seconds = P.elapsed();
    if (P.limit_hit) {
        P.res.status = Status::unresolved;
        P.res.min_rc.reset();
        P.res.best_route.clear();
    }
    return std::move(P.res);
}

}  // namespace kayros::exact
