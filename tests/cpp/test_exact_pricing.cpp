// Gates of the exact pricing pass (M2 design note, section 8, criteria 1 and 2),
// on small generated instances and on hand-made ones.
//
// G1  the leg code of the pass gives td::route_timing on every customer sequence.
// G2  with dominance off and zero duals, the pass enumerates exactly the feasible
//     routes of an independent brute force over all sequences, with equal costs.
// G3  with dominance on, the minimum reduced cost equals that of the brute force,
//     on random multipliers, cuts and forbidden arcs, and on 3 hand-made cases.
//
// The brute force uses only the header (td::Arc, td::Customer, td::route_timing)
// on the raw binary64 data. It shares no code with the pass.
//
// Exit code: 0 when every gate passes. Built against a mutated copy of the pass
// (KAYROS_EXACT_MUTANT_TEST), it exits 0 only when a G3 minimum differs from the
// brute force, i.e. when the gates catch the mutant, and 1 when the mutant survives.
#include "exact/exact_pricing.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace ex = kayros::exact;
namespace td = mamut_routing::td_exact;
using ex::Q;

namespace {

struct Rng {
    std::uint64_t s;
    std::uint64_t next() {  // splitmix64
        std::uint64_t z = (s += 0x9e3779b97f4a7c15ULL);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
        return z ^ (z >> 31);
    }
    int uni(int a, int b) { return a + static_cast<int>(next() % static_cast<std::uint64_t>(b - a + 1)); }
    bool coin(int percent) { return uni(1, 100) <= percent; }
};

struct Data {
    long long cap = 0;
    double lo = 0, hi = 0;
    std::optional<double> close;
    std::vector<ex::CustomerInput> cust;  // customer k+1
    std::vector<ex::ArcInput> arcs;
    int n() const { return static_cast<int>(cust.size()); }
};

ex::ArcInput arc(int i, int j, std::vector<double> xs, std::vector<double> ys) { return {i, j, std::move(xs), std::move(ys)}; }

// Value grid: multiples of 1/4 (dyadic, exact ties) or of 1/3 rounded to binary64 (long denominators).
double val(Rng &r, int a, int b, bool dyadic) { return dyadic ? r.uni(4 * a, 4 * b) / 4.0 : r.uni(3 * a, 3 * b) / 3.0; }

ex::ArcInput random_atf(Rng &r, int i, int j, double H, bool jumps, bool dyadic) {
    std::vector<double> xs{0}, ys;
    const int inner = r.uni(0, 4);
    std::vector<double> cut;
    for (int k = 0; k < inner; ++k) cut.push_back(val(r, 1, static_cast<int>(H) - 1, dyadic));
    std::sort(cut.begin(), cut.end());
    cut.erase(std::unique(cut.begin(), cut.end()), cut.end());
    for (double x : cut) xs.push_back(x);
    xs.push_back(H);
    double y = 0;
    for (std::size_t k = 0; k < xs.size(); ++k) {
        y = std::max(y, xs[k] + val(r, 1, 25, dyadic));
        ys.push_back(y);
        if (jumps && k > 0 && k + 1 < xs.size() && r.coin(40)) {  // FIFO jump: repeated abscissa, higher arrival
            y += val(r, 1, 12, dyadic);
            xs.insert(xs.begin() + static_cast<long>(k) + 1, xs[k]);
            ys.push_back(y);
            ++k;
        }
    }
    return arc(i, j, xs, ys);
}

Data random_instance(Rng &r, int n, bool jumps, bool dyadic) {
    Data d;
    const double H = 128;
    d.cap = r.uni(3, 7);
    d.lo = r.coin(50) ? 0 : val(r, 0, 10, dyadic);
    d.hi = r.coin(50) ? H : val(r, 40, 120, dyadic);
    if (r.coin(50)) d.close = val(r, 70, 128, dyadic);
    for (int v = 1; v <= n; ++v) {
        ex::CustomerInput c;
        if (r.coin(80)) {
            const double o = val(r, 0, 70, dyadic);
            c.window = std::make_pair(o, o + val(r, 0, 50, dyadic));
        }
        c.service = r.coin(30) ? 0 : val(r, 0, 4, dyadic);
        c.demand = r.uni(0, 3);
        d.cust.push_back(c);
    }
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= n; ++j)
            if (i != j && !(i == 0 && j == 0) && (i == 0 || j == 0 || r.coin(90))) d.arcs.push_back(random_atf(r, i, j, H, jumps, dyadic));
    return d;
}

ex::Instance build(const Data &d) { return ex::Instance(d.cap, d.lo, d.hi, d.close, d.cust, d.arcs); }

// ---------- brute force on the header alone ----------
struct Brute {
    std::map<std::vector<int>, std::optional<Q>> timing;  // every sequence within capacity: duration or infeasible
    std::map<std::vector<int>, Q> feasible;
};

Brute brute_force(const Data &d) {
    const int n = d.n();
    std::map<std::pair<int, int>, td::Arc> arcs;
    for (const auto &e : d.arcs) arcs.emplace(std::make_pair(e.from, e.to), td::Arc(e.xs.data(), e.ys.data(), e.xs.size()));
    std::vector<td::Customer> cust;
    for (const auto &c : d.cust) cust.emplace_back(c.window, c.service);
    Brute b;
    std::vector<int> seq;
    std::function<void(long long)> rec = [&](long long load) {
        if (!seq.empty()) {
            std::vector<const td::Arc *> legs;
            std::vector<td::Customer> cs;
            int v = 0;
            bool ok = true;
            for (std::size_t k = 0; k <= seq.size() && ok; ++k) {
                const int w = k < seq.size() ? seq[k] : 0;
                auto it = arcs.find({v, w});
                if (it == arcs.end()) ok = false;
                else legs.push_back(&it->second);
                if (k < seq.size()) cs.push_back(cust[w - 1]);
                v = w;
            }
            std::optional<Q> dur;
            if (ok)
                if (auto t = td::route_timing(legs, cs, d.lo, d.hi, d.close)) dur = t->duration;
            b.timing[seq] = dur;
            if (dur) b.feasible[seq] = *dur;
        }
        for (int w = 1; w <= n; ++w) {
            if (std::find(seq.begin(), seq.end(), w) != seq.end() || load + d.cust[w - 1].demand > d.cap) continue;
            seq.push_back(w);
            rec(load + d.cust[w - 1].demand);
            seq.pop_back();
        }
    };
    rec(0);
    return b;
}

// Reduced cost of every feasible route avoiding the forbidden arcs, minimum and negative count.
struct Reference {
    std::optional<Q> min;
    long long negatives = 0;
    std::map<std::vector<int>, Q> rc;
};

Reference reference(const Brute &b, const ex::Duals &D) {
    std::set<std::pair<int, int>> forb(D.forbidden.begin(), D.forbidden.end());
    Reference ref;
    for (const auto &[seq, dur] : b.feasible) {
        bool ok = !forb.count({0, seq.front()}) && !forb.count({seq.back(), 0});
        for (std::size_t k = 0; k + 1 < seq.size(); ++k) ok = ok && !forb.count({seq[k], seq[k + 1]});
        if (!ok) continue;
        Q rc = dur;
        for (int v : seq) rc -= D.pi[v];
        for (std::size_t c = 0; c < D.cuts.size(); ++c) {
            int hits = 0;
            for (int v : seq) hits += v == D.cuts[c][0] || v == D.cuts[c][1] || v == D.cuts[c][2];
            if (hits >= 2) rc -= D.sigma[c];
        }
        ref.rc[seq] = rc;
        if (!ref.min || rc < *ref.min) ref.min = rc;
        ref.negatives += rc < 0;
    }
    return ref;
}

Q frac(long a, long b) {
    Q q(a, b);
    q.canonicalize();
    return q;
}

ex::Duals random_duals(Rng &r, int n) {
    ex::Duals D;
    D.pi.assign(n + 1, Q(0));
    const int mode = r.uni(0, 2);
    for (int v = 1; v <= n; ++v)
        D.pi[v] = mode == 0 ? Q(r.uni(0, 60)) : mode == 1 ? frac(r.uni(0, 240), 4) : frac(r.uni(0, 600000), 9973);
    const int ncut = n >= 3 ? r.uni(0, 4) : 0;
    for (int c = 0; c < ncut; ++c) {
        std::array<int, 3> S;
        do S = {r.uni(1, n), r.uni(1, n), r.uni(1, n)};
        while (S[0] == S[1] || S[0] == S[2] || S[1] == S[2]);
        D.cuts.push_back(S);
        D.sigma.push_back(r.coin(40) ? Q(-r.uni(0, 30)) : frac(-r.uni(0, 3000), 97));
    }
    const int nf = r.uni(0, 5);
    for (int f = 0; f < nf; ++f) {
        int i = r.uni(0, n), j = r.uni(0, n);
        if (i != j) D.forbidden.push_back({i, j});
    }
    return D;
}

// ---------- bookkeeping ----------
struct Tally {
    long long checks = 0, other_failures = 0, min_mismatches = 0;
    void fail(const std::string &what) {
        ++other_failures;
        if (other_failures <= 20) std::cerr << "FAIL " << what << '\n';
    }
};

std::string str(const std::optional<Q> &q) { return q ? q->get_str() : "none"; }

// G3 on one setting: dominance minimum against the reference, plus checks of the oracle itself.
void g3(const std::string &tag, const ex::Instance &I, const Brute &b, const ex::Duals &D, Tally &t) {
    const Reference ref = reference(b, D);
    ex::Options nodom;
    nodom.dominance = false;
    const ex::Result rn = ex::price(I, D, nodom);
    ++t.checks;
    if (rn.status != ex::Status::finished || rn.min_rc != ref.min || rn.negative_count != ref.negatives)
        t.fail(tag + " no-dominance pass: min " + str(rn.min_rc) + " expected " + str(ref.min));
    const ex::Result rd = ex::price(I, D);
    ++t.checks;
    if (rd.status != ex::Status::finished) t.fail(tag + " dominance pass not finished");
    if (rd.min_rc != ref.min) {
        ++t.min_mismatches;
        if (t.min_mismatches <= 5) std::cerr << "MIN MISMATCH " << tag << ": " << str(rd.min_rc) << " expected " << str(ref.min) << '\n';
    }
    for (const auto &[route, rc] : rd.negatives) {
        auto it = ref.rc.find(route);
        if (it == ref.rc.end() || it->second != rc) t.fail(tag + " negative route with a wrong reduced cost");
    }
}

void random_gates(Tally &t) {
    int instances = 0;
    long long g1 = 0, g2 = 0, settings = 0;
    for (int seed = 1; seed <= 48; ++seed) {
        Rng r{static_cast<std::uint64_t>(seed) * 1000003ULL};
        const int n = 4 + seed % 4;            // 4 to 7 customers
        const bool jumps = seed % 3 != 0;      // 2 of 3 instances carry FIFO jumps
        const bool dyadic = seed % 2 == 0;
        const Data d = random_instance(r, n, jumps, dyadic);
        const ex::Instance I = build(d);
        const Brute b = brute_force(d);
        const std::string tag = "seed " + std::to_string(seed);
        ++instances;
        // G1
        for (const auto &[seq, dur] : b.timing) {
            ++t.checks, ++g1;
            if (ex::path_duration(I, seq) != dur) t.fail(tag + " G1 duration differs from route_timing");
        }
        // G2
        std::map<std::vector<int>, Q> seen;
        ex::Options enumerate;
        enumerate.dominance = false;
        enumerate.on_route = [&](const std::vector<int> &p, const Q &dur, const Q &) { seen[p] = dur; };
        ex::Duals zero;
        zero.pi.assign(n + 1, Q(0));
        ex::price(I, zero, enumerate);
        ++t.checks, ++g2;
        if (seen != b.feasible)
            t.fail(tag + " G2 enumeration: " + std::to_string(seen.size()) + " routes, brute force " + std::to_string(b.feasible.size()));
        // G3
        for (int s = 0; s < 40; ++s) {
            ++settings;
            g3(tag + " setting " + std::to_string(s), I, b, random_duals(r, n), t);
        }
    }
    std::cout << "random: " << instances << " instances, G1 " << g1 << " sequences, G2 " << g2 << " enumerations, G3 " << settings << " settings\n";
}

// ---------- hand-made cases ----------
Data base(int n, long long cap) {
    Data d;
    d.cap = cap, d.lo = 0, d.hi = 100;
    d.cust.assign(n, ex::CustomerInput{std::nullopt, 0, 1});
    return d;
}
ex::ArcInput flat(int i, int j, double travel) { return arc(i, j, {0, 100}, {travel, 100 + travel}); }

// Rule (c) needs the right limits. M = (0, 2) reaches 2 through an ATF with a FIFO jump at departure 5,
// L = (0, 1, 2) is continuous and crosses back above M's curve before its next breakpoint.
// Without the right-limit comparison L is discarded, and the route (1, 2, 3) of reduced cost -956/5 is lost.
Data right_limit_case() {
    Data d = base(3, 10);
    d.cust[2].window = std::make_pair(13.0, 13.0);
    d.arcs = {flat(0, 1, 1), arc(1, 2, {0, 6, 8, 100}, {5, 11, 21, 113}), arc(0, 2, {0, 5, 5, 100}, {1, 6, 15, 110}),
              flat(2, 3, 1), flat(3, 0, 1)};
    return d;
}

// Rule (c) needs the cut penalty. M = (0, 1, 4) has met 1 member of the cut {1, 2, 3}, L = (0, 4) none.
// Customer 1 is unreachable from L (window closes at 5), so W_M = W_L. Without pen, M discards L,
// and the route (4, 2) of reduced cost -35 is lost.
Data cut_penalty_case() {
    Data d = base(4, 10);
    d.cust[0].window = std::make_pair(0.0, 5.0);
    d.cust[0].demand = 0;
    d.cust[3].window = std::make_pair(0.0, 7.0);
    d.arcs = {flat(0, 1, 1), flat(1, 4, 1), flat(0, 4, 3), flat(4, 1, 10), flat(4, 2, 1), flat(2, 0, 1)};
    return d;
}

// Rule (a) is needed. M = (0, 1, 2) has a better curve than L = (0, 2), but 1 is still reachable from L.
// Customer 1 has demand 0, so q_M = q_L. Without the inclusion of the sets, M discards L and the
// route (2, 1) of reduced cost -89 is lost.
Data set_inclusion_case() {
    Data d = base(2, 10);
    d.cust[0].demand = 0;
    d.arcs = {flat(0, 1, 1), flat(1, 2, 1), flat(0, 2, 10), flat(2, 1, 1), flat(1, 0, 1)};
    return d;
}

void hand_case(const std::string &tag, const Data &d, ex::Duals D, const Q &expected, Tally &t) {
    const ex::Instance I = build(d);
    const Brute b = brute_force(d);
    const Reference ref = reference(b, D);
    ++t.checks;
    if (ref.min != expected) t.fail(tag + " brute force minimum " + str(ref.min) + " differs from the hand computation " + expected.get_str());
    g3(tag, I, b, D, t);
}

void hand_gates(Tally &t) {
    ex::Duals D;
    D.pi = {0, 0, 100, 100};
    hand_case("right-limit case", right_limit_case(), D, Q(-956, 5), t);
    D = {};
    D.pi = {0, 0, 20, 0, 20};
    D.cuts = {{1, 2, 3}};
    D.sigma = {Q(-10)};
    hand_case("cut-penalty case", cut_penalty_case(), D, Q(-35), t);
    D = {};
    D.pi = {0, 1, 100};
    hand_case("set-inclusion case", set_inclusion_case(), D, Q(-89), t);
    std::cout << "hand-made: 3 cases\n";
}

// curve_dominated on hand-built curves: f(x) = x on [0, 2] and g jumps from 0 to 1/2 at 0.
// Condition (c) fails on (0, 1/2), which only the right limit at 0 reveals.
void curve_gates(Tally &t) {
    const ex::Map f{{Q(0), Q(0), Q(0)}, {Q(2), Q(2), Q(2)}};
    const ex::Map g{{Q(0), Q(0), Q(1, 2)}, {Q(2), Q(1, 2), Q(1, 2)}};
    const ex::Map h{{Q(0), Q(0), Q(0)}, {Q(2), Q(1, 2), Q(1, 2)}};
    ++t.checks;
    if (ex::curve_dominated(f, g, Q(0))) t.fail("curve case: a jump above f right after 0 must refuse dominance");
    ++t.checks;
    if (!ex::curve_dominated(f, h, Q(0))) t.fail("curve case: a continuous g below f must give dominance");
    std::cout << "curves: 2 cases\n";
}

// The shadow may only reject: on curves of random open paths and shifts delta near the
// critical values, shadow_rejects implies that the exact test refuses.
void shadow_gates(Tally &t) {
    long long pairs = 0, rejected = 0, refused = 0;
    for (int seed = 1; seed <= 24; ++seed) {
        Rng r{static_cast<std::uint64_t>(seed) * 7919ULL};
        const int n = 4 + seed % 3;
        const Data d = random_instance(r, n, seed % 3 != 0, seed % 2 == 0);
        const ex::Instance I = build(d);
        std::vector<std::vector<ex::Map>> at(n + 1);  // curves of open paths, by last vertex
        for (int k = 0; k < 300; ++k) {
            ex::Map f = ex::initial_curve(I);
            int v = 0;
            std::vector<bool> seen(n + 1, false);
            for (int len = r.uni(1, n); len > 0 && !f.empty(); --len) {
                const int w = r.uni(1, n);
                if (seen[w] || !I.a(v, w) || !ex::extend_curve(I, f, v, w)) break;
                seen[w] = true, v = w;
                at[w].push_back(f);
            }
        }
        for (int v = 1; v <= n; ++v)
            for (std::size_t a = 0; a < at[v].size() && a < 40; ++a)
                for (std::size_t b = 0; b < at[v].size() && b < 40; ++b) {
                    const ex::Map &f = at[v][a], &g = at[v][b];
                    // Shifts that make the comparison tight: differences of abscissas, and small offsets.
                    const Q base = f.back().x - g.back().x;
                    const std::vector<Q> deltas{Q(0), Q(-base), base, Q(-base + Q(1, 1024)), Q(-base - Q(1, 1024)), Q(r.uni(-40, 40))};
                    for (const Q &delta : deltas) {
                        ++pairs, ++t.checks;
                        const bool rej = ex::shadow_rejects(f, g, delta);
                        const bool dom = ex::curve_dominated(f, g, delta);
                        rejected += rej, refused += !dom;
                        if (rej && dom) t.fail("shadow rejected a dominance that the exact test accepts");
                    }
                }
    }
    std::cout << "shadow: " << pairs << " curve pairs, exact refusals " << refused << ", shadow rejections " << rejected << '\n';
}

// A label limit makes the pass unresolved: no minimum and no best route are returned,
// and every negative route returned is genuine, with its exact reduced cost.
void limit_gates(Tally &t) {
    long long unresolved = 0, negatives = 0;
    for (int seed = 1; seed <= 12; ++seed) {
        Rng r{static_cast<std::uint64_t>(seed) * 104729ULL};
        const int n = 5 + seed % 3;
        const Data d = random_instance(r, n, seed % 2 == 0, seed % 3 == 0);
        const ex::Instance I = build(d);
        const Brute b = brute_force(d);
        for (int s = 0; s < 10; ++s) {
            const ex::Duals D = random_duals(r, n);
            const Reference ref = reference(b, D);
            for (const bool dominance : {true, false}) {
                ex::Options o;
                o.dominance = dominance;
                o.label_limit = r.uni(1, 8);
                const ex::Result res = ex::price(I, D, o);
                ++t.checks;
                if (res.counters.created < o.label_limit) continue;  // the pass ended before the limit
                ++unresolved;
                if (res.status != ex::Status::unresolved || res.min_rc || !res.best_route.empty())
                    t.fail("a pass stopped by its label limit must be unresolved without a minimum");
                for (const auto &[route, rc] : res.negatives) {
                    ++negatives;
                    auto it = ref.rc.find(route);
                    if (it == ref.rc.end() || it->second != rc) t.fail("unresolved pass returned a wrong negative route");
                }
            }
        }
    }
    std::cout << "limits: " << unresolved << " unresolved passes, " << negatives << " negative routes checked\n";
}

}  // namespace

int main() {
    Tally t;
    try {
#ifndef KAYROS_EXACT_MUTANT_TEST  // the mutation check counts minima only (criterion 2)
        curve_gates(t);
        shadow_gates(t);
        limit_gates(t);
#endif
        hand_gates(t);
        random_gates(t);
    } catch (const std::exception &e) {
        std::cerr << "exception: " << e.what() << '\n';
        return 3;
    }
    std::cout << t.checks << " checks, " << t.other_failures << " other failures, " << t.min_mismatches << " minimum mismatches\n";
#ifdef KAYROS_EXACT_MUTANT_TEST
    if (t.other_failures == 0 && t.min_mismatches > 0) { std::cout << "mutant caught\n"; return 0; }
    std::cout << (t.min_mismatches == 0 ? "MUTANT SURVIVED\n" : "unexpected failures besides the minimum\n");
    return t.min_mismatches == 0 ? 1 : 2;
#else
    return t.other_failures == 0 && t.min_mismatches == 0 ? 0 : 1;
#endif
}
