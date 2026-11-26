// Tests of the ledger of the exact layer (M2 design note, section 11) and of its
// verifier program (criterion 6).
//
// E2E  a test driver plays the search without any LP. At each node it takes the
//      cost-sharing multipliers pi_i = min c(r)/|r| over the routes r of the node
//      that visit i (U when no route visits i), which give Delta = 0. It closes,
//      offers the routes of the node when they form the only solution left, then
//      prunes, and else branches on an undecided arc used by a route of the node.
//      The certified U must equal the optimum of a brute force over all partitions.
// L    at the root of random instances, delta, L and the verdict of close equal a
//      reference computed from the brute force, with random multipliers and cuts.
// HAND hand-made instances for each precondition, prune, the bound kept by close,
//      kappa, the comparison L >= U and the forbidden arcs of the right child.
// REC  exact text forms, record round trip through the verifier program, and
//      tampered records that the verifier must refuse.
//
// The brute force uses only the header (td::Arc, td::Customer, td::route_timing),
// and the routes of a node are selected by a predicate on the route written from
// Claim 2, not from the forbidden arcs of the ledger.
//
// Exit code: 0 when every check passes. Built against a mutated copy of the ledger
// (KAYROS_LEDGER_MUTANT_TEST) it exits 0 only when some check fails, i.e. when
// the tests catch the mutant.
#include "exact/ledger.h"

#include <sys/wait.h>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#ifndef KAYROS_LEDGER_VERIFY_PATH
#error "KAYROS_LEDGER_VERIFY_PATH must name the verifier program"
#endif
#ifndef KAYROS_LEDGER_TMP
#error "KAYROS_LEDGER_TMP must name a file prefix for the records"
#endif

namespace ex = kayros::exact;
namespace td = mamut_routing::td_exact;
using ex::Q;
using Routes = std::vector<std::vector<int>>;

namespace {

long long checks = 0, failures = 0;
void expect(bool ok, const std::string &what) {
    ++checks;
    if (ok) return;
    ++failures;
    if (failures <= 40) std::cerr << "FAIL " << what << '\n';
}

Q canon(Q q) {
    q.canonicalize();
    return q;
}
Q frac(long a, long b) { return canon(Q(a, b)); }

// ---------- random instances (generator of test_exact_pricing.cpp) ----------
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

double val(Rng &r, int a, int b, bool dyadic) { return dyadic ? r.uni(4 * a, 4 * b) / 4.0 : r.uni(3 * a, 3 * b) / 3.0; }

ex::ArcInput random_atf(Rng &r, int i, int j, double H, bool jumps, bool dyadic) {
    std::vector<double> xs{0}, ys;
    std::vector<double> cut;
    for (int k = r.uni(0, 4); k > 0; --k) cut.push_back(val(r, 1, static_cast<int>(H) - 1, dyadic));
    std::sort(cut.begin(), cut.end());
    cut.erase(std::unique(cut.begin(), cut.end()), cut.end());
    for (double x : cut) xs.push_back(x);
    xs.push_back(H);
    double y = 0;
    for (std::size_t k = 0; k < xs.size(); ++k) {
        y = std::max(y, xs[k] + val(r, 1, 25, dyadic));
        ys.push_back(y);
        if (jumps && k > 0 && k + 1 < xs.size() && r.coin(40)) {
            y += val(r, 1, 12, dyadic);
            xs.insert(xs.begin() + static_cast<long>(k) + 1, xs[k]);
            ys.push_back(y);
            ++k;
        }
    }
    return {i, j, xs, ys};
}

ex::InstanceData random_instance(Rng &r, int n, bool jumps, bool dyadic) {
    ex::InstanceData d;
    const double H = 128;
    d.capacity = r.uni(3, 7);
    d.lo = r.coin(50) ? 0 : val(r, 0, 10, dyadic);
    d.hi = r.coin(50) ? H : val(r, 40, 120, dyadic);
    if (r.coin(50)) d.depot_close = val(r, 70, 128, dyadic);
    for (int v = 1; v <= n; ++v) {
        ex::CustomerInput c;
        if (r.coin(80)) {
            const double o = val(r, 0, 70, dyadic);
            c.window = std::make_pair(o, o + val(r, 0, 50, dyadic));
        }
        c.service = r.coin(30) ? 0 : val(r, 0, 4, dyadic);
        c.demand = r.uni(0, 3);
        d.customers.push_back(c);
    }
    for (int i = 0; i <= n; ++i)
        for (int j = 0; j <= n; ++j)
            if (i != j && (i == 0 || j == 0 || r.coin(90))) d.arcs.push_back(random_atf(r, i, j, H, jumps, dyadic));
    return d;
}

// ---------- brute force on the header alone ----------
struct Brute {
    int n = 0;
    std::map<std::vector<int>, Q> routes;  // every feasible route within capacity, exact duration
};

Brute brute_force(const ex::InstanceData &d) {
    Brute b;
    b.n = static_cast<int>(d.customers.size());
    std::map<std::pair<int, int>, td::Arc> arcs;
    for (const auto &e : d.arcs) arcs.emplace(std::make_pair(e.from, e.to), td::Arc(e.xs.data(), e.ys.data(), e.xs.size()));
    std::vector<td::Customer> cust;
    for (const auto &c : d.customers) cust.emplace_back(c.window, c.service);
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
            if (ok)
                if (auto t = td::route_timing(legs, cs, d.lo, d.hi, d.depot_close)) b.routes[seq] = t->duration;
        }
        for (int w = 1; w <= b.n; ++w) {
            if (std::find(seq.begin(), seq.end(), w) != seq.end() || load + d.customers[w - 1].demand > d.capacity) continue;
            seq.push_back(w);
            rec(load + d.customers[w - 1].demand);
            seq.pop_back();
        }
    };
    rec(0);
    return b;
}

// Partition of all customers into at most `fleet` routes of `routes`, of least
// (best = true) or largest cost, by dynamic programming over subsets.
std::optional<std::pair<Q, Routes>> partition(const Brute &b, const std::map<std::vector<int>, Q> &routes, int fleet, bool best) {
    const int full = (1 << b.n) - 1;
    std::map<int, std::pair<Q, std::vector<int>>> pick;  // mask -> chosen route on that mask
    for (const auto &[r, c] : routes) {
        int m = 0;
        for (int v : r) m |= 1 << (v - 1);
        auto it = pick.find(m);
        if (it == pick.end() || (best ? c < it->second.first : c > it->second.first)) pick[m] = {c, r};
    }
    // f[k][mask]: cost of a partition of mask into exactly k routes, with the last route chosen.
    std::vector<std::vector<std::optional<Q>>> f(fleet + 1, std::vector<std::optional<Q>>(full + 1));
    std::vector<std::vector<int>> last(fleet + 1, std::vector<int>(full + 1, 0));
    f[0][0] = Q(0);
    for (int k = 1; k <= fleet; ++k)
        for (int mask = 1; mask <= full; ++mask)
            for (const auto &[m, cr] : pick) {
                if ((m & mask) != m || !f[k - 1][mask & ~m]) continue;
                const Q c = *f[k - 1][mask & ~m] + cr.first;
                if (!f[k][mask] || (best ? c < *f[k][mask] : c > *f[k][mask])) f[k][mask] = c, last[k][mask] = m;
            }
    std::optional<std::pair<Q, Routes>> out;
    for (int k = 1; k <= fleet; ++k) {
        if (!f[k][full] || (out && !(best ? *f[k][full] < out->first : *f[k][full] > out->first))) continue;
        Routes X;
        for (int kk = k, mask = full; kk > 0; mask &= ~last[kk][mask], --kk) X.push_back(pick[last[kk][mask]].second);
        out = std::make_pair(canon(*f[k][full]), X);
    }
    return out;
}

// Claim 2 as a predicate on a route: left forbids traversing i -> j, right asks
// that the vertex after i is j and the vertex before j is i whenever i or j is on
// the route (the depot counts as a vertex before the first and after the last).
bool allowed(const std::vector<int> &r, const std::vector<ex::Decision> &decisions) {
    std::vector<int> path{0};
    path.insert(path.end(), r.begin(), r.end());
    path.push_back(0);
    for (const ex::Decision &d : decisions)
        for (std::size_t k = 1; k + 1 < path.size(); ++k) {
            if (!d.right && path[k] == d.i && path[k + 1] == d.j) return false;
            if (d.right && path[k] == d.i && path[k + 1] != d.j) return false;
            if (d.right && path[k] == d.j && path[k - 1] != d.i) return false;
        }
    return true;
}

std::map<std::vector<int>, Q> routes_at(const Brute &b, const std::vector<ex::Decision> &decisions) {
    std::map<std::vector<int>, Q> out;
    for (const auto &[r, c] : b.routes)
        if (allowed(r, decisions)) out.emplace(r, c);
    return out;
}

struct Reference {
    std::optional<Q> delta, bound;
    std::map<std::vector<int>, Q> negatives;
};

// delta and L of Claim 1 from the routes of the node, computed without the pass.
Reference reference(const std::map<std::vector<int>, Q> &R, int n, int kappa, const std::vector<Q> &pi,
                    const std::vector<std::array<int, 3>> &cuts, const std::vector<Q> &sigma) {
    Reference ref;
    std::optional<Q> min;
    for (const auto &[r, c] : R) {
        Q rc = c;
        for (int v : r) rc -= pi[v];
        for (std::size_t k = 0; k < cuts.size(); ++k) {
            int hits = 0;
            for (int v : r) hits += v == cuts[k][0] || v == cuts[k][1] || v == cuts[k][2];
            if (hits >= 2) rc -= sigma[k];
        }
        if (!min || rc < *min) min = rc;
        if (rc < 0) ref.negatives[r] = rc;
    }
    if (!min) return ref;
    ref.delta = *min < 0 ? Q(-*min) : Q(0);
    Q L = -kappa * *ref.delta;
    for (int v = 1; v <= n; ++v) L += pi[v];
    for (const Q &s : sigma) L += s;
    ref.bound = canon(L);
    ref.delta = canon(*ref.delta);
    return ref;
}

// Every refused operation must leave the ledger as it was.
struct Snapshot {
    std::size_t record, open;
    std::optional<Q> U;
    explicit Snapshot(const ex::Ledger &l) : record(l.record().size()), open(l.open_count()), U(l.U()) {}
    bool same(const ex::Ledger &l) const { return record == l.record().size() && open == l.open_count() && U == l.U(); }
};

// ---------- E2E ----------
struct E2E {
    bool solvable = false, gave_up = false, certified = false;
    long long nodes = 0;
    std::string record;  // the written record of a certified run
};

E2E drive(const ex::InstanceData &d, std::optional<int> fleet, const std::string &tag) {
    E2E out;
    const Brute b = brute_force(d);
    const int n = b.n;
    const int K = fleet ? *fleet : n;
    const int kappa = std::min(n, K);
    const auto opt = partition(b, b.routes, K, true);
    const auto worst = partition(b, b.routes, K, false);
    ex::Ledger L(d, fleet);
    if (!opt) {
        expect(!L.certified(), tag + " no solution yet certified");
        return out;
    }
    out.solvable = true;
    expect(L.offer(worst->second).accepted, tag + " offer of the most expensive solution refused");
    std::vector<std::pair<int, std::vector<ex::Decision>>> stack{{0, {}}};
    while (!stack.empty() && out.nodes < 200000) {
        auto [id, decisions] = stack.back();
        stack.pop_back();
        ++out.nodes;
        const auto R = routes_at(b, decisions);
        std::vector<Q> pi(n + 1, Q(0));
        std::vector<bool> covered(n + 1, false);
        for (const auto &[r, c] : R)
            for (int v : r) {
                const Q share = canon(c / static_cast<long>(r.size()));
                if (!covered[v] || share < pi[v]) pi[v] = share;
                covered[v] = true;
            }
        for (int v = 1; v <= n; ++v)
            if (!covered[v]) pi[v] = *L.U();
        const Reference ref = reference(R, n, kappa, pi, {}, {});
        const Q U = *L.U();
        const ex::CloseOutcome c = L.close(id, {}, pi, {});
        expect(c.delta == ref.delta && c.bound == ref.bound, tag + " close: delta or L differs from the reference");
        expect(c.removed == (!ref.bound || *ref.bound >= U), tag + " close verdict");
        if (c.removed) continue;
        // Not removed. If the routes of the node are disjoint and cover every customer,
        // they are its only solution, cheaper than U: offer it and prune.
        std::vector<int> count(n + 1, 0);
        for (const auto &[r, cost] : R)
            for (int v : r) ++count[v];
        if (std::all_of(count.begin() + 1, count.end(), [](int x) { return x == 1; })) {
            Routes X;
            for (const auto &[r, cost] : R) X.push_back(r);
            if (static_cast<int>(X.size()) > K) {  // the bound ignores the fleet: the driver gives up
                out.gave_up = true;
                expect(!L.certified(), tag + " certified after the driver gave up");
                return out;
            }
            expect(L.offer(X).accepted, tag + " offer of the solution of a node refused");
            expect(L.prune(id).removed, tag + " prune after an improving offer did not remove the node");
            continue;
        }
        // Branch on an arc used by a route of the node and not decided on its path.
        std::optional<std::pair<int, int>> e;
        for (const auto &[r, cost] : R) {
            for (std::size_t k = 0; k + 1 < r.size() && !e; ++k) {
                const bool decided = std::any_of(decisions.begin(), decisions.end(),
                                                 [&](const ex::Decision &x) { return x.i == r[k] && x.j == r[k + 1]; });
                if (!decided) e = std::make_pair(r[k], r[k + 1]);
            }
            if (e) break;
        }
        if (!e) {
            expect(false, tag + " no arc to branch on at an open node");
            return out;
        }
        const ex::BranchOutcome br = L.branch(id, e->first, e->second);
        expect(br.accepted, tag + " branch refused");
        auto left = decisions, right = decisions;
        left.push_back({e->first, e->second, false});
        right.push_back({e->first, e->second, true});
        stack.push_back({br.right, right});
        stack.push_back({br.left, left});
    }
    out.certified = L.certified();
    if (out.certified) {
        expect(*L.U() == opt->first, tag + " certified U " + L.U()->get_str() + " differs from the optimum " + opt->first.get_str());
        std::ostringstream s;
        L.write(s);
        out.record = s.str();
    } else {
        expect(!stack.empty() || L.open_count() > 0, tag + " not certified with an empty stack");
    }
    return out;
}

int run_verifier(const std::string &record, const std::string &name, std::string *output = nullptr) {
    const std::string path = std::string(KAYROS_LEDGER_TMP) + "-" + name + ".txt";
    std::ofstream(path) << record;
    const std::string cmd = std::string("'") + KAYROS_LEDGER_VERIFY_PATH + "' '" + path + "' > '" + path + ".out' 2>&1";
    const int status = std::system(cmd.c_str());
    if (output) {
        std::ifstream in(path + ".out");
        std::stringstream s;
        s << in.rdbuf();
        *output = s.str();
    }
    return WIFEXITED(status) ? WEXITSTATUS(status) : 255;
}

std::vector<std::string> lines(const std::string &text) {
    std::vector<std::string> out;
    std::istringstream in(text);
    for (std::string l; std::getline(in, l);) out.push_back(l);
    return out;
}

std::string join(const std::vector<std::string> &ls) {
    std::string s;
    for (const auto &l : ls) s += l + '\n';
    return s;
}

long long e2e_certified = 0, e2e_runs = 0, e2e_nodes = 0, e2e_verified = 0, e2e_unsolvable = 0, e2e_gave_up = 0;

void e2e_gates() {
    for (int seed = 1; seed <= 60; ++seed) {
        Rng r{static_cast<std::uint64_t>(seed) * 2654435761ULL};
        const int n = 4 + seed % 4;  // 4 to 7
        const ex::InstanceData d = random_instance(r, n, seed % 3 != 0, seed % 2 == 0);
        // No fleet limit, a limit of n (not binding), and a limit of 2 (may bind, the driver may give up).
        const std::vector<std::optional<int>> fleets{std::nullopt, n, 2};
        for (const auto &fleet : fleets) {
            const std::string tag = "e2e seed " + std::to_string(seed) + " fleet " + (fleet ? std::to_string(*fleet) : "none");
            const E2E e = drive(d, fleet, tag);
            ++e2e_runs, e2e_nodes += e.nodes, e2e_certified += e.certified, e2e_unsolvable += !e.solvable, e2e_gave_up += e.gave_up;
            if (!fleet || *fleet == n) expect(e.certified || !e.solvable, tag + " not certified with a fleet limit that does not bind");
            if (e.certified && seed % 5 == 0) {
                std::string text;
                expect(run_verifier(e.record, "e2e", &text) == 0, tag + " verifier refused a certified record: " + text);
                ++e2e_verified;
            }
        }
    }
    std::cout << "e2e: " << e2e_runs << " runs, " << e2e_certified << " certified, " << e2e_unsolvable << " without solution, "
              << e2e_gave_up << " given up on the fleet limit, " << e2e_nodes << " nodes, " << e2e_verified << " records verified\n";
}

// ---------- L ----------
void bound_gates() {
    long long settings = 0, removed = 0, positive_delta = 0;
    for (int seed = 1; seed <= 30; ++seed) {
        Rng r{static_cast<std::uint64_t>(seed) * 40503ULL};
        const int n = 4 + seed % 4;
        const ex::InstanceData d = random_instance(r, n, seed % 2 == 0, seed % 3 == 0);
        const Brute b = brute_force(d);
        const auto opt = partition(b, b.routes, n, true);
        if (!opt) continue;
        for (const std::optional<int> fleet : {std::optional<int>(), std::optional<int>(n), std::optional<int>(3)}) {
            const int kappa = fleet ? std::min(n, *fleet) : n;
            const auto sol = partition(b, b.routes, fleet ? *fleet : n, true);
            if (!sol) continue;
            for (int s = 0; s < 8; ++s) {
                ex::Ledger L(d, fleet);
                expect(L.offer(sol->second).accepted, "bound: offer refused");
                // Half of the settings: random multipliers (Delta > 0 mostly). The other half: the
                // cost shares of the routes at the root, scaled by 9/10 to 11/10 (L near U).
                std::vector<Q> pi(n + 1, Q(0));
                for (int v = 1; v <= n; ++v) pi[v] = frac(r.uni(0, 4000), r.uni(1, 97));
                if (s % 2 == 1) {
                    std::vector<bool> covered(n + 1, false);
                    for (const auto &[route, cost] : b.routes)
                        for (int v : route) {
                            const Q share = canon(cost / static_cast<long>(route.size()));
                            if (!covered[v] || share < pi[v]) pi[v] = share;
                            covered[v] = true;
                        }
                    const Q scale = frac(r.uni(90, 110), 100);
                    for (int v = 1; v <= n; ++v) pi[v] = canon(pi[v] * scale);
                }
                std::vector<std::array<int, 3>> cuts;
                std::vector<Q> sigma;
                for (int k = r.uni(0, 3); k > 0; --k) {
                    std::array<int, 3> S;
                    do S = {r.uni(1, n), r.uni(1, n), r.uni(1, n)};
                    while (S[0] == S[1] || S[0] == S[2] || S[1] == S[2]);
                    cuts.push_back(S), sigma.push_back(frac(-r.uni(0, 300), r.uni(1, 13)));
                }
                const Reference ref = reference(b.routes, n, kappa, pi, cuts, sigma);
                const ex::CloseOutcome c = L.close(0, cuts, pi, sigma);
                ++settings, removed += c.removed, positive_delta += ref.delta && sgn(*ref.delta) > 0;
                expect(c.delta == ref.delta && c.bound == ref.bound, "bound: delta or L differs from the reference");
                expect(c.removed == (ref.bound && *ref.bound >= *L.U()), "bound: verdict differs from L >= U");
                // With dominance the pass returns some of the negative routes, each with its exact reduced cost.
                bool genuine = c.negative_count == static_cast<long long>(c.negatives.size());
                for (const auto &[route, rc] : c.negatives) genuine = genuine && ref.negatives.count(route) && ref.negatives.at(route) == rc;
                expect(genuine && (c.negatives.empty() == ref.negatives.empty()), "bound: a returned negative route is not genuine");
                expect(c.removed ? L.record().size() == 2 && L.certified() : L.record().size() == 1, "bound: record of a close");
            }
        }
    }
    std::cout << "bound: " << settings << " settings, " << removed << " removed, " << positive_delta << " with Delta > 0\n";
}

// ---------- HAND ----------
ex::ArcInput flat(int i, int j, double travel) { return {i, j, {0, 100}, {travel, 100 + travel}}; }

// 2 customers of demand 1. Routes (1) of cost 2, (2) of cost 4, and with chain = true
// also (1, 2) and (2, 1) of cost 4.
ex::InstanceData two(bool chain, long long capacity = 10) {
    ex::InstanceData d;
    d.capacity = capacity, d.lo = 0, d.hi = 100;
    d.customers.assign(2, ex::CustomerInput{std::nullopt, 0, 1});
    d.arcs = {flat(0, 1, 1), flat(1, 0, 1), flat(0, 2, 2), flat(2, 0, 2)};
    if (chain) d.arcs.push_back(flat(1, 2, 1)), d.arcs.push_back(flat(2, 1, 1));
    return d;
}

std::vector<Q> P(std::initializer_list<Q> v) { return v; }

void hand_gates(std::string *prune_record) {
    // Construction.
    for (int K : {0, -1}) {
        bool threw = false;
        try { ex::Ledger L(two(false), K); } catch (const std::invalid_argument &) { threw = true; }
        expect(threw, "fleet limit " + std::to_string(K) + " accepted at construction");
    }
    {
        ex::InstanceData d = two(false);
        d.customers[0].demand = -1;
        bool threw = false;
        try { ex::Ledger L(d, std::nullopt); } catch (const std::invalid_argument &) { threw = true; }
        expect(threw, "negative demand accepted at construction");
        d.customers[0].demand = 11;
        threw = false;
        try { ex::Ledger L(d, std::nullopt); } catch (const std::invalid_argument &) { threw = true; }
        expect(threw, "demand above the capacity accepted at construction");
    }
    // Offers.
    {
        ex::Ledger L(two(false), 2);
        expect(!L.certified() && L.open_count() == 1 && L.is_open(0), "initial state");
        const Snapshot s(L);
        expect(L.close(0, {}, P({0, 2, 4}), {}).reason == "no incumbent" && s.same(L), "close before any incumbent");
        expect(!L.prune(0).removed && s.same(L), "prune before any close");
        const std::vector<std::pair<Routes, std::string>> bad{
            {{{1}}, "missing customer"},           {{{1}, {2}, {1}}, "customer twice"},
            {{{1, 1}, {2}}, "customer twice in a route"}, {{{1, 0, 2}}, "depot inside a route"},
            {{{1}, {}, {2}}, "empty route"},       {{{1, 2}}, "route without its arcs"},
            {{{1}, {3}}, "unknown customer"},      {{}, "no route"}};
        for (const auto &[routes, what] : bad) {
            const ex::OfferOutcome o = L.offer(routes);
            expect(!o.accepted && s.same(L), "offer accepted: " + what);
        }
        const ex::OfferOutcome o = L.offer({{2}, {1}});
        expect(o.accepted && o.cost == Q(6) && L.U() == Q(6) && L.record().size() == 1, "offer of a solution");
        const Snapshot t(L);
        expect(!L.offer({{1}, {2}}).accepted && t.same(L), "offer of an equal cost accepted");
    }
    {
        ex::Ledger L(two(false), 1);
        expect(!L.offer({{1}, {2}}).accepted && !L.U(), "offer over the fleet limit accepted");
        ex::Ledger C(two(true, 1), std::nullopt);
        expect(!C.offer({{1, 2}}).accepted && !C.U(), "offer over capacity accepted");
        expect(C.offer({{1}, {2}}).accepted, "offer without a fleet limit refused");
    }
    {
        // Time-infeasible route: customer 2 closes at 1, before any arrival.
        ex::InstanceData d = two(true);
        d.customers[1].window = std::make_pair(0.0, 1.0);
        ex::Ledger L(d, std::nullopt);
        expect(!L.offer({{1}, {2}}).accepted && !L.U(), "offer of a time-infeasible route accepted");
    }
    // Close, kappa and the comparison L >= U on (1) and (2), U = 6, kappa = 2.
    {
        auto fresh = [] {
            ex::Ledger L(two(false), std::nullopt);
            L.offer({{1}, {2}});
            return L;
        };
        ex::Ledger L = fresh();
        ex::CloseOutcome c = L.close(0, {}, P({0, 4, 4}), {});  // rc(1) = -2, Delta = 2, L = 8 - 2*2 = 4
        expect(!c.removed && c.delta == Q(2) && c.bound == Q(4) && L.record().size() == 1, "kappa: L = sum pi - kappa Delta");
        L = fresh();
        c = L.close(0, {}, P({0, 2, frac(7, 2)}), {});  // Delta = 0, L = 11/2 < 6
        expect(!c.removed && c.bound == frac(11, 2) && L.is_open(0), "L = U - 1/2 removed the node");
        L = fresh();
        c = L.close(0, {}, P({0, 2, 4}), {});  // tie L = U
        expect(c.removed && c.bound == Q(6) && L.certified() && L.record().size() == 2, "tie L = U did not remove");
        // Preconditions of close.
        L = fresh();
        const Snapshot s(L);
        const std::vector<std::pair<std::string, std::function<ex::CloseOutcome()>>> bad{
            {"pi must have n+1 entries", [&] { return L.close(0, {}, P({2, 4}), {}); }},
            {"pi[0] must be 0", [&] { return L.close(0, {}, P({1, 2, 4}), {}); }},
            {"one sigma per cut", [&] { return L.close(0, {}, P({0, 2, 4}), P({0})); }},
            {"cut customers not distinct", [&] { return L.close(0, {{1, 1, 2}}, P({0, 2, 4}), P({0})); }},
            {"cut vertex not a customer", [&] { return L.close(0, {{0, 1, 2}}, P({0, 2, 4}), P({0})); }},
            {"node not open", [&] { return L.close(7, {}, P({0, 2, 4}), {}); }},
            {"node not open", [&] { return L.close(-1, {}, P({0, 2, 4}), {}); }}};
        for (const auto &[why, f] : bad) {
            const ex::CloseOutcome o = f();
            expect(!o.removed && o.reason == why && s.same(L), "close not refused for '" + why + "': " + o.reason);
        }
        expect(!L.prune(0).removed, "prune without stored bound after refused closes");
        // Branch preconditions and a removed node.
        for (auto [i, j] : std::vector<std::pair<int, int>>{{1, 1}, {0, 1}, {1, 0}, {1, 3}, {-1, 2}})
            expect(!L.branch(0, i, j).accepted && s.same(L), "branch accepted on arc " + std::to_string(i) + "," + std::to_string(j));
        expect(!L.branch(5, 1, 2).accepted && s.same(L), "branch on an unknown node");
        expect(L.close(0, {}, P({0, 2, 4}), {}).removed, "close of the root");
        const Snapshot t(L);
        expect(!L.branch(0, 1, 2).accepted && !L.close(0, {}, P({0, 2, 4}), {}).removed && !L.prune(0).removed && t.same(L),
               "operation on a removed node");
    }
    // A node never closed: not certified.
    {
        ex::Ledger L(two(true), std::nullopt);
        L.offer({{1, 2}});
        const ex::BranchOutcome b = L.branch(0, 1, 2);
        expect(b.accepted && b.left == 1 && b.right == 2 && L.open_count() == 2, "branch ids");
        expect(L.close(b.left, {}, P({0, 2, 2}), {}).removed, "close of the left child");
        expect(!L.certified() && L.open_count() == 1, "certified with an open node");
        expect(!L.branch(0, 1, 2).accepted, "branch on a branched node");
    }
    // Forbidden arcs of the 2 children of (1, 2), n = 2.
    {
        ex::Ledger L(two(true), std::nullopt);
        L.branch(0, 1, 2);
        auto sorted = [](std::vector<std::pair<int, int>> v) {
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
            return v;
        };
        expect(sorted(*L.forbidden(1)) == std::vector<std::pair<int, int>>{{1, 2}}, "forbidden arcs of the left child");
        expect(sorted(*L.forbidden(2)) == std::vector<std::pair<int, int>>{{0, 2}, {1, 0}}, "forbidden arcs of the right child");
        expect(!L.forbidden(0) && !L.forbidden(3), "forbidden arcs of a node not open");
        // At the right child only (1, 2) remains, of cost 4. With pi_1 = 3 the excluded
        // route (1), which ends with the arc (1, 0), would have reduced cost -1.
        L.offer({{1, 2}});
        const ex::CloseOutcome c = L.close(2, {}, P({0, 3, 0}), {});
        expect(c.delta == Q(0) && c.bound == Q(3) && c.negatives.empty(), "right child priced a route through a depot arc it forbids");
    }
    // Prune and the stored bound.
    {
        ex::Ledger L(two(true), std::nullopt);
        expect(L.offer({{1}, {2}}).accepted, "prune case: first offer");  // U = 6
        const ex::BranchOutcome b = L.branch(0, 1, 2);
        expect(!L.prune(b.left).removed, "prune without a stored bound removed the node");
        ex::CloseOutcome c = L.close(b.right, {}, P({0, 1, 1}), {});  // only (1, 2) of cost 4: L = 2
        expect(!c.removed && c.bound == Q(2) && L.record().size() == 2, "prune case: close below U is not recorded");
        c = L.close(b.right, {}, P({0, 2, 2}), {});  // L = 4, stored
        expect(!c.removed && c.bound == Q(4), "prune case: second close");
        c = L.close(b.right, {}, P({0, 1, 1}), {});  // L = 2 does not replace 4
        const Snapshot s(L);
        expect(!L.prune(b.right).removed && s.same(L), "prune with L = 4 < U = 6 removed the node");
        expect(L.offer({{1, 2}}).accepted && L.U() == Q(4), "prune case: improving offer");
        expect(L.prune(b.right).removed && !L.is_open(b.right), "prune after an improving offer did not remove");
        const ex::Operation &op = L.record().back();
        expect(op.kind == ex::Operation::Kind::close && op.certificate.pi == P({0, 2, 2}) && op.certificate.bound == Q(4) && op.U == Q(4),
               "prune records the close of the largest stored bound");
        expect(L.close(b.left, {}, P({0, 2, 2}), {}).removed && L.certified(), "prune case: close of the left child");
        std::ostringstream w;
        L.write(w);
        *prune_record = w.str();
    }
    // Cuts and sigma on a random instance with 5 customers.
    {
        ex::InstanceData d;
        std::optional<std::pair<Q, Routes>> opt;
        for (std::uint64_t seed = 77; !opt; ++seed) {
            Rng r{seed};
            d = random_instance(r, 5, true, false);
            const Brute b = brute_force(d);
            opt = partition(b, b.routes, 5, true);
        }
        ex::Ledger L(d, std::nullopt);
        L.offer(opt->second);
        const Snapshot s(L);
        std::vector<Q> pi(6, Q(1));
        pi[0] = 0;
        ex::CloseOutcome o = L.close(0, {{1, 2, 3}}, pi, P({frac(1, 3)}));
        expect(!o.removed && o.reason == "positive cut multiplier" && s.same(L), "positive sigma accepted: " + o.reason);
        o = L.close(0, {{1, 2, 2}}, pi, P({-1}));
        expect(!o.removed && o.reason == "cut customers not distinct" && s.same(L), "cut with a repeated customer accepted: " + o.reason);
        o = L.close(0, {{1, 2, 6}}, pi, P({-1}));
        expect(!o.removed && o.reason == "cut vertex not a customer" && s.same(L), "cut customer above n accepted: " + o.reason);
        // A tiny label limit leaves the pass unresolved: nothing changes, no bound is stored.
        ex::Limits lim;
        lim.label_limit = 1;
        const ex::CloseOutcome c = L.close(0, {}, pi, {}, lim);
        expect(!c.removed && c.reason == "exact pass unresolved" && !c.bound && s.same(L), "unresolved pass accepted: " + c.reason);
        expect(!L.prune(0).removed && s.same(L), "unresolved pass stored a bound");
    }
    std::cout << "hand-made: done\n";
}

// ---------- REC ----------
void record_gates(const std::string &prune_record) {
    // Binary64 and rational text forms.
    Rng r{12345};
    long long doubles = 0;
    for (int k = 0; k < 20000; ++k) {
        std::uint64_t bits = r.next();
        double x;
        std::memcpy(&x, &bits, sizeof x);
        if (!std::isfinite(x)) continue;
        const double y = ex::parse_double(ex::hex_double(x));
        ++doubles;
        expect(std::memcmp(&x, &y, sizeof x) == 0, "binary64 round trip of " + ex::hex_double(x));
    }
    for (double x : {0.0, -0.0, 1.0 / 3, 128.0, 5e-324, 1.7976931348623157e308}) {
        const double y = ex::parse_double(ex::hex_double(x));
        expect(std::memcmp(&x, &y, sizeof x) == 0, "binary64 round trip of " + ex::hex_double(x));
    }
    for (const std::string bad : {"2/4", "1/0", "abc", "", "1/-2", "+1", " 1"}) {
        bool threw = false;
        try { ex::parse_rational(bad); } catch (const std::exception &) { threw = true; }
        expect(threw, "rational token '" + bad + "' accepted");
    }
    for (const std::string bad : {"1.5", "0x1.8p+0 ", "inf", "nan", "0x1p+2000"}) {
        bool threw = false;
        try { ex::parse_double(bad); } catch (const std::exception &) { threw = true; }
        expect(threw, "binary64 token '" + bad + "' accepted");
    }
    expect(ex::parse_rational("-7/3") == frac(-7, 3), "rational -7/3");

    // The prune record: offer (6), branch, offer (4), close (from prune), close.
    std::string out;
    expect(run_verifier(prune_record, "prune", &out) == 0 && out.find("certified U=4 ") == 0, "verifier refused the prune record: " + out);
    {
        std::istringstream in(prune_record);
        const ex::Replay rep = ex::replay(in);
        expect(rep.certified && rep.U == Q(4) && rep.operations == 5 && rep.incumbent == Routes{{1, 2}}, "in-process replay of the prune record");
    }
    const std::vector<std::string> L = lines(prune_record);
    std::vector<std::size_t> offers, branches, closes;
    const std::size_t body = std::find(L.begin(), L.end(), "operations") - L.begin();
    for (std::size_t k = body + 1; k < L.size(); ++k) {
        if (L[k].rfind("offer ", 0) == 0) offers.push_back(k);
        if (L[k].rfind("branch ", 0) == 0) branches.push_back(k);
        if (L[k].rfind("close ", 0) == 0) closes.push_back(k);
    }
    expect(offers.size() == 2 && branches.size() == 1 && closes.size() == 2, "shape of the prune record");
    if (offers.size() != 2 || branches.size() != 1 || closes.size() != 2) return;
    auto tamper = [&](const std::string &what, std::function<void(std::vector<std::string> &)> f) {
        std::vector<std::string> t = L;
        f(t);
        std::string o;
        expect(run_verifier(join(t), "tamper", &o) != 0, "verifier accepted a record with " + what);
    };
    auto replace = [](std::string &s, const std::string &from, const std::string &to) {
        const std::size_t at = s.find(from);
        if (at != std::string::npos) s.replace(at, from.size(), to);
        return at != std::string::npos;
    };
    tamper("a pi value lowered at the tie leaf", [&](auto &t) { expect(replace(t[closes[1]], " pi 2 2 2 ", " pi 2 2 3/2 "), "tamper pi"); });
    tamper("a pi value raised", [&](auto &t) { expect(replace(t[closes[1]], " pi 2 2 2 ", " pi 2 2 5/2 "), "tamper pi up"); });
    tamper("a route changed", [&](auto &t) { expect(replace(t[offers[1]], "routes 1 2 1 2 ", "routes 1 2 1 1 "), "tamper route"); });
    tamper("a close line dropped", [&](auto &t) { t.erase(t.begin() + static_cast<long>(closes[1])); });
    tamper("the branch line dropped", [&](auto &t) { t.erase(t.begin() + static_cast<long>(branches[0])); });
    tamper("the improving offer dropped", [&](auto &t) { t.erase(t.begin() + static_cast<long>(offers[1])); });
    tamper("a U field changed", [&](auto &t) { expect(replace(t[closes[1]], " U 4", " U 5"), "tamper U"); });
    tamper("an L field changed", [&](auto &t) { expect(replace(t[closes[1]], " L 4 ", " L 5 "), "tamper L"); });
    tamper("the pruned close moved before the improving offer", [&](auto &t) {
        const std::string c = t[closes[0]];
        t.erase(t.begin() + static_cast<long>(closes[0]));
        t.insert(t.begin() + static_cast<long>(offers[1]), c);
    });
    tamper("a cost field changed", [&](auto &t) { expect(replace(t[offers[1]], " cost 4", " cost 5"), "tamper cost"); });
    tamper("a header knot changed", [&](auto &t) {
        for (auto &l : t)
            if (l.rfind("arc 1 2 ", 0) == 0) expect(replace(l, "0x1p+0", "0x1.8p+0"), "tamper knot");  // travel 1 -> 3/2
    });
    tamper("the fleet set to 0", [&](auto &t) { expect(replace(t[1], "fleet none", "fleet 0"), "tamper fleet"); });
    tamper("a trailing token", [&](auto &t) { t[closes[1]] += " 1"; });
    tamper("a duplicate close line", [&](auto &t) { t.push_back(t[closes[1]]); });
    std::cout << "record: " << doubles << " binary64 round trips, verifier and 14 tampered records\n";
}

}  // namespace

int main() {
    std::string prune_record;
    const std::vector<std::pair<std::string, std::function<void()>>> groups{
        {"hand", [&] { hand_gates(&prune_record); }},
        {"record", [&] { record_gates(prune_record); }},
        {"bound", bound_gates},
        {"e2e", e2e_gates}};
    for (const auto &[name, f] : groups) {
        try {
            f();
        } catch (const std::exception &e) {
            expect(false, name + " threw: " + e.what());
        }
    }
    std::cout << checks << " checks, " << failures << " failures\n";
#ifdef KAYROS_LEDGER_MUTANT_TEST
    std::cout << (failures > 0 ? "mutant caught\n" : "MUTANT SURVIVED\n");
    return failures > 0 ? 0 : 1;
#else
    return failures == 0 ? 0 : 1;
#endif
}
