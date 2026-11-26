// Ledger of the exact layer. See ledger.h for the contract and the Claims.
#include "exact/ledger.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <istream>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace kayros::exact {

namespace {

Q canonical(Q q) {
    q.canonicalize();
    return q;
}

}  // namespace

Ledger::Ledger(InstanceData data, std::optional<int> fleet)
    : data_(std::move(data)), fleet_(fleet),
      instance_(data_.capacity, data_.lo, data_.hi, data_.depot_close, data_.customers, data_.arcs) {
    if (fleet_ && *fleet_ < 1) throw std::invalid_argument("fleet limit below 1");
    // The pass checks the capacity on every prefix of a route, offer on the whole
    // route. Both agree when demands are nonnegative. The bounds keep every load
    // sum of the pass and of offer below 2^63.
    if (data_.capacity < 0 || data_.capacity > (1LL << 62)) throw std::invalid_argument("capacity outside 0..2^62");
    for (const CustomerInput &c : data_.customers)
        if (c.demand < 0 || c.demand > data_.capacity) throw std::invalid_argument("demand outside 0..capacity");
    open_[0] = Node{};
}

// Claim 2. Left forbids (i, j). Right forbids every other arc out of i and every
// other arc into j, k ranging over all vertices. Tail 0 is the start depot and
// head 0 the end depot, so right forbids (i, 0) and (0, j). Self loops are not
// arcs of the pass and are skipped.
std::vector<std::pair<int, int>> Ledger::forbidden_arcs(const std::vector<Decision> &decisions) const {
    const int n = instance_.n;
    std::vector<std::pair<int, int>> out;
    for (const Decision &d : decisions) {
        if (!d.right) {
            out.push_back({d.i, d.j});
            continue;
        }
        for (int k = 0; k <= n; ++k) {
            if (k != d.j && k != d.i) out.push_back({d.i, k});
            if (k != d.i && k != d.j) out.push_back({k, d.j});
        }
    }
    return out;
}

std::optional<std::vector<std::pair<int, int>>> Ledger::forbidden(int node) const {
    auto it = open_.find(node);
    if (it == open_.end()) return std::nullopt;
    return forbidden_arcs(it->second.decisions);
}

// Accepted iff the routes form a solution and (no incumbent or cost < U).
// path_duration checks that every vertex is in 0..n, that every leg is an arc of
// the instance, and the timing (horizon, departure interval, windows, depot
// close). It does not check the capacity, repeated customers, an empty route or
// a vertex 0 inside a route (which it would read as a return to the depot), so
// these are checked here.
OfferOutcome Ledger::offer(const std::vector<std::vector<int>> &routes) {
    const int n = instance_.n;
    OfferOutcome o;
    if (fleet_ && routes.size() > static_cast<std::size_t>(*fleet_)) return o.reason = "more routes than the fleet limit", o;
    std::vector<int> visits(n + 1, 0);
    Q cost = 0;
    for (const std::vector<int> &r : routes) {
        if (r.empty()) return o.reason = "empty route", o;
        long long load = 0;
        for (int v : r) {
            if (v < 1 || v > n) return o.reason = "route vertex not a customer", o;
            ++visits[v];
            if (instance_.dem[v] > instance_.cap - load) return o.reason = "route over capacity", o;
            load += instance_.dem[v];
        }
        const std::optional<Q> d = path_duration(instance_, r);
        if (!d) return o.reason = "infeasible route", o;
        cost += *d;
    }
    for (int v = 1; v <= n; ++v)
        if (visits[v] != 1) return o.reason = "customer " + std::to_string(v) + " not visited exactly once", o;
    cost.canonicalize();
    o.cost = cost;
    if (U_ && !(cost < *U_)) return o.reason = "not strictly below the incumbent", o;
    U_ = cost;
    incumbent_ = routes;
    Operation op;
    op.kind = Operation::Kind::offer;
    op.routes = routes;
    op.cost = cost;
    record_.push_back(op);
    o.accepted = true;
    return o;
}

BranchOutcome Ledger::branch(int node, int i, int j) {
    const int n = instance_.n;
    BranchOutcome b;
    auto it = open_.find(node);
    if (it == open_.end()) return b.reason = "node not open", b;
    if (i < 1 || i > n || j < 1 || j > n || i == j) return b.reason = "arc not between 2 distinct customers", b;
    // Branching again on an arc already decided on the path is accepted. Claim 2
    // holds for any node and any arc, so the 2 children still cover the node.
    Node left{it->second.decisions, std::nullopt}, right{it->second.decisions, std::nullopt};
    left.decisions.push_back({i, j, false});
    right.decisions.push_back({i, j, true});
    b.left = next_id_, b.right = next_id_ + 1;
    open_.erase(it);
    open_[b.left] = std::move(left);
    open_[b.right] = std::move(right);
    next_id_ += 2;
    Operation op;
    op.kind = Operation::Kind::branch;
    op.node = node, op.i = i, op.j = j, op.left = b.left, op.right = b.right;
    record_.push_back(op);
    b.accepted = true;
    return b;
}

void Ledger::remove(int node, const Certificate &c) {
    open_.erase(node);
    Operation op;
    op.kind = Operation::Kind::close;
    op.node = node;
    op.certificate = c;
    op.U = *U_;
    record_.push_back(op);
}

bool Ledger::reaches(const Q &L) const { return L >= *U_; }

// Claim 1 with the Delta of Claim 9. The checks below make every input valid for
// price, which therefore does not throw on them.
CloseOutcome Ledger::close(int node, const std::vector<std::array<int, 3>> &cuts, const std::vector<Q> &pi,
                           const std::vector<Q> &sigma, const Limits &limits) {
    const int n = instance_.n;
    CloseOutcome c;
    auto it = open_.find(node);
    if (it == open_.end()) return c.reason = "node not open", c;
    if (!U_) return c.reason = "no incumbent", c;
    if (pi.size() != static_cast<std::size_t>(n + 1)) return c.reason = "pi must have n+1 entries", c;
    if (sigma.size() != cuts.size()) return c.reason = "one sigma per cut", c;
    for (const auto &S : cuts) {
        for (int v : S)
            if (v < 1 || v > n) return c.reason = "cut vertex not a customer", c;
        if (S[0] == S[1] || S[0] == S[2] || S[1] == S[2]) return c.reason = "cut customers not distinct", c;
    }
    // GMP compares canonical rationals only, so the checks read canonical copies.
    Certificate cert;
    cert.cuts = cuts;
    for (const Q &p : pi) cert.pi.push_back(canonical(p));
    for (const Q &s : sigma) cert.sigma.push_back(canonical(s));
    if (sgn(cert.pi[0]) != 0) return c.reason = "pi[0] must be 0", c;  // L sums pi over the customers only
    for (const Q &s : cert.sigma)
        if (s > 0) return c.reason = "positive cut multiplier", c;
    Duals duals{cert.pi, cuts, cert.sigma, forbidden_arcs(it->second.decisions)};
    Options options;
    options.label_limit = limits.label_limit;
    options.time_limit = limits.time_limit;
    options.memory_limit = limits.memory_limit;
    Result res = price(instance_, duals, options);
    c.negative_count = res.negative_count;
    c.negatives = std::move(res.negatives);
    if (res.status != Status::finished) return c.reason = "exact pass unresolved", c;
    if (!res.min_rc) {  // R_nu is empty, so no solution lies in the node
        remove(node, cert);
        c.removed = true;
        return c.reason = "no feasible route at the node", c;
    }
    const Q delta = *res.min_rc < 0 ? Q(-*res.min_rc) : Q(0);
    const Q kappa(fleet_ ? std::min(n, *fleet_) : n);
    Q L = 0;
    for (int v = 1; v <= n; ++v) L += cert.pi[v];
    for (const Q &s : cert.sigma) L += s;
    L -= kappa * delta;
    L.canonicalize();
    cert.delta = canonical(delta);
    cert.bound = L;
    c.delta = cert.delta, c.bound = L;
    if (reaches(L)) {
        remove(node, cert);
        c.removed = true;
        return c.reason = "bound at least U", c;
    }
    // The node stays open. Keep the largest bound for a later prune, record nothing.
    Node &nd = it->second;
    if (!nd.best || L > *nd.best->bound) nd.best = cert;
    return c.reason = "bound below U", c;
}

PruneOutcome Ledger::prune(int node) {
    PruneOutcome p;
    auto it = open_.find(node);
    if (it == open_.end()) return p.reason = "node not open", p;
    if (!it->second.best) return p.reason = "no stored bound", p;
    if (!U_) return p.reason = "no incumbent", p;  // never true, a stored bound needs an incumbent
    if (!reaches(*it->second.best->bound)) return p.reason = "stored bound below U", p;
    const Certificate cert = *it->second.best;
    remove(node, cert);
    p.removed = true;
    return p.reason = "stored bound at least U", p;
}

// ---------- record ----------

std::string hex_double(double x) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%a", x);
    return buf;
}

double parse_double(const std::string &s) {
    char *end = nullptr;
    const double x = std::strtod(s.c_str(), &end);
    if (s.empty() || end != s.c_str() + s.size() || !std::isfinite(x) || hex_double(x) != s)
        throw std::runtime_error("bad binary64 token '" + s + "'");
    return x;
}

Q parse_rational(const std::string &s) {
    Q q;
    if (s.empty() || q.set_str(s, 10) != 0 || sgn(q.get_den()) == 0) throw std::runtime_error("bad rational token '" + s + "'");
    q.canonicalize();
    if (q.get_str() != s) throw std::runtime_error("non-canonical rational token '" + s + "'");
    return q;
}

namespace {

std::string opt_q(const std::optional<Q> &q) { return q ? q->get_str() : "none"; }

void write_operation(std::ostream &out, const Operation &op) {
    switch (op.kind) {
    case Operation::Kind::offer:
        out << "offer routes " << op.routes.size();
        for (const auto &r : op.routes) {
            out << ' ' << r.size();
            for (int v : r) out << ' ' << v;
        }
        out << " cost " << op.cost.get_str();
        break;
    case Operation::Kind::branch:
        out << "branch node " << op.node << " arc " << op.i << ' ' << op.j << " children " << op.left << ' ' << op.right;
        break;
    case Operation::Kind::close: {
        const Certificate &c = op.certificate;
        out << "close node " << op.node << " pi " << c.pi.size() - 1;
        for (std::size_t v = 1; v < c.pi.size(); ++v) out << ' ' << c.pi[v].get_str();
        out << " cuts " << c.cuts.size();
        for (std::size_t k = 0; k < c.cuts.size(); ++k)
            out << ' ' << c.cuts[k][0] << ' ' << c.cuts[k][1] << ' ' << c.cuts[k][2] << ' ' << c.sigma[k].get_str();
        out << " delta " << opt_q(c.delta) << " L " << opt_q(c.bound) << " U " << op.U.get_str();
        break;
    }
    }
    out << '\n';
}

// Tokens of 1 line, every read checked.
struct Tokens {
    std::istringstream in;
    explicit Tokens(const std::string &line) : in(line) {}
    std::string word() {
        std::string w;
        if (!(in >> w)) throw std::runtime_error("line ends too early");
        return w;
    }
    void expect(const std::string &w) {
        if (word() != w) throw std::runtime_error("expected '" + w + "'");
    }
    long long integer() {
        const std::string w = word();
        std::size_t used = 0;
        long long x = 0;
        try { x = std::stoll(w, &used); } catch (const std::exception &) { used = 0; }
        if (used != w.size() || std::to_string(x) != w) throw std::runtime_error("bad integer token '" + w + "'");
        return x;
    }
    int small(long long lo, long long hi) {
        const long long x = integer();
        if (x < lo || x > hi) throw std::runtime_error("integer out of range");
        return static_cast<int>(x);
    }
    Q rational() { return parse_rational(word()); }
    std::optional<Q> opt_rational() {
        const std::string w = word();
        if (w == "none") return std::nullopt;
        return parse_rational(w);
    }
    double binary64() { return parse_double(word()); }
    void end() {
        std::string w;
        if (in >> w) throw std::runtime_error("unexpected token '" + w + "'");
    }
};

Tokens next_line(std::istream &in) {
    std::string line;
    if (!std::getline(in, line)) throw std::runtime_error("record ends inside the header");
    return Tokens(line);
}

constexpr int big = 1 << 30;

Operation read_operation(const std::string &line) {
    Tokens t(line);
    Operation op;
    const std::string kind = t.word();
    if (kind == "offer") {
        op.kind = Operation::Kind::offer;
        t.expect("routes");
        const int m = t.small(0, big);
        for (int r = 0; r < m; ++r) {
            std::vector<int> route(t.small(0, big));
            for (int &v : route) v = t.small(-big, big);
            op.routes.push_back(route);
        }
        t.expect("cost");
        op.cost = t.rational();
    } else if (kind == "branch") {
        op.kind = Operation::Kind::branch;
        t.expect("node");
        op.node = t.small(-big, big);
        t.expect("arc");
        op.i = t.small(-big, big), op.j = t.small(-big, big);
        t.expect("children");
        op.left = t.small(-big, big), op.right = t.small(-big, big);
    } else if (kind == "close") {
        op.kind = Operation::Kind::close;
        Certificate &c = op.certificate;
        t.expect("node");
        op.node = t.small(-big, big);
        t.expect("pi");
        const int n = t.small(0, big);
        c.pi.push_back(Q(0));
        for (int v = 0; v < n; ++v) c.pi.push_back(t.rational());
        t.expect("cuts");
        const int m = t.small(0, big);
        for (int k = 0; k < m; ++k) {
            std::array<int, 3> S{t.small(-big, big), t.small(-big, big), t.small(-big, big)};
            c.cuts.push_back(S);
            c.sigma.push_back(t.rational());
        }
        t.expect("delta");
        c.delta = t.opt_rational();
        t.expect("L");
        c.bound = t.opt_rational();
        t.expect("U");
        op.U = t.rational();
    } else {
        throw std::runtime_error("unknown operation '" + kind + "'");
    }
    t.end();
    return op;
}

// Applies 1 stored operation. Returns an empty string when the ledger accepts it
// and recomputes every informative field, else the error.
std::string apply(Ledger &ledger, const Operation &op) {
    switch (op.kind) {
    case Operation::Kind::offer: {
        const OfferOutcome o = ledger.offer(op.routes);
        if (!o.accepted) return "offer refused: " + o.reason;
        if (*o.cost != op.cost) return "offer cost differs from the record";
        return "";
    }
    case Operation::Kind::branch: {
        const BranchOutcome b = ledger.branch(op.node, op.i, op.j);
        if (!b.accepted) return "branch refused: " + b.reason;
        if (b.left != op.left || b.right != op.right) return "children ids differ from the record";
        return "";
    }
    case Operation::Kind::close: {
        if (!ledger.U() || *ledger.U() != op.U) return "U differs from the record";
        const Certificate &c = op.certificate;
        const CloseOutcome r = ledger.close(op.node, c.cuts, c.pi, c.sigma);
        if (!r.removed) return "close did not remove the node: " + r.reason;
        if (r.delta != c.delta || r.bound != c.bound) return "delta or L differs from the record";
        return "";
    }
    }
    return "unknown operation";
}

}  // namespace

void Ledger::write(std::ostream &out) const {
    const InstanceData &d = data_;
    out << "kayros-ledger-record 1\n";
    out << "fleet " << (fleet_ ? std::to_string(*fleet_) : "none") << '\n';
    out << "capacity " << d.capacity << '\n';
    out << "departure " << hex_double(d.lo) << ' ' << hex_double(d.hi) << '\n';
    out << "close " << (d.depot_close ? hex_double(*d.depot_close) : "none") << '\n';
    out << "customers " << d.customers.size() << '\n';
    for (const CustomerInput &c : d.customers) {
        out << "customer window ";
        if (c.window) out << hex_double(c.window->first) << ' ' << hex_double(c.window->second);
        else out << "none";
        out << " service " << hex_double(c.service) << " demand " << c.demand << '\n';
    }
    out << "arcs " << d.arcs.size() << '\n';
    for (const ArcInput &a : d.arcs) {
        out << "arc " << a.from << ' ' << a.to << " knots " << a.xs.size();
        for (std::size_t k = 0; k < a.xs.size(); ++k) out << ' ' << hex_double(a.xs[k]) << ' ' << hex_double(a.ys[k]);
        out << '\n';
    }
    out << "operations\n";
    for (const Operation &op : record_) write_operation(out, op);
}

Replay replay(std::istream &in) {
    Replay out;
    try {
        InstanceData d;
        std::optional<int> fleet;
        {
            Tokens t = next_line(in);
            t.expect("kayros-ledger-record"), t.expect("1"), t.end();
        }
        {
            Tokens t = next_line(in);
            t.expect("fleet");
            const std::string w = t.word();
            if (w != "none") fleet = Tokens(w).small(-big, big);
            t.end();
        }
        {
            Tokens t = next_line(in);
            t.expect("capacity"), d.capacity = t.integer(), t.end();
        }
        {
            Tokens t = next_line(in);
            t.expect("departure"), d.lo = t.binary64(), d.hi = t.binary64(), t.end();
        }
        {
            Tokens t = next_line(in);
            t.expect("close");
            const std::string w = t.word();
            if (w != "none") d.depot_close = parse_double(w);
            t.end();
        }
        int count = 0;
        {
            Tokens t = next_line(in);
            t.expect("customers"), count = t.small(0, big), t.end();
        }
        for (int k = 0; k < count; ++k) {
            Tokens t = next_line(in);
            CustomerInput c;
            t.expect("customer"), t.expect("window");
            const std::string w = t.word();
            if (w != "none") c.window = std::make_pair(parse_double(w), t.binary64());
            t.expect("service"), c.service = t.binary64();
            t.expect("demand"), c.demand = t.integer();
            t.end();
            d.customers.push_back(c);
        }
        {
            Tokens t = next_line(in);
            t.expect("arcs"), count = t.small(0, big), t.end();
        }
        for (int k = 0; k < count; ++k) {
            Tokens t = next_line(in);
            ArcInput a;
            t.expect("arc"), a.from = t.small(-big, big), a.to = t.small(-big, big);
            t.expect("knots");
            const int m = t.small(0, big);
            for (int q = 0; q < m; ++q) a.xs.push_back(t.binary64()), a.ys.push_back(t.binary64());
            t.end();
            d.arcs.push_back(a);
        }
        {
            Tokens t = next_line(in);
            t.expect("operations"), t.end();
        }
        Ledger ledger(d, fleet);
        std::string line;
        while (std::getline(in, line)) {
            ++out.operations;
            const std::string error = apply(ledger, read_operation(line));
            if (!error.empty()) {
                out.message = "operation " + std::to_string(out.operations) + ": " + error;
                return out;
            }
        }
        out.U = ledger.U();
        out.incumbent = ledger.incumbent();
        out.certified = ledger.certified();
        if (!out.certified)
            out.message = ledger.U() ? std::to_string(ledger.open_count()) + " nodes still open" : "no incumbent";
    } catch (const std::exception &e) {
        out.certified = false;
        out.message = std::string("malformed record: ") + e.what();
    }
    return out;
}

}  // namespace kayros::exact
