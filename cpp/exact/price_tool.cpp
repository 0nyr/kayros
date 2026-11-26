// Command-line driver of the exact pricing pass, for gates and measurements on
// exported instances. Same input format, options and JSON fields as the s139
// prototype, so its gate scripts run unchanged on this build.
//
// Usage: kayros_exact_price INSTANCE.txt DUALS.txt [--no-dom] [--node-limit N] [--time-limit S]
//                           [--dump-routes FILE] [--eval-paths FILE] [--neg-cap K]
// With --eval-paths, each line of FILE is a customer sequence, and the program
// prints its exact duration computed with the leg code of the pass, or INFEASIBLE.
#include "exact/exact_pricing.h"

#include <sys/resource.h>

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace ex = kayros::exact;
using ex::Q;

namespace {

double hexd(const std::string &s) { return std::strtod(s.c_str(), nullptr); }  // exact for hex floats
Q rat(const std::string &s) { Q q(s, 10); q.canonicalize(); return q; }

ex::Instance read_instance(const char *path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open instance");
    std::string tok, a, b, c;
    int n;
    long long cap;
    in >> tok >> n >> cap;
    int hc;
    in >> a >> b >> hc >> c;
    const double lo = hexd(a), hi = hexd(b), close = hexd(c);
    in >> tok;
    std::vector<long long> dem(n + 1);
    for (auto &d : dem) in >> d;
    in >> tok;
    std::vector<ex::CustomerInput> cust(n);
    for (int v = 1; v <= n; ++v) {
        int hw;
        std::string o, cl, s;
        in >> hw >> o >> cl >> s;
        if (hw) cust[v - 1].window = std::make_pair(hexd(o), hexd(cl));
        cust[v - 1].service = hexd(s);
        cust[v - 1].demand = dem[v];
    }
    int count;
    in >> tok >> count;
    std::vector<ex::ArcInput> arcs(count);
    for (auto &e : arcs) {
        int k;
        in >> e.from >> e.to >> k;
        e.xs.resize(k), e.ys.resize(k);
        for (int t = 0; t < k; ++t) { in >> a >> b; e.xs[t] = hexd(a), e.ys[t] = hexd(b); }
    }
    return ex::Instance(cap, lo, hi, hc ? std::optional<double>(close) : std::nullopt, cust, arcs);
}

bool read_setting(std::istream &in, int n, std::string &id, ex::Duals &S) {
    std::string tok;
    if (!(in >> tok)) return false;
    if (tok != "SETTING") throw std::runtime_error("expected SETTING");
    in >> id >> tok;
    S = {};
    S.pi.assign(n + 1, Q(0));
    for (int v = 1; v <= n; ++v) { in >> tok; S.pi[v] = rat(tok); }
    int m;
    in >> tok >> m;
    S.cuts.resize(m), S.sigma.resize(m);
    for (int c = 0; c < m; ++c) {
        in >> S.cuts[c][0] >> S.cuts[c][1] >> S.cuts[c][2] >> tok;
        S.sigma[c] = rat(tok);
    }
    int f;
    in >> tok >> f;
    S.forbidden.resize(f);
    for (auto &[i, j] : S.forbidden) in >> i >> j;
    in >> tok;  // END
    return true;
}

long peak_rss_kb() { rusage u; getrusage(RUSAGE_SELF, &u); return u.ru_maxrss; }

std::string json_path(const std::vector<int> &p) {
    std::string s = "[";
    for (std::size_t i = 0; i < p.size(); ++i) s += (i ? "," : "") + std::to_string(p[i]);
    return s + "]";
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 3) { std::cerr << "usage: kayros_exact_price INSTANCE DUALS [options]\n"; return 2; }
    ex::Options opt;
    std::string dump_path, eval_path;
    for (int i = 3; i < argc; ++i) {
        std::string o = argv[i];
        if (o == "--no-dom") opt.dominance = false;
        else if (o == "--node-limit") opt.label_limit = std::atoll(argv[++i]);
        else if (o == "--time-limit") opt.time_limit = std::atof(argv[++i]);
        else if (o == "--neg-cap") opt.negative_cap = std::atoi(argv[++i]);
        else if (o == "--dump-routes") dump_path = argv[++i];
        else if (o == "--eval-paths") eval_path = argv[++i];
        else { std::cerr << "unknown option " << o << "\n"; return 2; }
    }
    auto tr = std::chrono::steady_clock::now();
    const ex::Instance I = read_instance(argv[1]);
    const double read_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - tr).count();
    if (!eval_path.empty()) {
        std::ifstream in(eval_path);
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream ss(line);
            std::vector<int> p;
            for (int x; ss >> x;) p.push_back(x);
            const auto d = ex::path_duration(I, p);
            std::cout << (d ? d->get_str() : std::string("INFEASIBLE")) << '\n';
        }
        return 0;
    }
    std::ifstream din(argv[2]);
    std::unique_ptr<std::ofstream> dump;
    if (!dump_path.empty()) {
        dump = std::make_unique<std::ofstream>(dump_path);
        opt.on_route = [&](const std::vector<int> &p, const Q &dur, const Q &rc) {
            for (int x : p) *dump << x << ' ';
            *dump << "| " << dur.get_str() << ' ' << rc.get_str() << '\n';
        };
    }
    std::string id;
    ex::Duals S;
    while (read_setting(din, I.n, id, S)) {
        const ex::Result r = ex::price(I, S, opt);
        const ex::Counters &s = r.counters;
        const bool fin = r.status == ex::Status::finished;
        std::ostringstream o;
        o << "{\"id\":\"" << id << "\",\"dominance\":" << (opt.dominance ? "true" : "false")
          << ",\"status\":\"" << (fin ? "finished" : "limit") << "\""
          << ",\"min_rc\":" << (r.min_rc ? "\"" + r.min_rc->get_str() + "\"" : std::string("null"))
          << ",\"min_rc_float\":" << (r.min_rc ? r.min_rc->get_d() : 0.0)
          << ",\"best_path\":" << json_path(r.best_route) << ",\"negative_count\":" << r.negative_count
          << ",\"labels_created\":" << s.created << ",\"labels_dominated\":" << s.dominated
          << ",\"labels_extended\":" << s.extended << ",\"labels_empty\":" << s.empty
          << ",\"completed\":" << s.completed << ",\"dom_tests\":" << s.dom_tests << ",\"dom_scans\":" << s.dom_scans << ",\"shadow_rejections\":" << s.shadow_rejections
          << ",\"max_knots\":" << s.max_knots
          << ",\"wall_s\":" << r.seconds << ",\"read_s\":" << read_s << ",\"peak_rss_kb\":" << peak_rss_kb() << ",\"negatives\":[";
        for (std::size_t k = 0; k < r.negatives.size(); ++k)
            o << (k ? "," : "") << "{\"path\":" << json_path(r.negatives[k].first) << ",\"rc\":\"" << r.negatives[k].second.get_str() << "\"}";
        o << "]}";
        std::cout << o.str() << std::endl;
    }
    return 0;
}
