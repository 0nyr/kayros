// Exact pricing pass of the Kayros exact layer (M2 design note of session 139,
// Claims 7 to 9). Forward labeling over elementary routes in GMP rationals, on
// the curves of the exact header of MAMUT-routing-lib (td_exact.hpp), with
// full dominance on whole labels. No binary64 value takes part in a decision.
//
// When the pass ends with status finished, min_rc is the exact minimum reduced
// cost over all feasible routes that respect the capacity and use no forbidden
// arc. When a limit stops it, the status is unresolved and no minimum is
// returned. The negative routes found so far are still returned with their
// exact reduced costs, since each of them is a genuine route.
#pragma once

#include <mamut_routing_lib/td_exact.hpp>

#include <array>
#include <bitset>
#include <cstddef>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace kayros::exact {

namespace td = mamut_routing::td_exact;
using td::Map;
using td::Q;

constexpr int max_customers = 255;
using Bits = std::bitset<max_customers + 1>;

struct CustomerInput {
    td::WindowInput window;  // (opens, closes) or none
    double service = 0;
    long long demand = 0;
};

// One ATF of the profile, as binary64 knots. Vertex 0 is the start depot as a
// tail and the end depot as a head.
struct ArcInput {
    int from = 0, to = 0;
    std::vector<double> xs, ys;
};

// The instance as the library checker reads it. lo and hi bound the depot
// departure, close is the depot close applied to the return.
struct Instance {
    int n = 0;
    long long cap = 0;
    Q low, high;                                    // initial curve: max(lo, horizon start), min(hi, horizon end)
    std::optional<Q> close;
    Q hend;                                         // common horizon end of the ATFs
    std::vector<long long> dem;                     // 0..n
    std::vector<std::optional<td::Customer>> cust;  // 1..n
    std::vector<bool> has_win;
    std::vector<Q> win_close, service;              // 1..n
    std::vector<std::optional<td::Arc>> arc;        // (n+1)^2, index i*(n+1)+j, j = 0 is the end depot
    std::vector<bool> reach;                        // (n+1)^2, a path from v to w exists
    std::vector<Q> dmin;                            // (n+1)^2, Claim 8 lower bound d(v, w) where reach holds

    // customers[k] is customer k+1. Throws std::invalid_argument on input the
    // profile refuses (non-FIFO knots, inverted window) and on unequal horizons.
    Instance(long long capacity, double lo, double hi, std::optional<double> depot_close,
             const std::vector<CustomerInput> &customers, const std::vector<ArcInput> &arcs);

    const td::Arc *a(int i, int j) const {
        const auto &x = arc[i * (n + 1) + j];
        return x ? &*x : nullptr;
    }
};

// Multipliers and arcs of one node. pi[0] is unused. A cut is a triple of
// distinct customers with multiplier sigma <= 0.
struct Duals {
    std::vector<Q> pi;
    std::vector<std::array<int, 3>> cuts;
    std::vector<Q> sigma;
    std::vector<std::pair<int, int>> forbidden;  // tail 0 = start depot, head 0 = end depot
};

struct Options {
    // false gives the exhaustive enumeration used as the test oracle.
    bool dominance = true;
    long long label_limit = 0;  // 0: none
    double time_limit = 0;      // seconds, 0: none
    std::size_t negative_cap = 10000;
    // Called on every completed route: customers, exact duration, exact reduced cost.
    std::function<void(const std::vector<int> &, const Q &, const Q &)> on_route;
};

struct Counters {
    long long created = 0, dominated = 0, extended = 0, completed = 0, dom_tests = 0, dom_scans = 0, empty = 0;
    long long shadow_rejections = 0;  // dominance tests rejected by the binary64 shadow
    std::size_t max_knots = 0;
};

enum class Status { finished, unresolved };

struct Result {
    Status status = Status::finished;
    std::optional<Q> min_rc;  // set only when finished and some route exists
    std::vector<int> best_route;
    long long negative_count = 0;
    std::vector<std::pair<std::vector<int>, Q>> negatives;  // the first negative_cap ones
    Counters counters;
    double seconds = 0;
};

Result price(const Instance &instance, const Duals &duals, const Options &options = {});

// Building blocks, exposed for the tests.

// Initial curve and one leg, in the order of td::arrival_curve. w = 0 is the return to the depot.
Map initial_curve(const Instance &I);
bool extend_curve(const Instance &I, Map &f, int v, int w);
Q min_duration(const Map &f);
// Exact duration of the route 0, customers..., 0 with the leg code, or none when infeasible.
std::optional<Q> path_duration(const Instance &I, const std::vector<int> &customers);

// Condition (c) of Claim 7 as decided in the note: y = x + delta <= last x of g and
// g(max(y, first x of g)) <= f(x) for every x of the domain of f. f is the curve
// of the dominated label L, g that of the dominator M. scans counts merge scans.
bool curve_dominated(const Map &f, const Map &g, const Q &delta, long long *scans = nullptr);

// The binary64 shadow test on the same arguments. It may only reject: when it
// returns true, curve_dominated(f, g, delta) returns false.
bool shadow_rejects(const Map &f, const Map &g, const Q &delta);

}  // namespace kayros::exact
