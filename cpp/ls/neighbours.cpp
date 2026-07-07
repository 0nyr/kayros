#include <algorithm>
#include <limits>
#include <numeric>

#include "ls/neighbours.h"

namespace kayros {

namespace {

constexpr double kFar = std::numeric_limits<double>::infinity();

// prox(i, j) for clients i != j; kFar when the arc can never be traversed
// feasibly (TW screen) or its ATF misses the feasible departure domain.
double proximity(const Instance& inst, std::int32_t i, std::int32_t j,
                 double weight_wait) {
    const PwlfView alpha = inst.arc(i, j);
    if (alpha.n == 0) return kFar;

    double e_i = inst.horizon_start, l_i = inst.horizon_end;
    double e_j = inst.horizon_start, l_j = inst.horizon_end;
    if (inst.has_time_windows) {
        e_i = inst.tw_earliest[i];
        l_i = inst.tw_latest[i];
        e_j = inst.tw_earliest[j];
        l_j = inst.tw_latest[j];
    }
    const double s_i = inst.service_times[i];

    // Feasible departure domain from i, intersected with the ATF domain.
    const double lo = std::max(e_i + s_i, alpha.xs[0]);
    const double hi = std::min(l_i + s_i, alpha.xs[alpha.n - 1]);
    if (lo > hi) return kFar;

    // TW screen: even the earliest feasible departure arrives past l_j.
    if (evaluate(alpha, lo) > l_j) return kFar;

    // Exact min of tau(t) = alpha(t) - t over [lo, hi]: tau is piecewise
    // linear between the ATF breakpoints, so the minimum is attained at a
    // breakpoint inside the domain or at a domain endpoint. Duplicate xs
    // (vertical steps) are covered by scanning every k.
    double mindur = std::min(evaluate(alpha, lo) - lo, evaluate(alpha, hi) - hi);
    for (std::int64_t bp = 0; bp < alpha.n; ++bp) {
        const double x = alpha.xs[bp];
        if (x <= lo || x >= hi) continue;
        mindur = std::min(mindur, alpha.ys[bp] - x);
    }

    // Waiting at j that even the latest feasible departure cannot avoid.
    const double minwait = std::max(0.0, e_j - evaluate(alpha, hi));

    return mindur + weight_wait * minwait;
}

}  // namespace

NeighbourLists build_neighbour_lists(const Instance& inst,
                                     std::int32_t num_neighbours,
                                     double weight_wait) {
    NeighbourLists nb;
    if (num_neighbours <= 0) return nb;  // exhaustive sentinel

    const std::int32_t n = inst.num_customers;
    const std::int32_t nv = inst.num_vertices();
    nb.k = num_neighbours;
    nb.num_vertices = nv;

    // Symmetrised proximity over client pairs (row-major, clients 1..n).
    std::vector<double> prox(static_cast<std::size_t>(nv) * nv, kFar);
    for (std::int32_t i = 1; i <= n; ++i) {
        for (std::int32_t j = i + 1; j <= n; ++j) {
            const double p = std::min(proximity(inst, i, j, weight_wait),
                                      proximity(inst, j, i, weight_wait));
            prox[static_cast<std::size_t>(i) * nv + j] = p;
            prox[static_cast<std::size_t>(j) * nv + i] = p;
        }
    }

    // Top-k per client (finite proximities only; ties break on client id),
    // then union-symmetrise the adjacency via the bitset.
    const std::size_t words =
        (static_cast<std::size_t>(nv) * nv + 63) / 64;
    nb.bits.assign(words, 0);
    const auto set_bit = [&](std::int32_t a, std::int32_t b) {
        const std::size_t bit =
            static_cast<std::size_t>(a) * static_cast<std::size_t>(nv) +
            static_cast<std::size_t>(b);
        nb.bits[bit >> 6] |= std::uint64_t{1} << (bit & 63);
    };

    std::vector<std::int32_t> order(static_cast<std::size_t>(n));
    for (std::int32_t i = 1; i <= n; ++i) {
        std::iota(order.begin(), order.end(), 1);
        order.erase(order.begin() + (i - 1));  // exclude self
        const double* row = prox.data() + static_cast<std::size_t>(i) * nv;
        const std::size_t kk =
            std::min<std::size_t>(static_cast<std::size_t>(num_neighbours),
                                  order.size());
        std::partial_sort(order.begin(),
                          order.begin() + static_cast<std::ptrdiff_t>(kk),
                          order.end(), [&](std::int32_t a, std::int32_t b) {
                              if (row[a] != row[b]) return row[a] < row[b];
                              return a < b;
                          });
        for (std::size_t t = 0; t < kk; ++t) {
            if (row[order[t]] == kFar) break;  // infeasible edges stay out
            set_bit(i, order[t]);
            set_bit(order[t], i);
        }
        order.resize(static_cast<std::size_t>(n));  // restore for next i
    }

    // CSR lists from the symmetric bitset (ascending ids: deterministic).
    nb.offsets.assign(static_cast<std::size_t>(nv) + 1, 0);
    for (std::int32_t i = 1; i <= n; ++i) {
        std::int32_t count = 0;
        for (std::int32_t j = 1; j <= n; ++j) {
            if (nb.is_neighbour(i, j)) ++count;
        }
        nb.offsets[static_cast<std::size_t>(i) + 1] = count;
    }
    for (std::size_t i = 1; i <= static_cast<std::size_t>(nv); ++i) {
        nb.offsets[i] += nb.offsets[i - 1];
    }
    nb.flat.resize(static_cast<std::size_t>(nb.offsets.back()));
    for (std::int32_t i = 1; i <= n; ++i) {
        std::int32_t* out = nb.flat.data() + nb.offsets[static_cast<std::size_t>(i)];
        for (std::int32_t j = 1; j <= n; ++j) {
            if (nb.is_neighbour(i, j)) *out++ = j;
        }
    }
    return nb;
}

}  // namespace kayros
