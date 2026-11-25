#include <stdexcept>

#include "ls/ls.h"

namespace kayros {

namespace {

RouteEval finish(PwlfView acc) {
    if (acc.n == 0) return {false, 0.0, 0.0};
    const MinShift s = min_shifted_image(acc);
    return {true, s.value, s.argmin_x};
}

// A candidate starts with a view of the prefix, and each composition writes to the alternate accumulator. Leaf and query buffers are independent of both accumulators.
struct SpliceWork {
    Pwlf a, b, leaf, query;
    PwlfView cur;
    int in = -1;  // buffer holding cur: 0 = a, 1 = b, -1 = neither
    double dep[2];

    // cur <- f ∘ cur. Returns false when the result is empty.
    bool step(PwlfView f) {
        if (f.n == 0) return false;
        Pwlf& target = in == 0 ? b : a;
        compose_into(f, cur, target);
        in = in == 0 ? 1 : 0;
        cur = view(target);
        return cur.n != 0;
    }
};

thread_local SpliceWork splice_work;

}  // namespace

RouteEval evaluate_splice(const Instance& inst, const RouteState& r1,
                          std::int64_t i1, std::int64_t j1,
                          const RouteState& r2, std::int64_t i2,
                          std::int64_t j2) {
    const std::vector<std::int32_t>& route1 = r1.vertices;
    const std::vector<std::int32_t>& route2 = r2.vertices;
    const std::int64_t m1 = static_cast<std::int64_t>(route1.size());
    const bool incoming = i2 <= j2;
    if (i1 < 0 || i1 > m1 || j1 < i1 - 1 || j1 >= m1) {
        throw std::invalid_argument("invalid [i1, j1]");
    }
    if (incoming && (i2 < 0 || j2 >= static_cast<std::int64_t>(route2.size()))) {
        throw std::invalid_argument("invalid [i2, j2]");
    }
    const bool head = i1 > 0;
    const bool tail = j1 + 1 < m1;
    if (!head && !incoming && !tail) return {false, 0.0, 0.0};
    SpliceWork& w = splice_work;
    w.in = -1;
    w.cur = head ? r1.tree.query_view(0, i1 - 1, w.query) : inst.departure_window(w.dep);
    if (w.cur.n == 0) return {false, 0.0, 0.0};

    std::int32_t last = head ? route1[static_cast<std::size_t>(i1 - 1)] : 0;
    if (incoming) {
        if (!w.step(inst.move_leaf(last, route2[static_cast<std::size_t>(i2)], w.leaf)))
            return {false, 0.0, 0.0};
        if (j2 > i2 && !w.step(r2.tree.query_view(i2 + 1, j2, w.query))) return {false, 0.0, 0.0};
        last = route2[static_cast<std::size_t>(j2)];
    }
    if (tail) {
        if (!w.step(inst.move_leaf(last, route1[static_cast<std::size_t>(j1 + 1)], w.leaf)))
            return {false, 0.0, 0.0};
        if (j1 + 2 <= m1 && !w.step(r1.tree.query_view(j1 + 2, m1, w.query)))
            return {false, 0.0, 0.0};
    } else {
        if (!w.step(inst.move_leaf(last, 0, w.leaf))) return {false, 0.0, 0.0};
    }
    return finish(w.cur);
}

RouteEval evaluate_intra_relocate(const Instance& inst, const RouteState& r,
                                  std::int64_t i, std::int64_t p) {
    const std::vector<std::int32_t>& route = r.vertices;
    const std::int64_t m = static_cast<std::int64_t>(route.size());
    if (m < 2 || i < 0 || i >= m || p < 0 || p > m || p == i || p == i + 1) {
        throw std::invalid_argument("invalid intra relocate");
    }
    const std::int32_t c = route[static_cast<std::size_t>(i)];
    SpliceWork& w = splice_work;
    w.in = -1;
    const auto at = [&route](std::int64_t k) { return route[static_cast<std::size_t>(k)]; };
    if (p < i) {
        // New order: r[0..p-1], c, r[p..i-1], r[i+1..m-1].
        w.cur = p > 0 ? r.tree.query_view(0, p - 1, w.query) : inst.departure_window(w.dep);
        if (w.cur.n == 0) return {false, 0.0, 0.0};
        const std::int32_t before = p > 0 ? at(p - 1) : 0;
        if (!w.step(inst.move_leaf(before, c, w.leaf))) return {false, 0.0, 0.0};
        if (!w.step(inst.move_leaf(c, at(p), w.leaf))) return {false, 0.0, 0.0};
        if (p + 1 <= i - 1 && !w.step(r.tree.query_view(p + 1, i - 1, w.query)))
            return {false, 0.0, 0.0};
        if (i + 1 <= m - 1) {
            if (!w.step(inst.move_leaf(at(i - 1), at(i + 1), w.leaf)))
                return {false, 0.0, 0.0};
            if (i + 2 <= m && !w.step(r.tree.query_view(i + 2, m, w.query)))
                return {false, 0.0, 0.0};
        } else {
            if (!w.step(inst.move_leaf(at(i - 1), 0, w.leaf))) return {false, 0.0, 0.0};
        }
    } else {
        // p > i + 1. New order: r[0..i-1], r[i+1..p-1], c, r[p..m-1].
        w.cur = i > 0 ? r.tree.query_view(0, i - 1, w.query) : inst.departure_window(w.dep);
        if (w.cur.n == 0) return {false, 0.0, 0.0};
        const std::int32_t before = i > 0 ? at(i - 1) : 0;
        if (!w.step(inst.move_leaf(before, at(i + 1), w.leaf))) return {false, 0.0, 0.0};
        if (i + 2 <= p - 1 && !w.step(r.tree.query_view(i + 2, p - 1, w.query)))
            return {false, 0.0, 0.0};
        if (!w.step(inst.move_leaf(at(p - 1), c, w.leaf))) return {false, 0.0, 0.0};
        if (p <= m - 1) {
            if (!w.step(inst.move_leaf(c, at(p), w.leaf))) return {false, 0.0, 0.0};
            if (p + 1 <= m && !w.step(r.tree.query_view(p + 1, m, w.query)))
                return {false, 0.0, 0.0};
        } else {
            if (!w.step(inst.move_leaf(c, 0, w.leaf))) return {false, 0.0, 0.0};
        }
    }
    return finish(w.cur);
}

bool price_insertion(const Instance& inst, const RouteState& recv, PwlfView prefix,
                     std::int32_t before, std::int32_t c, std::int64_t p,
                     std::int64_t mb, MinShift& out) {
    SpliceWork& w = splice_work;
    w.in = -1;
    w.cur = prefix;
    if (!w.step(inst.move_leaf(before, c, w.leaf))) return false;
    if (p < mb) {
        const std::int32_t succ = recv.vertices[static_cast<std::size_t>(p)];
        if (!w.step(inst.move_leaf(c, succ, w.leaf))) return false;
        if (!w.step(recv.tree.query_view(p + 1, mb, w.query))) return false;
    } else {
        if (!w.step(inst.move_leaf(c, 0, w.leaf))) return false;
    }
    out = min_shifted_image(w.cur);
    return true;
}

}  // namespace kayros
