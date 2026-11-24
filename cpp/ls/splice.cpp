#include <algorithm>
#include <stdexcept>

#include "ls/ls.h"

namespace kayros {

namespace {

RouteEval finish(const Pwlf& acc) {
    if (acc.xs.empty()) return {false, 0.0, 0.0};
    const MinShift s = min_shifted_image(view(acc));
    return {true, s.value, s.argmin_x};
}

RouteEval finish(PwlfView acc) {
    if (acc.n == 0) return {false, 0.0, 0.0};
    const MinShift s = min_shifted_image(acc);
    return {true, s.value, s.argmin_x};
}

// Fold study no-alloc arms: reusable buffers of the ranking evaluators. The
// accumulator alternates between a and b, the other buffers hold the leaf or
// the tree query of the current step.
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

RouteEval evaluate_splice_no_alloc(const Instance& inst, const RouteState& r1,
                                   std::int64_t i1, std::int64_t j1,
                                   const RouteState& r2, std::int64_t i2,
                                   std::int64_t j2) {
    const std::vector<std::int32_t>& route1 = r1.vertices;
    const std::vector<std::int32_t>& route2 = r2.vertices;
    const std::int64_t m1 = static_cast<std::int64_t>(route1.size());
    const bool incoming = i2 <= j2;
    const bool head = i1 > 0;
    const bool tail = j1 + 1 < m1;
    if (!head && !incoming && !tail) return {false, 0.0, 0.0};
    SpliceWork& w = splice_work;
    w.in = -1;
    w.cur = head ? r1.tree.query_view(0, i1 - 1, w.query) : departure_identity_view(inst, w.dep);
    if (w.cur.n == 0) return {false, 0.0, 0.0};

    std::int32_t last = head ? route1[static_cast<std::size_t>(i1 - 1)] : 0;
    if (incoming) {
        if (!w.step(bridge_leaf_view(inst, last, route2[static_cast<std::size_t>(i2)], w.leaf)))
            return {false, 0.0, 0.0};
        if (j2 > i2 && !w.step(r2.tree.query_view(i2 + 1, j2, w.query))) return {false, 0.0, 0.0};
        last = route2[static_cast<std::size_t>(j2)];
    }
    if (tail) {
        if (!w.step(bridge_leaf_view(inst, last, route1[static_cast<std::size_t>(j1 + 1)], w.leaf)))
            return {false, 0.0, 0.0};
        if (j1 + 2 <= m1 && !w.step(r1.tree.query_view(j1 + 2, m1, w.query)))
            return {false, 0.0, 0.0};
    } else {
        if (!w.step(return_leaf_view(inst, last, w.leaf))) return {false, 0.0, 0.0};
    }
    return finish(w.cur);
}

RouteEval evaluate_intra_relocate_no_alloc(const Instance& inst, const RouteState& r,
                                           std::int64_t i, std::int64_t p) {
    const std::vector<std::int32_t>& route = r.vertices;
    const std::int64_t m = static_cast<std::int64_t>(route.size());
    const std::int32_t c = route[static_cast<std::size_t>(i)];
    SpliceWork& w = splice_work;
    w.in = -1;
    const auto at = [&route](std::int64_t k) { return route[static_cast<std::size_t>(k)]; };
    if (p < i) {
        // New order: r[0..p-1], c, r[p..i-1], r[i+1..m-1].
        w.cur = p > 0 ? r.tree.query_view(0, p - 1, w.query) : departure_identity_view(inst, w.dep);
        if (w.cur.n == 0) return {false, 0.0, 0.0};
        const std::int32_t before = p > 0 ? at(p - 1) : 0;
        if (!w.step(bridge_leaf_view(inst, before, c, w.leaf))) return {false, 0.0, 0.0};
        if (!w.step(bridge_leaf_view(inst, c, at(p), w.leaf))) return {false, 0.0, 0.0};
        if (p + 1 <= i - 1 && !w.step(r.tree.query_view(p + 1, i - 1, w.query)))
            return {false, 0.0, 0.0};
        if (i + 1 <= m - 1) {
            if (!w.step(bridge_leaf_view(inst, at(i - 1), at(i + 1), w.leaf)))
                return {false, 0.0, 0.0};
            if (i + 2 <= m && !w.step(r.tree.query_view(i + 2, m, w.query)))
                return {false, 0.0, 0.0};
        } else {
            if (!w.step(return_leaf_view(inst, at(i - 1), w.leaf))) return {false, 0.0, 0.0};
        }
    } else {
        // p > i + 1. New order: r[0..i-1], r[i+1..p-1], c, r[p..m-1].
        w.cur = i > 0 ? r.tree.query_view(0, i - 1, w.query) : departure_identity_view(inst, w.dep);
        if (w.cur.n == 0) return {false, 0.0, 0.0};
        const std::int32_t before = i > 0 ? at(i - 1) : 0;
        if (!w.step(bridge_leaf_view(inst, before, at(i + 1), w.leaf))) return {false, 0.0, 0.0};
        if (i + 2 <= p - 1 && !w.step(r.tree.query_view(i + 2, p - 1, w.query)))
            return {false, 0.0, 0.0};
        if (!w.step(bridge_leaf_view(inst, at(p - 1), c, w.leaf))) return {false, 0.0, 0.0};
        if (p <= m - 1) {
            if (!w.step(bridge_leaf_view(inst, c, at(p), w.leaf))) return {false, 0.0, 0.0};
            if (p + 1 <= m && !w.step(r.tree.query_view(p + 1, m, w.query)))
                return {false, 0.0, 0.0};
        } else {
            if (!w.step(return_leaf_view(inst, c, w.leaf))) return {false, 0.0, 0.0};
        }
    }
    return finish(w.cur);
}

}  // namespace

bool price_insertion_no_alloc(const Instance& inst, const RouteState& recv, PwlfView prefix,
                              std::int32_t before, std::int32_t c, std::int64_t p,
                              std::int64_t mb, MinShift& out) {
    SpliceWork& w = splice_work;
    w.in = -1;
    w.cur = prefix;
    if (!w.step(bridge_leaf_view(inst, before, c, w.leaf))) return false;
    if (p < mb) {
        const std::int32_t succ = recv.vertices[static_cast<std::size_t>(p)];
        if (!w.step(bridge_leaf_view(inst, c, succ, w.leaf))) return false;
        if (!w.step(recv.tree.query_view(p + 1, mb, w.query))) return false;
    } else {
        if (!w.step(return_leaf_view(inst, c, w.leaf))) return false;
    }
    out = min_shifted_image(w.cur);
    return true;
}

RouteEval evaluate_splice(const Instance& inst, const RouteState& r1,
                          std::int64_t i1, std::int64_t j1,
                          const RouteState& r2, std::int64_t i2,
                          std::int64_t j2) {
    const std::vector<std::int32_t>& route1 = r1.vertices;
    const std::vector<std::int32_t>& route2 = r2.vertices;
    const std::int64_t m1 = static_cast<std::int64_t>(route1.size());
    const std::int64_t m2 = static_cast<std::int64_t>(route2.size());
    const bool incoming = i2 <= j2;
    if (i1 < 0 || i1 > m1 || j1 < i1 - 1 || j1 >= m1) {
        throw std::invalid_argument("invalid [i1, j1]");
    }
    if (incoming && (i2 < 0 || j2 >= m2)) {
        throw std::invalid_argument("invalid [i2, j2]");
    }
    if (inst.arm_no_alloc()) return evaluate_splice_no_alloc(inst, r1, i1, j1, r2, i2, j2);
    const bool head = i1 > 0;
    const bool tail = j1 + 1 < m1;
    if (!head && !incoming && !tail) return {false, 0.0, 0.0};

    Pwlf acc = head ? r1.tree.query(0, i1 - 1) : departure_identity(inst);
    if (acc.xs.empty()) return {false, 0.0, 0.0};

    std::int32_t last = head ? route1[static_cast<std::size_t>(i1 - 1)] : 0;

    if (incoming) {
        const Pwlf bridge =
            bridge_leaf(inst, last, route2[static_cast<std::size_t>(i2)]);
        if (bridge.xs.empty()) return {false, 0.0, 0.0};
        acc = compose(view(bridge), view(acc));
        if (acc.xs.empty()) return {false, 0.0, 0.0};
        if (j2 > i2) {
            const Pwlf middle = r2.tree.query(i2 + 1, j2);
            if (middle.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(middle), view(acc));
            if (acc.xs.empty()) return {false, 0.0, 0.0};
        }
        last = route2[static_cast<std::size_t>(j2)];
    }

    if (tail) {
        const Pwlf bridge =
            bridge_leaf(inst, last, route1[static_cast<std::size_t>(j1 + 1)]);
        if (bridge.xs.empty()) return {false, 0.0, 0.0};
        acc = compose(view(bridge), view(acc));
        if (acc.xs.empty()) return {false, 0.0, 0.0};
        if (j1 + 2 <= m1) {
            const Pwlf suffix = r1.tree.query(j1 + 2, m1);
            if (suffix.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(suffix), view(acc));
        }
    } else {
        const Pwlf ret = return_leaf(inst, last);
        if (ret.xs.empty()) return {false, 0.0, 0.0};
        acc = compose(view(ret), view(acc));
    }
    return finish(acc);
}

RouteEval evaluate_intra_relocate(const Instance& inst, const RouteState& r,
                                  std::int64_t i, std::int64_t p) {
    const std::vector<std::int32_t>& route = r.vertices;
    const std::int64_t m = static_cast<std::int64_t>(route.size());
    if (m < 2 || i < 0 || i >= m || p < 0 || p > m || p == i || p == i + 1) {
        throw std::invalid_argument("invalid intra relocate");
    }
    if (inst.arm_no_alloc()) return evaluate_intra_relocate_no_alloc(inst, r, i, p);
    const std::int32_t c = route[static_cast<std::size_t>(i)];
    Pwlf acc;

    if (p < i) {
        // New order: r[0..p-1], c, r[p..i-1], r[i+1..m-1].
        acc = p > 0 ? r.tree.query(0, p - 1) : departure_identity(inst);
        if (acc.xs.empty()) return {false, 0.0, 0.0};
        const std::int32_t before = p > 0 ? route[static_cast<std::size_t>(p - 1)] : 0;
        for (const Pwlf& step :
             {bridge_leaf(inst, before, c),
              bridge_leaf(inst, c, route[static_cast<std::size_t>(p)])}) {
            if (step.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(step), view(acc));
            if (acc.xs.empty()) return {false, 0.0, 0.0};
        }
        if (p + 1 <= i - 1) {
            const Pwlf run = r.tree.query(p + 1, i - 1);
            if (run.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(run), view(acc));
            if (acc.xs.empty()) return {false, 0.0, 0.0};
        }
        // Seam where c was removed (i >= 1 since p <= i - 1 >= 0).
        if (i + 1 <= m - 1) {
            const Pwlf bridge = bridge_leaf(
                inst, route[static_cast<std::size_t>(i - 1)],
                route[static_cast<std::size_t>(i + 1)]);
            if (bridge.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(bridge), view(acc));
            if (acc.xs.empty()) return {false, 0.0, 0.0};
            if (i + 2 <= m) {
                const Pwlf rest = r.tree.query(i + 2, m);
                if (rest.xs.empty()) return {false, 0.0, 0.0};
                acc = compose(view(rest), view(acc));
            }
        } else {
            const Pwlf ret =
                return_leaf(inst, route[static_cast<std::size_t>(i - 1)]);
            if (ret.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(ret), view(acc));
        }
    } else {
        // p > i + 1. New order: r[0..i-1], r[i+1..p-1], c, r[p..m-1].
        acc = i > 0 ? r.tree.query(0, i - 1) : departure_identity(inst);
        if (acc.xs.empty()) return {false, 0.0, 0.0};
        const std::int32_t before = i > 0 ? route[static_cast<std::size_t>(i - 1)] : 0;
        const Pwlf seam =
            bridge_leaf(inst, before, route[static_cast<std::size_t>(i + 1)]);
        if (seam.xs.empty()) return {false, 0.0, 0.0};
        acc = compose(view(seam), view(acc));
        if (acc.xs.empty()) return {false, 0.0, 0.0};
        if (i + 2 <= p - 1) {
            const Pwlf run = r.tree.query(i + 2, p - 1);
            if (run.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(run), view(acc));
            if (acc.xs.empty()) return {false, 0.0, 0.0};
        }
        const Pwlf in_bridge = bridge_leaf(
            inst, route[static_cast<std::size_t>(p - 1)], c);
        if (in_bridge.xs.empty()) return {false, 0.0, 0.0};
        acc = compose(view(in_bridge), view(acc));
        if (acc.xs.empty()) return {false, 0.0, 0.0};
        if (p <= m - 1) {
            const Pwlf out_bridge =
                bridge_leaf(inst, c, route[static_cast<std::size_t>(p)]);
            if (out_bridge.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(out_bridge), view(acc));
            if (acc.xs.empty()) return {false, 0.0, 0.0};
            if (p + 1 <= m) {
                const Pwlf rest = r.tree.query(p + 1, m);
                if (rest.xs.empty()) return {false, 0.0, 0.0};
                acc = compose(view(rest), view(acc));
            }
        } else {
            const Pwlf ret = return_leaf(inst, c);
            if (ret.xs.empty()) return {false, 0.0, 0.0};
            acc = compose(view(ret), view(acc));
        }
    }
    return finish(acc);
}

}  // namespace kayros
