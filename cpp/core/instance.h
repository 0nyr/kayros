#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "pwlf/pwlf.h"

namespace kayros {

// Fold study (session 136): binary64 evaluation variants selected at run time. The fast fold is an internal approximation, the final solution is checked by the exact evaluator of the library. Arms are combinations of 3 switches:
// - in_place (F1): no composition with theta. Window, wait and service are applied on the breakpoints (apply_ready_time, restrict_domain).
// - leaf_cache (F2): the leaves theta_j o alpha_ij are built once per arc and cached on the instance. The reference fold then makes 1 composition per stop, and bridge_leaf / return_leaf return the cached leaf.
// - no_alloc (F3): the ranking evaluators compose into reusable buffers (compose_into, tree query views), with F0 arithmetic.
// F0 is the code before the study and is the default.
enum class FoldArm : std::int32_t { F0 = 0, F1 = 1, F2 = 2, F3 = 3, F4 = 4, F5 = 5 };

struct LeafCache {
    std::vector<Pwlf> leaf;   // (n+1)^2 entries, leaf[i*(n+1)+j] = theta_j o alpha_ij
    std::vector<char> ready;  // filled flag per entry
    std::vector<Pwlf> ret;    // n+1 entries, return leaf of i
    std::vector<char> ret_ready;
};

// TD instance in checker conventions: vertices 0..n with 0 = depot, customers
// 1..n; complete arc set carrying arrival-time functions alpha_ij over the
// horizon (arrival_at_j = alpha_ij(departure_from_i), FIFO by monotonicity).
struct Instance {
    std::int32_t num_customers = 0;   // n
    std::int32_t num_vehicles = -1;   // fleet upper bound; -1 = unbounded
    std::int64_t vehicle_capacity = 0;
    // FleetCostDuration fixed cost F per used (non-empty) route, in the
    // instance's time unit; 0 = plain Duration (every pre-M7 caller). The
    // solution objective is then fold + F * K with K the route count, priced
    // exactly like the checker: canonical duration fold first, one multiply,
    // one add.
    double fixed_route_cost = 0.0;
    double horizon_start = 0.0;
    double horizon_end = 0.0;
    bool has_time_windows = false;    // TDVRPTW when true, TDVRP otherwise
    std::vector<std::int64_t> demands;    // size n+1, demands[0] = 0
    std::vector<double> service_times;    // size n+1
    std::vector<double> tw_earliest;      // size n+1 iff has_time_windows
    std::vector<double> tw_latest;        // size n+1 iff has_time_windows

    // Arc ATF pool: arc (i,j) owns the slice [atf_offset[a], atf_offset[a+1])
    // of atf_xs/atf_ys, with a = i*(n+1)+j. Self-arcs have empty slices.
    std::vector<std::int64_t> atf_offset;  // size (n+1)*(n+1)+1
    std::vector<double> atf_xs;
    std::vector<double> atf_ys;

    // Fold study arm (see FoldArm). The cache is filled lazily by the leaf builders, and is reset whenever the arm changes (set_fold_arm).
    FoldArm fold_arm = FoldArm::F0;
    std::shared_ptr<LeafCache> leaf_cache;

    bool arm_in_place() const { return fold_arm == FoldArm::F1 || fold_arm == FoldArm::F5; }
    bool arm_leaf_cache() const {
        return fold_arm == FoldArm::F2 || fold_arm == FoldArm::F4 || fold_arm == FoldArm::F5;
    }
    bool arm_no_alloc() const {
        return fold_arm == FoldArm::F3 || fold_arm == FoldArm::F4 || fold_arm == FoldArm::F5;
    }
    void set_fold_arm(FoldArm arm) {
        fold_arm = arm;
        leaf_cache.reset();
        if (arm_leaf_cache()) {
            const std::size_t nv = static_cast<std::size_t>(num_vertices());
            leaf_cache = std::make_shared<LeafCache>();
            leaf_cache->leaf.resize(nv * nv);
            leaf_cache->ready.assign(nv * nv, 0);
            leaf_cache->ret.resize(nv);
            leaf_cache->ret_ready.assign(nv, 0);
        }
    }

    std::int32_t num_vertices() const { return num_customers + 1; }

    PwlfView arc(std::int32_t i, std::int32_t j) const {
        const std::int64_t a =
            static_cast<std::int64_t>(i) * num_vertices() + j;
        const std::int64_t b = atf_offset[a];
        const std::int64_t e = atf_offset[a + 1];
        return {atf_xs.data() + b, atf_ys.data() + b, e - b};
    }
};

struct RouteEval {
    bool feasible = false;
    double duration = 0.0;   // Delta*_r = min_t (delta_r(t) - t)
    double departure = 0.0;  // earliest optimal depot departure t*_r
};

// Route ready-time function delta_r over feasible depot departure times.
// Exact port of the checker's compute_route_ready_time_function: per vertex
// acc <- theta_v ∘ alpha_{prev,v} ∘ acc, with the TDVRPTW depot TW restriction
// at departure and on return. Returns the empty function when time-infeasible.
// route points to customer ids in 1..n, depot excluded.
Pwlf route_ready_time_function(const Instance& inst, const std::int32_t* route,
                               std::int64_t len);

// Optimal duration and earliest optimal departure (checker's compute_route_duration).
RouteEval evaluate_route(const Instance& inst, const std::int32_t* route,
                         std::int64_t len);

// Fold study leaf builders (core side, shared by the reference fold and the LS leaves). leaf = theta_to o alpha_{from,to}, return leaf = [due-date cut o] alpha_{from,0}. The *_uncached builders follow inst.arm_in_place(): composition with theta (F0 arithmetic) or apply_ready_time (F1). The cached accessors require inst.arm_leaf_cache() and fill the cache on first use.
Pwlf leaf_uncached(const Instance& inst, std::int32_t from, std::int32_t to);
Pwlf return_leaf_uncached(const Instance& inst, std::int32_t from);
const Pwlf& cached_leaf(const Instance& inst, std::int32_t from, std::int32_t to);
const Pwlf& cached_return_leaf(const Instance& inst, std::int32_t from);

}  // namespace kayros
