#include <chrono>
#include <cmath>
#include <random>
#include <utility>
#include <vector>

#include "core/queries.h"
#include "heuristics/heuristics.h"
#include "ls/ls.h"
#include "ls/perturb.h"

namespace kayros {

namespace {

// Rejection restore: vertex vectors + the full staleness bookkeeping. The
// staleness arrays MUST travel with the solution — stamps taken while the
// rejected candidate was live certify move coverage for THAT solution, not
// for the restored one.
struct IlsSnapshot {
    std::vector<std::vector<std::int32_t>> vertices;
    std::vector<std::int64_t> touched;
    std::vector<std::vector<std::int64_t>> last_tested;
    std::int64_t epoch = 0;
};

void take_snapshot(const SearchState& ss, IlsSnapshot& snap) {
    snap.vertices.clear();
    snap.vertices.reserve(ss.states.size());
    for (const RouteState& s : ss.states) snap.vertices.push_back(s.vertices);
    snap.touched = ss.touched;
    snap.last_tested = ss.last_tested;
    snap.epoch = ss.epoch;
}

void restore_snapshot(const Instance& inst, SearchState& ss,
                      const IlsSnapshot& snap) {
    ss.states.resize(snap.vertices.size());
    for (std::size_t k = 0; k < snap.vertices.size(); ++k) {
        if (ss.states[k].vertices != snap.vertices[k]) {
            const bool ok =
                build_route_state(inst, snap.vertices[k], ss.states[k]);
            (void)ok;  // snapshot solutions were feasible by invariant
        }
    }
    ss.touched = snap.touched;
    ss.last_tested = snap.last_tested;
    ss.epoch = snap.epoch;
}

std::vector<std::vector<std::int32_t>> extract_routes(const SearchState& ss) {
    std::vector<std::vector<std::int32_t>> routes;
    routes.reserve(ss.states.size());
    for (const RouteState& s : ss.states) routes.push_back(s.vertices);
    return routes;
}

}  // namespace

SolveResult solve_ils(const Instance& inst, const IlsParams& params,
                      std::uint64_t seed, double time_limit_seconds,
                      const IncumbentCallback& on_incumbent) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto elapsed = [&start]() {
        return std::chrono::duration<double>(Clock::now() - start).count();
    };
    Clock::time_point deadline_point{};
    const Clock::time_point* deadline = nullptr;
    if (time_limit_seconds > 0.0) {
        deadline_point = start + std::chrono::duration_cast<Clock::duration>(
                                     std::chrono::duration<double>(
                                         time_limit_seconds));
        deadline = &deadline_point;
    }
    const auto past = [&]() {
        return deadline != nullptr && Clock::now() >= *deadline;
    };

    SolveResult result;

    // Greedy seed + granular descent.
    std::vector<std::vector<std::int32_t>> seed_routes;
    if (!greedy_makespan(inst, seed_routes) ||
        solution_duration(inst, seed_routes) == kInfeasible) {
        result.status = SolveStatus::Infeasible;
        return result;
    }
    const NeighbourLists nb =
        build_neighbour_lists(inst, params.num_neighbours, params.weight_wait);
    const NeighbourLists exhaustive;  // k == 0 sentinel

    SearchState ss;
    if (!init_search_state(inst, seed_routes, ss)) {
        result.status = SolveStatus::Infeasible;
        return result;
    }
    double curr = ls_descend(inst, nb, ss, nullptr, deadline);

    double best = curr;
    result.routes = extract_routes(ss);
    result.value = best;
    result.incumbents.push_back({best, elapsed(), 0, 0});  // origin 0 = seed
    if (on_incumbent) on_incumbent(result.incumbents.back(), result.routes);

    // Exhaustive polish of the initial best (PyVRP's exhaustive_on_best).
    if (params.exhaustive_on_best && !past()) {
        mark_all_touched(ss);
        curr = ls_descend(inst, exhaustive, ss, nullptr, deadline);
        if (curr < best) {
            best = curr;
            result.routes = extract_routes(ss);
            result.value = best;
            result.incumbents.push_back({best, elapsed(), 0, 2});
            if (on_incumbent) on_incumbent(result.incumbents.back(), result.routes);
        }
    }

    std::mt19937_64 rng(seed);
    PerturbParams perturb_params;
    perturb_params.min_removals = params.min_perturbations;
    perturb_params.max_removals = params.max_perturbations;

    // LAHC slots (Burke & Bykov 2017 with PyVRP's RingBuffer semantics: the
    // index advances every iteration; a slot is rewritten only when the
    // current solution improves on it). Empty slots fall back to the initial
    // cost.
    const std::size_t history_len =
        params.history_length > 0
            ? static_cast<std::size_t>(params.history_length)
            : 1;
    std::vector<double> history(history_len,
                                std::numeric_limits<double>::quiet_NaN());
    std::uint64_t history_idx = 0;
    const double init_cost = curr;

    IlsSnapshot snapshot;
    SolveStatus status = SolveStatus::Finished;
    std::uint64_t iteration = 0;
    std::int64_t no_improvement = 0;

    for (; iteration < params.max_iterations; ++iteration) {
        if (past()) {
            status = SolveStatus::TimeLimit;
            break;
        }
        if (no_improvement == params.restart_no_improvement) {
            // Restart-to-best: fresh state, cleared history.
            if (!init_search_state(inst, result.routes, ss)) break;
            curr = best;
            std::fill(history.begin(), history.end(),
                      std::numeric_limits<double>::quiet_NaN());
            history_idx = 0;
            no_improvement = 0;
        }

        take_snapshot(ss, snapshot);
        perturb(inst, nb, ss, rng, perturb_params);
        double cand = ls_descend(inst, nb, ss, nullptr, deadline);

        ++no_improvement;
        if (cand < best) {
            no_improvement = 0;
            if (params.exhaustive_on_best && !past()) {
                mark_all_touched(ss);
                cand = ls_descend(inst, exhaustive, ss, nullptr, deadline);
            }
            best = cand;
            result.routes = extract_routes(ss);
            result.value = best;
            result.incumbents.push_back({best, elapsed(), iteration + 1, 2});
            if (on_incumbent) on_incumbent(result.incumbents.back(), result.routes);
        }

        // LAHC acceptance (enhancement 1: accept on improving the current).
        const std::size_t slot = static_cast<std::size_t>(
            history_idx % static_cast<std::uint64_t>(history_len));
        const bool slot_empty = std::isnan(history[slot]);
        const double late = slot_empty ? init_cost : history[slot];
        if (cand < late || cand < curr) {
            curr = cand;
        } else {
            restore_snapshot(inst, ss, snapshot);
        }
        // History update (enhancement 2: rewrite only on improvement).
        if (curr < late || slot_empty) history[slot] = curr;
        ++history_idx;
    }

    result.iterations_run = iteration;
    result.status = result.routes.empty() ? SolveStatus::Infeasible : status;
    return result;
}

}  // namespace kayros
