#include "heuristics/reference_search.h"

#include <algorithm>
#include <numeric>
#include <random>
#include <stdexcept>

#include "core/exact_route_pricer.h"

namespace kayros {
namespace {
using Clock = std::chrono::steady_clock;
using Q = td::Q;

struct State {
    Routes routes;
    std::vector<Q> durations;
    Q cost = 0;
};

struct Rank {
    std::int64_t excess;
    Q cost;
    bool operator<(const Rank& other) const {
        return excess != other.excess ? excess < other.excess : cost < other.cost;
    }
};

class Search {
    const Instance& inst;
    const IlsParams& params;
    const ReferenceCallback& notify;
    Clock::time_point start = Clock::now();
    double limit;
    std::mt19937_64 rng;
    ExactRoutePricer pricer;
    Q charge;
    bool fleet;
    std::int32_t cap;
    ReferenceResult result;
    std::optional<State> incumbent;
    std::uint64_t last_best_iter = 0;
    std::int64_t last_best_work = 0;

    double elapsed() const { return std::chrono::duration<double>(Clock::now() - start).count(); }
    bool expired() const { return limit > 0 && elapsed() >= limit; }

    Rank rank(const State& state) const {
        auto k = static_cast<std::int64_t>(state.routes.size());
        return {cap > 0 ? std::max<std::int64_t>(0, k - cap) : 0, state.cost};
    }

    // The caller builds proposals by moving existing customers or inserting an
    // unserved customer. Unchanged routes retain their exact prices.
    std::optional<State> price(Routes routes, const State* previous = nullptr) {
        ++result.work_units;
        std::erase_if(routes, [](const Route& route) { return route.empty(); });
        if (inst.num_vehicles >= 0 && routes.size() > static_cast<std::size_t>(inst.num_vehicles))
            return std::nullopt;
        State trial{std::move(routes), {}, 0};
        trial.cost = charge * trial.routes.size();
        for (const auto& route : trial.routes) {
            if (previous) {
                auto it = std::find(previous->routes.begin(), previous->routes.end(), route);
                if (it != previous->routes.end()) {
                    auto& duration = previous->durations[it - previous->routes.begin()];
                    trial.durations.push_back(duration);
                    trial.cost += duration;
                    continue;
                }
            }
            std::int64_t load = 0;
            for (auto customer : route) {
                auto demand = inst.demands[customer];
                if (demand > inst.vehicle_capacity - load) return std::nullopt;
                load += demand;
            }
            auto timing = pricer.time(route);
            if (!timing) return std::nullopt;
            trial.cost += timing->duration;
            trial.durations.push_back(std::move(timing->duration));
        }
        return trial;
    }

    bool publish(const State& state, std::int32_t origin) {
        if (rank(state).excess) {
            ++result.k_cap_stats.rejected_above_cap;
            return false;
        }
        if (incumbent && state.cost >= incumbent->cost) return false;
        const auto k = static_cast<std::int64_t>(state.routes.size());
        auto& stats = result.k_stats;
        if (!incumbent) {
            stats.k_best_min = stats.k_best_max = k;
            if (cap > 0) {
                result.k_cap_stats.reached_cap = 1;
                result.k_cap_stats.first_capped_work = result.work_units;
            }
        } else {
            auto previous_k = static_cast<std::int64_t>(incumbent->routes.size());
            stats.new_best_k_up += k > previous_k;
            stats.new_best_k_down += k < previous_k;
            stats.k_best_min = std::min(stats.k_best_min, k);
            stats.k_best_max = std::max(stats.k_best_max, k);
        }
        incumbent = state;
        last_best_iter = result.iterations_run;
        last_best_work = result.work_units;
        if (notify) notify(state.routes, state.cost, elapsed(), result.iterations_run, origin);
        return true;
    }

    void check_coverage(const Routes& routes) const {
        std::vector<bool> seen(inst.num_vertices());
        for (const auto& route : routes) {
            if (route.empty()) throw std::invalid_argument("warm start contains an empty route");
            for (auto id : route) {
                if (id <= 0 || id > inst.num_customers || seen[id])
                    throw std::invalid_argument("warm start must cover each customer exactly once");
                seen[id] = true;
            }
        }
        if (std::find(seen.begin() + 1, seen.end(), false) != seen.end())
            throw std::invalid_argument("warm start must cover each customer exactly once");
    }

    // Try every insertion gap. A work threshold applies only during fleet drain.
    std::optional<State> insert(const State& state, std::int32_t customer, bool allow_new,
                                std::int64_t stop_work) {
        std::optional<State> best;
        for (std::size_t r = 0; r < state.routes.size() + allow_new; ++r) {
            bool singleton = r == state.routes.size();
            auto n = singleton ? 0 : state.routes[r].size();
            for (std::size_t pos = 0; pos <= n; ++pos) {
                if (expired() || result.work_units >= stop_work) return std::nullopt;
                Routes routes = state.routes;
                if (singleton) routes.emplace_back();
                routes[r].insert(routes[r].begin() + pos, customer);
                auto candidate = price(std::move(routes), &state);
                if (candidate && (!best || rank(*candidate) < rank(*best))) best = std::move(candidate);
            }
        }
        return best;
    }

    std::optional<State> repair(State state, const Route& removed, bool allow_new,
                                std::int64_t stop_work = std::numeric_limits<std::int64_t>::max()) {
        for (auto id : removed) {
            auto candidate = insert(state, id, allow_new, stop_work);
            if (!candidate) return std::nullopt;
            state = std::move(*candidate);
            if (!allow_new) ++result.fd_stats.step1_inserts;
        }
        return state;
    }

    // A customer whose singleton misses its window can still have a feasible
    // predecessor in a nonmetric instance. Revisit failed insertions after a
    // later customer changes the available routes.
    std::optional<State> construct() {
        Route remaining(inst.num_customers);
        std::iota(remaining.begin(), remaining.end(), 1);
        State state;
        while (!remaining.empty()) {
            Route deferred;
            for (auto id : remaining) {
                if (expired()) return std::nullopt;
                auto trial = insert(state, id, true, std::numeric_limits<std::int64_t>::max());
                if (trial) state = std::move(*trial);
                else deferred.push_back(id);
            }
            if (deferred.size() == remaining.size()) return std::nullopt;
            remaining = std::move(deferred);
        }
        return state;
    }

    // Exhaustive best improvement. Equal rational costs retain enumeration
    // order, including when two route orders describe the same solution.
    bool descent_step(State& state) {
        std::optional<State> best;
        auto test = [&](Routes routes) {
            auto candidate = price(std::move(routes), &state);
            if (candidate && rank(*candidate) < rank(best ? *best : state)) best = std::move(candidate);
        };
        const auto& src = state.routes;
        for (std::size_t a = 0; a < src.size() && !expired(); ++a) {
            for (std::size_t i = 0; i < src[a].size() && !expired(); ++i) {
                auto removed = src;
                auto customer = removed[a][i];
                removed[a].erase(removed[a].begin() + i);
                for (std::size_t b = 0; b < src.size() && !expired(); ++b) {
                    for (std::size_t j = 0; j <= removed[b].size() && !expired(); ++j) {
                        if (a == b && i == j) continue;
                        auto trial = removed;
                        trial[b].insert(trial[b].begin() + j, customer);
                        test(std::move(trial));
                    }
                    if (b < a) continue;
                    for (std::size_t j = b == a ? i + 1 : 0; j < src[b].size() && !expired(); ++j) {
                        auto trial = src;
                        std::swap(trial[a][i], trial[b][j]);
                        test(std::move(trial));
                        if (a == b && j > i + 2 && !expired()) {
                            trial = src;
                            std::reverse(trial[a].begin() + i, trial[a].begin() + j + 1);
                            test(std::move(trial));
                        }
                    }
                }
                if (src[a].size() > 1 && !expired()) {
                    removed.push_back({customer});
                    test(std::move(removed));
                }
            }
            for (std::size_t b = a + 1; b < src.size() && !expired(); ++b) {
                for (std::size_t i = 0; i <= src[a].size() && !expired(); ++i) {
                    for (std::size_t j = 0; j <= src[b].size() && !expired(); ++j) {
                        if ((i == 0 && j == 0) || (i == src[a].size() && j == src[b].size())) continue;
                        auto trial = src;
                        trial[a].resize(i);
                        trial[a].insert(trial[a].end(), src[b].begin() + j, src[b].end());
                        trial[b].resize(j);
                        trial[b].insert(trial[b].end(), src[a].begin() + i, src[a].end());
                        test(std::move(trial));
                    }
                }
            }
        }
        if (!best) return false;
        state = std::move(*best);
        publish(state, 2);
        return true;
    }

    void descend(State& state) {
        while (!expired() && descent_step(state)) {}
    }

    // Removing a whole route leaves its peers feasible. Commit a drain only
    // after every displaced customer fits and the complete result improves.
    void drain(State& state, bool initial) {
        if (!fleet || params.fd_attempts <= 0 || state.routes.size() <= 1 || expired()) return;
        auto& stats = result.fd_stats;
        ++stats.triggers;
        auto before = result.work_units;
        auto max_work = std::numeric_limits<std::int64_t>::max();
        auto stop = params.fd_work_cap > 0 ? before + std::min(params.fd_work_cap, max_work - before) : max_work;
        std::vector<std::size_t> order(state.routes.size());
        std::iota(order.begin(), order.end(), 0);
        std::stable_sort(order.begin(), order.end(), [&](auto a, auto b) {
            return state.routes[a].size() < state.routes[b].size();
        });
        for (std::size_t attempt = 0; attempt < order.size() && attempt < static_cast<std::size_t>(params.fd_attempts); ++attempt) {
            if (expired() || result.work_units >= stop) break;
            ++stats.attempts;
            if (initial && rank(state).excess) ++result.k_cap_stats.seed_drain_attempts;
            auto idx = order[attempt];
            auto routes = state.routes;
            routes.erase(routes.begin() + idx);
            auto base = price(std::move(routes), &state);
            auto repaired = repair(std::move(*base), state.routes[idx], false, stop);
            if (!repaired) {
                if (expired()) ++stats.rollbacks_time;
                else if (result.work_units >= stop) ++stats.rollbacks_work;
                else ++stats.rollbacks_deadend;
                continue;
            }
            if (rank(*repaired) < rank(state)) {
                ++stats.successes;
                state = std::move(*repaired);
                publish(state, 2);
                break;
            }
        }
        stats.evaluated += result.work_units - before;
    }

    std::optional<State> perturb(const State& state, bool& dissolved) {
        auto& stats = result.fleet_stats;
        ++stats.kicks_total;
        Route ids;
        for (auto& r : state.routes) ids.insert(ids.end(), r.begin(), r.end());
        if (ids.empty()) return std::nullopt;
        std::shuffle(ids.begin(), ids.end(), rng);
        auto hi = std::clamp(params.max_perturbations, 1, inst.num_customers);
        auto lo = std::clamp<std::int32_t>(params.min_perturbations, 1, hi);
        ids.resize(std::uniform_int_distribution<std::int32_t>(lo, hi)(rng));
        dissolved = fleet && state.routes.size() > 1 && std::uniform_int_distribution<int>(0, 99)(rng) < params.dissolve_pct;
        if (dissolved) {
            auto index = std::uniform_int_distribution<std::size_t>(0, state.routes.size() - 1)(rng);
            for (auto id : state.routes[index])
                if (std::find(ids.begin(), ids.end(), id) == ids.end()) ids.push_back(id);
            std::shuffle(ids.begin(), ids.end(), rng);
        }
        auto routes = state.routes;
        std::vector<bool> removed(inst.num_vertices());
        for (auto id : ids) removed[id] = true;
        for (auto& route : routes) std::erase_if(route, [&](auto id) { return removed[id]; });
        // Shortcuts in a nonmetric TD instance can make the residual infeasible.
        auto base = price(std::move(routes), &state);
        if (!base) return std::nullopt;
        auto before_repair = base->routes.size();
        auto repaired = repair(std::move(*base), ids, true);
        if (repaired) {
            ++stats.kicks_applied;
            stats.dissolved_armed += dissolved;
            stats.normal_kicks += !dissolved;
            auto k = repaired->routes.size();
            result.k_stats.singleton_opens += k - before_repair;
            result.k_stats.kicks_opening += k > before_repair;
            if (dissolved) {
                stats.dissolve_undone_in_kick += k > before_repair;
                stats.k_after_kick_lt += k < state.routes.size();
                stats.k_after_kick_eq += k == state.routes.size();
                stats.k_after_kick_gt += k > state.routes.size();
            }
        }
        return repaired;
    }

public:
    Search(const Instance& i, const IlsParams& p, std::uint64_t seed, double seconds,
           const ReferenceCallback& cb, bool fleet_objective)
        : inst(i), params(p), notify(cb), limit(seconds), rng(seed), pricer(i),
          charge(td::exact(i.fixed_route_cost)), fleet(fleet_objective || charge > 0), cap(fleet ? p.k_cap : 0) {}

    ReferenceResult run(const Routes& warm) {
        if (params.sq_work_cap > 0 || params.sq_ladder)
            throw std::invalid_argument("exact reference search does not support squeeze");
        std::optional<State> state;
        if (!warm.empty()) {
            check_coverage(warm);
            state = price(warm);
            if (!state) throw std::invalid_argument("warm start is infeasible under exact route pricing or resource limits");
        } else {
            state = construct();
        }
        if (state) {
            result.k_stats.k_seed = state->routes.size();
            publish(*state, warm.empty() ? 0 : 3);
            drain(*state, true);
            descend(*state);
            iterate(*state);
        }
        if (incumbent) {
            result.routes = std::move(incumbent->routes);
            result.exact_cost = std::move(incumbent->cost);
            result.k_stats.k_final = result.routes.size();
        }
        result.status = expired() ? SolveStatus::TimeLimit : result.exact_cost ? SolveStatus::Finished : SolveStatus::Infeasible;
        return std::move(result);
    }

    void iterate(State& state) {
        if (state.routes.empty()) return;
        std::vector<Rank> history(std::max(1, params.lahc_history), rank(state));
        auto last_drain = result.work_units;
        for (; result.iterations_run < params.ils_max_iterations && !expired();) {
            ++result.iterations_run;
            bool dissolved = false;
            auto proposed = perturb(state, dissolved);
            auto& slot = history[(result.iterations_run - 1) % history.size()];
            if (proposed) {
                auto old_k = state.routes.size();
                auto& ks = result.k_stats;
                auto& fs = result.fleet_stats;
                auto prior_best = incumbent ? std::optional{incumbent->cost} : std::nullopt;
                auto prior_best_k = incumbent ? incumbent->routes.size() : 0;
                ks.k_up_after_kick += proposed->routes.size() > old_k;
                ks.k_down_after_kick += proposed->routes.size() < old_k;
                publish(*proposed, 2);
                descend(*proposed);
                auto k = proposed->routes.size();
                ks.k_up_after_descent += k > old_k;
                ks.k_down_after_descent += k < old_k;
                if (dissolved) {
                    fs.k_after_descent_lt += k < old_k;
                    fs.k_after_descent_eq += k == old_k;
                    fs.k_after_descent_gt += k > old_k;
                }
                if (incumbent && (!prior_best || incumbent->cost < *prior_best)) {
                    fs.dissolved_new_best += dissolved;
                    fs.normal_new_best += !dissolved;
                    fs.dissolved_new_best_k_lt += dissolved && prior_best && incumbent->routes.size() < prior_best_k;
                }
                auto trial = rank(*proposed);
                auto current = rank(state);
                if (trial.excess <= current.excess && (!(current < trial) || !(slot < trial))) {
                    ks.accepted_k_up += k > old_k;
                    ks.accepted_k_down += k < old_k;
                    fs.dissolved_accepted_lahc += dissolved;
                    fs.normal_accepted_lahc += !dissolved;
                    state = std::move(*proposed);
                }
            }
            slot = rank(state);
            bool restart_iter = params.restart_no_improvement > 0 &&
                result.iterations_run - last_best_iter >= static_cast<std::uint64_t>(params.restart_no_improvement);
            bool restart_work = params.restart_no_improvement_work > 0 &&
                result.work_units - last_best_work >= params.restart_no_improvement_work;
            bool restart = incumbent && (restart_iter || restart_work);
            if (restart) {
                ++result.restarts;
                state = *incumbent;
                std::fill(history.begin(), history.end(), rank(state));
                last_best_iter = result.iterations_run;
                last_best_work = result.work_units;
            }
            if (restart || (params.fd_period_work > 0 && result.work_units - last_drain >= params.fd_period_work)) {
                drain(state, false);
                last_drain = result.work_units;
                descend(state);
            }
        }
    }
};
}  // namespace

ReferenceResult solve_reference(const Instance& instance, const IlsParams& params, std::uint64_t seed,
                                double time_limit, const ReferenceCallback& notify, const Routes& initial_routes,
                                bool fleet_cost_objective) {
    return Search(instance, params, seed, time_limit, notify, fleet_cost_objective).run(initial_routes);
}
}  // namespace kayros
