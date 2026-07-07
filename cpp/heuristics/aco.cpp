#include <chrono>
#include <cmath>
#include <random>
#include <utility>

#include "heuristics/construct.h"
#include "heuristics/heuristics.h"
#include "ls/ls.h"

namespace kayros {

namespace {

// Platform-stable uniform double in [0, 1) (implementation-defined behavior
// of std::uniform_real_distribution would break cross-platform determinism).
inline double rand01(std::mt19937_64& rng) {
    return std::ldexp(static_cast<double>(rng() >> 11), -53);
}

// Exponentiation by squaring with integer exponents (alpha/beta are ints).
inline double fast_pow(double base, std::uint32_t exponent) {
    double result = 1.0;
    while (exponent != 0) {
        if (exponent & 1u) result *= base;
        base *= base;
        exponent >>= 1u;
    }
    return result;
}

inline double bound_pheromone(double value, double tau_min, double tau_max) {
    if (value < tau_min) return tau_min;
    if (value > tau_max) return tau_max;
    return value;
}

// Relative earliest-ready-time heuristic: eta(v) = min_eat / eat(v) in (0, 1].
inline double heuristic_eta(double min_eat, double value) {
    if (value <= 0.0 || value == kInfeasible) return 0.0;
    if (min_eat <= 0.0 || min_eat == kInfeasible) return 1.0 / value;
    return min_eat / value;
}

// Roulette by binary search on the cumulative weight array: selects index k
// with probability (p[k] - p[k-1]) / p[nb-1].
inline std::size_t choose_candidate_index(const double* p, std::size_t nb,
                                          double f) {
    std::size_t left = 0;
    std::size_t right = nb - 1;
    const double total = p[nb - 1];
    while (left < right) {
        const std::size_t k = (left + right + 1) / 2;
        if (f < p[k - 1] / total) {
            right = k - 1;
        } else if (f > p[k] / total) {
            left = k + 1;
        } else {
            return k;
        }
    }
    return left;
}

struct Scratch {
    std::vector<double> direct;
    std::vector<double> eat;
    std::vector<std::int32_t> neighbors;
    std::vector<double> cumulative;
};

// One roulette selection step. Returns the chosen customer and its ready time,
// or -1 to close the route: either no candidate is reachable, or the drawn
// candidate fails the depot-return/capacity checks (the roll is spent and the
// customer stays available for later routes, as in the original heuristic).
std::int32_t select_next(const Instance& inst,
                         const std::vector<double>& pheromone,
                         const AcoParams& params, std::mt19937_64& rng,
                         std::int32_t current, double t, std::int64_t load,
                         const std::vector<std::uint8_t>& free_v,
                         Scratch& scratch, double* ready_out) {
    const std::int32_t nv = inst.num_vertices();
    scratch.direct.assign(static_cast<std::size_t>(nv), kInfeasible);
    scratch.neighbors.clear();
    for (std::int32_t v = 1; v < nv; ++v) {
        if (!free_v[v]) continue;
        const double ready = ready_next(inst, current, v, t);
        scratch.direct[v] = ready;
        if (ready != kInfeasible) scratch.neighbors.push_back(v);
    }
    if (scratch.neighbors.empty()) return -1;

    detail::earliest_ready_times(inst, current, t, free_v, scratch.eat);
    double min_eat = kInfeasible;
    for (std::int32_t v = 0; v < nv; ++v) {
        if (scratch.eat[v] < min_eat) min_eat = scratch.eat[v];
    }

    const std::size_t nb = scratch.neighbors.size();
    scratch.cumulative.resize(nb);
    double sum = 0.0;
    for (std::size_t idx = 0; idx < nb; ++idx) {
        const std::int32_t v = scratch.neighbors[idx];
        sum += fast_pow(pheromone[static_cast<std::size_t>(current) * nv + v],
                        params.alpha) *
               fast_pow(heuristic_eta(min_eat, scratch.eat[v]), params.beta);
        scratch.cumulative[idx] = sum;
    }

    std::size_t selected = 0;
    const double roll = rand01(rng);
    if (sum > 0.0) {
        selected = choose_candidate_index(scratch.cumulative.data(), nb, roll);
    }  // degenerate all-zero weights: fall back to the first candidate
    const std::int32_t next = scratch.neighbors[selected];

    const double next_ready = scratch.direct[next];
    if (!depot_return_feasible(inst, next, next_ready)) return -1;
    if (load + inst.demands[next] > inst.vehicle_capacity) return -1;
    *ready_out = next_ready;
    return next;
}

// Build one solution: every route departs at the earliest feasible depot time
// (FIFO makespan-construction trick); waiting and service live inside the
// ready-time queries. Returns false when construction gets stuck.
bool build_ant(const Instance& inst, const std::vector<double>& pheromone,
               const AcoParams& params, std::mt19937_64& rng, Scratch& scratch,
               std::vector<std::vector<std::int32_t>>& routes) {
    const std::int32_t n = inst.num_customers;
    std::vector<std::uint8_t> free_v(static_cast<std::size_t>(n) + 1, 1);
    free_v[0] = 0;
    std::int32_t remaining = n;
    routes.clear();
    const double dep_lo = departure_low(inst);
    while (remaining > 0) {
        std::vector<std::int32_t> path;
        std::int32_t current = 0;
        double t = dep_lo;
        std::int64_t load = 0;
        while (true) {
            double ready = 0.0;
            const std::int32_t next = select_next(inst, pheromone, params, rng,
                                                  current, t, load, free_v,
                                                  scratch, &ready);
            if (next < 0) break;
            path.push_back(next);
            t = ready;
            load += inst.demands[next];
            free_v[next] = 0;
            --remaining;
            current = next;
        }
        if (path.empty()) return false;  // guard against the original's infinite loop
        routes.push_back(std::move(path));
    }
    return true;
}

}  // namespace

SolveResult solve_aco(const Instance& inst, const AcoParams& params,
                      std::uint64_t seed, double time_limit_seconds,
                      const IncumbentCallback& on_incumbent) {
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    const auto elapsed = [&start]() {
        return std::chrono::duration<double>(Clock::now() - start).count();
    };

    SolveResult result;
    const std::int32_t nv = inst.num_vertices();

    // M7.0 granular candidate lists, built once per solve (exhaustive
    // sentinel when num_neighbours <= 0).
    const NeighbourLists nb = params.use_local_search
        ? build_neighbour_lists(inst, params.num_neighbours, params.weight_wait)
        : NeighbourLists{};

    // Greedy seed: the incumbent the colony must beat.
    double best_value = kInfeasible;
    {
        std::vector<std::vector<std::int32_t>> greedy_routes;
        if (greedy_makespan(inst, greedy_routes)) {
            double value = solution_duration(inst, greedy_routes);
            if (params.use_local_search && value != kInfeasible) {
                value = local_search(inst, nb, greedy_routes);
            }
            if (value != kInfeasible) {
                best_value = value;
                result.routes = std::move(greedy_routes);
                result.value = value;
                result.incumbents.push_back({value, elapsed(), 0, 0});
                if (on_incumbent) {
                    on_incumbent(result.incumbents.back(), result.routes);
                }
            }
        }
    }

    std::mt19937_64 rng(seed);
    std::vector<double> pheromone(
        static_cast<std::size_t>(nv) * static_cast<std::size_t>(nv),
        params.tau_0);
    std::vector<std::vector<std::vector<std::int32_t>>> ants(params.nb_ants);
    std::vector<double> values(params.nb_ants, kInfeasible);
    Scratch scratch;

    double pheromone_sum_last = 0.0;
    std::uint64_t quiet_iterations = 0;  // cumulative, never reset (original behavior)
    SolveStatus status = SolveStatus::Finished;
    std::uint64_t iteration = 0;

    for (; iteration < params.max_iterations; ++iteration) {
        // (a) construction — all ants, sequential, one shared RNG stream
        for (std::uint32_t ant = 0; ant < params.nb_ants; ++ant) {
            values[ant] = build_ant(inst, pheromone, params, rng, scratch, ants[ant])
                              ? solution_duration(inst, ants[ant])
                              : kInfeasible;
        }

        // (a') TD-LS (MMAS + local search): the improved solutions both
        // compete for the incumbent and deposit. Scope: the iteration-best
        // feasible ant (default) or every feasible ant.
        if (params.use_local_search) {
            if (params.ls_all_ants) {
                for (std::uint32_t ant = 0; ant < params.nb_ants; ++ant) {
                    if (values[ant] == kInfeasible) continue;
                    values[ant] = local_search(inst, nb, ants[ant]);
                }
            } else {
                std::uint32_t best_it = params.nb_ants;
                for (std::uint32_t ant = 0; ant < params.nb_ants; ++ant) {
                    if (values[ant] == kInfeasible) continue;
                    if (best_it == params.nb_ants || values[ant] < values[best_it]) {
                        best_it = ant;
                    }
                }
                if (best_it < params.nb_ants) {
                    values[best_it] = local_search(inst, nb, ants[best_it]);
                }
            }
        }

        // (b) evaporation on the full matrix, floor-clamped at tau_min
        for (double& tau : pheromone) {
            tau = bound_pheromone(tau * (1.0 - params.rho), params.tau_min,
                                  params.tau_max);
        }

        // (c) deposits (all feasible ants, per-arc MMAS clamping) + best tracking
        bool found_new_best = false;
        std::uint32_t best_ant = 0;
        for (std::uint32_t ant = 0; ant < params.nb_ants; ++ant) {
            if (values[ant] == kInfeasible) continue;
            const double delta_tau = 1.0 / values[ant];
            for (const auto& route : ants[ant]) {
                std::int32_t prev = 0;
                for (const std::int32_t v : route) {
                    double& tau = pheromone[static_cast<std::size_t>(prev) * nv + v];
                    tau = bound_pheromone(tau + delta_tau, params.tau_min,
                                          params.tau_max);
                    prev = v;
                }
                double& tau = pheromone[static_cast<std::size_t>(prev) * nv];
                tau = bound_pheromone(tau + delta_tau, params.tau_min,
                                      params.tau_max);
            }
            if (values[ant] < best_value) {
                best_value = values[ant];
                best_ant = ant;
                found_new_best = true;
            }
        }
        if (found_new_best) {
            result.routes = ants[best_ant];
            result.value = best_value;
            result.incumbents.push_back({best_value, elapsed(), iteration + 1, 1});
            if (on_incumbent) {
                on_incumbent(result.incumbents.back(), result.routes);
            }
        }

        // (d) convergence test on total pheromone mass
        double pheromone_sum = 0.0;
        for (const double tau : pheromone) pheromone_sum += tau;
        const double delta = std::abs(pheromone_sum - pheromone_sum_last);
        pheromone_sum_last = pheromone_sum;
        if (delta < params.delta_pheromone_threshold) ++quiet_iterations;
        if (quiet_iterations >= params.max_no_improvement) {
            status = SolveStatus::Converged;
            ++iteration;
            break;
        }

        // (e) wall-clock limit, once per iteration
        if (time_limit_seconds > 0.0 && elapsed() >= time_limit_seconds) {
            status = SolveStatus::TimeLimit;
            ++iteration;
            break;
        }
    }

    result.iterations_run = iteration;
    result.status = result.routes.empty() ? SolveStatus::Infeasible : status;
    return result;
}

}  // namespace kayros
