#include <goc/goc.h>
#include <nyr/nyr.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <optional>
#include <stdexcept>

#include "labeling/bidirectional_labeling.h"

using namespace goc;
using namespace nyr;
using namespace solver;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

double cost(int u, int v, int seed) {
    return (u == 0 || v == 5) ? 1.0 : 1.0 + ((u * 7 + v * 3 + seed) % 4) / 4.0;
}

VRPInstance instance(int seed) {
    VRPInstance vrp;
    vrp.D = Digraph(6);
    vrp.o = 0;
    vrp.d = 5;
    vrp.T = 20.0;
    vrp.horizon = {0.0, vrp.T};
    vrp.Q = 3.0;
    vrp.q = {0.0, 1.0, 1.0, 1.0, 1.0, 0.0};
    vrp.tw = std::vector<goc::Interval>(6, vrp.horizon);
    vrp.time_steps = {vrp.horizon};
    vrp.tau = vrp.pretau = vrp.arr = vrp.dep = Matrix<PWLFunction>(6, 6);
    vrp.LDT = Matrix<double>(6, 6, -goc::INFTY);
    for (int u = 0; u < 6; ++u) {
        vrp.tau[u][u] = vrp.pretau[u][u] = PWLFunction::ConstantFunction(0.0, vrp.horizon);
        vrp.arr[u][u] = vrp.dep[u][u] = PWLFunction::IdentityFunction(vrp.horizon);
        vrp.LDT[u][u] = vrp.T;
        for (int v = 0; v < 6; ++v) {
            if (u == v || u == vrp.d || v == vrp.o) continue;
            vrp.D.AddArc({u, v});
            const double travel = cost(u, v, seed);
            vrp.tau[u][v] = PWLFunction::ConstantFunction(travel, {0.0, vrp.T - travel});
            vrp.arr[u][v].AddPiece(LinearFunction({0.0, travel}, {vrp.T - travel, vrp.T}));
            vrp.dep[u][v].AddPiece(LinearFunction({travel, 0.0}, {vrp.T, vrp.T - travel}));
            vrp.pretau[u][v] = PWLFunction::ConstantFunction(travel, {travel, vrp.T});
            vrp.LDT[u][v] = vrp.T - travel;
        }
    }
    return vrp;
}

double route_reduced_cost(const GraphPath& path, const PricingProblem& problem, int seed) {
    require(path.size() >= 3 && path.front() == 0 && path.back() == 5,
            "Pricing returned an invalid depot path");
    unsigned visited = 0;
    double value = 0.0;
    int cut_visits = 0;
    for (std::size_t k = 1; k < path.size(); ++k) {
        const int u = path[k - 1], v = path[k];
        require(!(u == 1 && v == 2), "Pricing returned a forbidden arc");
        value += cost(u, v, seed);
        if (v == 5) continue;
        require(v >= 1 && v <= 4 && !(visited & (1U << v)), "Pricing returned a nonelementary path");
        visited |= 1U << v;
        value -= problem.P[v];
        if (v <= 3) ++cut_visits;
    }
    require(path.size() - 2 <= 3, "Pricing exceeded capacity");
    if (cut_visits >= 2) value -= problem.sigma[0];
    return value;
}

double oracle(const PricingProblem& problem, int seed) {
    double best = goc::INFTY;
    GraphPath path{0};
    std::function<void(unsigned)> visit = [&](unsigned used) {
        if (path.size() > 1) {
            path.push_back(5);
            best = std::min(best, route_reduced_cost(path, problem, seed));
            path.pop_back();
        }
        if (path.size() == 4) return;
        for (int v = 1; v <= 4; ++v) {
            if ((used & (1U << v)) || (path.back() == 1 && v == 2)) continue;
            path.push_back(v);
            visit(used | (1U << v));
            path.pop_back();
        }
    };
    visit(0);
    return best;
}

int main() {
    try {
        int calls = 0;
        long long corrected = 0;
        for (bool arithmetic : {false, true}) {
            StepExactArithmeticScope scope(arithmetic);
            for (int seed : {0, 1}) {
                const auto vrp = instance(seed);
                for (bool correction : {false, true})
                    for (bool partial : {false, true})
                        for (bool closing : {false, true})
                            for (bool sorting : {false, true}) {
                                const std::optional<TDNGRoutesParams> ng = std::nullopt;
                                BidirectionalLabeling labeling(vrp, ng);
                                labeling.correcting = correction;
                                labeling.partial = partial;
                                labeling.closing_state = closing;
                                labeling.sort_by_cost = sorting;
                                for (bool negative : {true, false}) {
                                    PricingProblem problem;
                                    problem.A = {{1, 2}};
                                    problem.P = negative ? std::vector<double>{0.0, 2.5, 2.75, 2.25, 3.0, 0.0}
                                                         : std::vector<double>(6, 0.0);
                                    SubsetRowCut cut;
                                    cut.set(1).set(2).set(3);
                                    problem.S = {cut};
                                    problem.sigma = {-0.5};
                                    const double expected = oracle(problem, seed);
                                    std::vector<goc::Route> routes;
                                    const auto log = labeling.Run(problem, &routes, LabelingLevel::Exact);
                                    require(log.status == BLBStatus::Finished, "Tiny exact pricing must complete");
                                    double observed = goc::INFTY;
                                    for (const auto& route : routes) {
                                        const double reduced = route_reduced_cost(route.path, problem, seed);
                                        double duration = 0.0;
                                        for (std::size_t k = 1; k < route.path.size(); ++k)
                                            duration += cost(route.path[k-1], route.path[k], seed);
                                        require(std::abs(route.duration - duration) < 1e-9,
                                                "Returned duration differs from independent constant-arc sum");
                                        require(reduced < -1e-6, "Pricing returned a nonnegative reduced-cost route");
                                        observed = std::min(observed, reduced);
                                    }
                                    if (expected < -1e-6)
                                        require(std::abs(observed - expected) < 1e-9,
                                                "Exact pricing lost the exhaustive oracle's best negative route");
                                    else require(routes.empty(), "Nonnegative pricing oracle requires an empty pool");
                                    corrected += *log.forward_log->corrected_count + *log.backward_log->corrected_count;
                                    ++calls;
                                }
                            }
            }
        }
        std::cout << calls << " synthetic exact-pricing oracle checks pass, corrected count " << corrected << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
