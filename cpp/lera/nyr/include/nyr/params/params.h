#pragma once

#include <cstdint>

#include <goc/goc.h>

#include "nyr/solutions/objectives.h"
#include "nyr/time/time.h"

namespace nyr
{

class GlobalParams: public goc::Printable
{
public:
    const ProgramClock& pclock; // Program clock to measure time.
    
    const ObjectiveFunction objective; // global objective function to optimize.
    const nyr::Durex time_limit; // global time limit
    const bool initialization_heuristics; // whether to use initialization heuristics

    GlobalParams(
        const ProgramClock& pclock,
        ObjectiveFunction objective,
        nyr::Durex time_limit,
        bool initialization_heuristics
    ):  
        pclock(pclock),
        objective(objective), 
        time_limit(time_limit),
        initialization_heuristics(initialization_heuristics)
    {}

    void Print(std::ostream& os) const override
    {
        os << "Global Parameters:" << std::endl;
        os << "  Objective: " << objective << std::endl;
        os << "  Time limit: " << time_limit.count() << " seconds" << std::endl;
        os << "  Initialization heuristics: " << (initialization_heuristics ? "true" : "false") << std::endl;
    }
};

class BidirectionalLabelingParams: public goc::Printable
{
public: 
    const ProgramClock& pclock; // Program clock to measure time.

    // Pure labeling parameters
    const bool partial; // whether to use partial labeling
    const bool limited_extension; // whether to use limited extension
    const bool lazy_extension; // whether to use lazy extension
    const bool unreachable_strengthened; // whether to use unreachable strengthened
    const bool sort_by_cost; // whether to sort by cost
    const bool symmetric; // whether to use symmetric labeling
    const bool iterative_merge; // whether to use iterative merge

    // Labeling Algorithm levels (relaxation or exact)
    const bool lal_heuristic_cost; // whether to use heuristic cost*
    const bool lal_heuristic_elementarity; // whether to use heuristic elementarity
    const bool lal_heuristic_ng_routes; // whether to use heuristic NG routes
    const bool lal_exact_labeling; // whether to use exact labeling

    // NG-Routes parameters
    const double ratio_nb_neighbors; // ratio of neighbors to use for NG-Routes
    const int ng_nb_neighbors; // number of neighbors to use for NG-Routes
    const int ng_max_neighbors; // maximum number of neighbors to use for NG-Routes

    BidirectionalLabelingParams(
        const ProgramClock& pclock,
        bool partial,
        bool limited_extension,
        bool lazy_extension,
        bool unreachable_strengthened,
        bool sort_by_cost,
        bool symmetric,
        bool iterative_merge,
        bool lal_heuristic_cost,
        bool lal_heuristic_elementarity,
        bool lal_heuristic_ng_routes,
        bool lal_exact_labeling,
        double ratio_nb_neighbors,
        int ng_nb_neighbors,
        int ng_max_neighbors
    ):
        pclock(pclock),
        partial(partial),
        limited_extension(limited_extension),
        lazy_extension(lazy_extension),
        unreachable_strengthened(unreachable_strengthened),
        sort_by_cost(sort_by_cost),
        symmetric(symmetric),
        iterative_merge(iterative_merge),
        lal_heuristic_cost(lal_heuristic_cost),
        lal_heuristic_elementarity(lal_heuristic_elementarity),
        lal_heuristic_ng_routes(lal_heuristic_ng_routes),
        lal_exact_labeling(lal_exact_labeling),
        ratio_nb_neighbors(ratio_nb_neighbors),
        ng_nb_neighbors(ng_nb_neighbors),
        ng_max_neighbors(ng_max_neighbors)
    {}

    void Print(std::ostream& os) const override
    {
        os << "Bidirectional Labeling Parameters:" << std::endl;
        os << "  Partial: " << (partial ? "true" : "false") << std::endl;
        os << "  Limited extension: " << (limited_extension ? "true" : "false") << std::endl;
        os << "  Lazy extension: " << (lazy_extension ? "true" : "false") << std::endl;
        os << "  Unreachable strengthened: " << (unreachable_strengthened ? "true" : "false") << std::endl;
        os << "  Sort by cost: " << (sort_by_cost ? "true" : "false") << std::endl;
        os << "  Symmetric: " << (symmetric ? "true" : "false") << std::endl;
        os << "  Iterative merge: " << (iterative_merge ? "true" : "false") << std::endl;
        os << "  LAL Heuristic cost: " << (lal_heuristic_cost ? "true" : "false") << std::endl;
        os << "  LAL Heuristic elementarity: " << (lal_heuristic_elementarity ? "true" : "false") << std::endl;
        os << "  LAL Heuristic NG routes: " << (lal_heuristic_ng_routes ? "true" : "false") << std::endl;
        os << "  LAL Exact labeling: " << (lal_exact_labeling ? "true" : "false") << std::endl;
        if (lal_heuristic_ng_routes)
        {
            os << "  NG-Routes ratio nb neighbors: " << ratio_nb_neighbors << std::endl;
            os << "  NG-Routes nb neighbors: " << ng_nb_neighbors << std::endl;
            os << "  NG-Routes max neighbors: " << ng_max_neighbors << std::endl;
        }
    }
};

class BCPParams: public goc::Printable
{
public:
    const ProgramClock& pclock; // Program clock to measure time.

    const int cut_limit; // cut limit
    const int node_limit; // node limit

    BCPParams(
        const ProgramClock& pclock,
        int cut_limit,
        int node_limit
    ):
        pclock(pclock),
        cut_limit(cut_limit),
        node_limit(node_limit)
    {}

    void Print(std::ostream& os) const override
    {
        os << "BCP Parameters:" << std::endl;
        os << "  Cut limit: " << cut_limit << std::endl;
        os << "  Node limit: " << node_limit << std::endl;
    }
};

class AntColonyParams: public goc::Printable
{
public:
    const nyr::GlobalParams& gparams; // global parameters

    // ACO parameters
    const uint64_t max_nb_iterations; // maximum number of iterations
    const uint64_t max_no_improvement; // maximum number of iterations without improvement
    const uint32_t nb_ants; // number of ants
    const uint32_t alpha; // pheromone importance
    const uint32_t beta; // heuristic importance
    const double rho; // pheromone evaporation rate
    const double tau_min; // minimum pheromone level
    const double tau_0; // initial pheromone level
    const double tau_max; // maximum pheromone level
    const double delta_pheromone_threshold; // pheromone threshold for determining convergence

    // Constructor to initialize all const members
    AntColonyParams(
        const nyr::GlobalParams& gparams,
        uint64_t max_nb_iterations,
        uint64_t max_no_improvement,
        uint32_t nb_ants,
        uint32_t alpha,
        uint32_t beta,
        double rho,
        double tau_min,
        double tau_0,
        double tau_max,
        double delta_pheromone_threshold
    ):
        gparams(gparams),
        max_nb_iterations(max_nb_iterations),
        max_no_improvement(max_no_improvement),
        nb_ants(nb_ants),
        alpha(alpha),
        beta(beta),
        rho(rho),
        tau_min(tau_min),
        tau_0(tau_0),
        tau_max(tau_max),
        delta_pheromone_threshold(delta_pheromone_threshold)
    {}

    void Print(std::ostream& os) const override
    {
        os << "Ant Colony Params:" << std::endl;
        os << "  max_nb_iterations: " << max_nb_iterations << std::endl;
        os << "  max_no_improvement: " << max_no_improvement << std::endl;
        os << "  nb_ants: " << nb_ants << std::endl;
        os << "  alpha: " << alpha << std::endl;
        os << "  beta: " << beta << std::endl;
        os << "  rho: " << rho << std::endl;
        os << "  tau_min: " << tau_min << std::endl;
        os << "  tau_0: " << tau_0 << std::endl;
        os << "  tau_max: " << tau_max << std::endl;
        os << "  delta_pheromone_threshold: " << delta_pheromone_threshold << std::endl;
    }
};

} // namespace nyr