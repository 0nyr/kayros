#pragma once

#include <cstddef>
#include <vector>
#include <goc/goc.h>

#include "nyr/vrp/types.h"
#include "nyr/solutions/route.h"
#include "nyr/math/ndcpwlf.h"
#include "nyr/vrp/instance.h"

namespace nyr {

/// Arc Ready Time Functions
using ARTFs = goc::Matrix<nyr::NDCPWLF>;

/// Create a matrix of ARTFs for the given VRP instance.
ARTFs make_artfs(const VRPInstance& instance);

/// @brief Performs tree chain composition of ARTFs
/// over the given path, following the process described in
/// Visser et al. 2020: doi = {10.1287/trsc.2019.0938}
/// NOTE: Do NOT save intermediate results, just returns
/// the final NDCPWLF.
template<typename PathContainer>
NDCPWLF perform_tree_chain_composition(
    const ARTFs& deltas,
    const PathContainer& path
) {
    using std::begin;
    using std::end;
    auto path_begin = begin(path);
    auto path_end = end(path);
    size_t path_size = std::distance(path_begin, path_end);

    #ifndef NDEBUG
    if (path_size <= 1) {
        throw std::invalid_argument("Path must contain at least one arc.");
    }
    #endif

    // First init loop done manually over refs of ARTFs
    size_t nb_composed_functions = path_size / 2;
    std::vector<nyr::NDCPWLF> composed_functions;
    composed_functions.reserve(nb_composed_functions);

    size_t k = (path_size - 1) / 2;
    auto it = path_begin;
    for (size_t i = 0; i < k; ++i) {
        auto v0 = *it++;
        auto v1 = *it++;
        auto v2 = *it;
        #ifndef NDEBUG
        std::clog << "Composing functions: " 
                  << "g: " << v0 << " -> " << v1
                  << " and f: " << v1 << " -> " << v2
                  << std::endl;
        #endif
        auto& g = deltas[v0][v1];
        auto& f = deltas[v1][v2];
        composed_functions.push_back(
            f.compose(g)
        );
    }
    if ((path_size - 1) % 2 == 1) {
        // If odd, append the last element to the next list of composed functions
        auto it_last1 = path_begin;
        std::advance(it_last1, path_size - 2);
        auto it_last2 = it_last1;
        ++it_last2;
        composed_functions.push_back(
            deltas[*it_last1][*it_last2]
        );
    }
    #ifndef NDEBUG
    std::clog << "Initial composed functions size: " 
              << composed_functions.size() 
              << ", k = " << k
              << std::endl;
    #endif

    // Loop of tree compositions
    while (composed_functions.size() > 1) {
        size_t nb_next_composed_functions = (composed_functions.size() + 1) / 2;
        std::vector<nyr::NDCPWLF> next_composed_functions;
        next_composed_functions.reserve(nb_next_composed_functions);
        k = composed_functions.size() / 2;
        for (size_t i = 0; i < k; ++i) {
            #ifndef NDEBUG
            std::clog << "Composing functions: "
                      << "g: " << "[" << 2*i << "] " 
                      << " and f: " << "[" << (2*i + 1) << "] "
                      << std::endl;
            #endif
            auto& g = composed_functions[2*i];
            auto& f = composed_functions[2*i + 1];
            next_composed_functions.push_back(
                f.compose(g)
            );
        }
        if (composed_functions.size() % 2 == 1) {
            next_composed_functions.push_back(
                std::move(composed_functions.back())
            );
        }
        composed_functions = std::move(next_composed_functions); 
    }

    return composed_functions[0];
}


/// @brief Performs sequential chain composition of ARTFs
/// over the given path. This is the equivalent of the Lera-Romero
/// procedure, which is not optimal.
/// NOTE: This function is intended to be used for testing and debugging purposes.
/// NOTE: Do NOT save intermediate results, just returns
/// the final NDCPWLF.
NDCPWLF perform_sequential_chain_composition(
    const ARTFs& deltas,
    const goc::GraphPath& path
);

/// @brief Computes the optimal departure time and duration
/// for a given \delta^{\textbf{r}} RRTF.
/// NOTE: If the path is infeasible, it returns {INFTY, INFTY}.
/// Complexity: O(p) where p is the number of breakpoints in the delta path.
std::pair<nyr::TimeUnit, nyr::TimeUnit> 
compute_optimal_departure_time_and_duration(
    const nyr::NDCPWLF& delta_path
);

/// @brief  Computes the optimal departure time and duration
/// for a given path, using tree-chain composition of ARTFs.
template<typename PathContainer>
std::pair<nyr::TimeUnit, nyr::TimeUnit> 
compute_optimal_departure_time_and_duration_from_path(
    const ARTFs& deltas,
    const PathContainer& path
) {
    nyr::NDCPWLF delta_path = perform_tree_chain_composition(deltas, path);
    return compute_optimal_departure_time_and_duration(delta_path);
}

/// Return a RouteDuration object that contains its own 
/// copy of the path, t0, and duration.
inline RouteDuration compute_RouteDuration(
    const nyr::NDCPWLF& delta_path,
    const goc::GraphPath& path
) {
    auto [t0, duration] = compute_optimal_departure_time_and_duration(delta_path);
    return RouteDuration(path, t0, duration);
}

/// Returns a RouteDuration provided its path.
inline RouteDuration compute_RouteDuration(
    const ARTFs& deltas,
    const goc::GraphPath& path
) {
    NDCPWLF delta_path = perform_tree_chain_composition(deltas, path);
    return compute_RouteDuration(delta_path, path);
}

/// Returns a RouteDuration provided its path.
/// Uses Lera-Romero procedure which is unoptimal.
RouteDuration compute_RouteDuration_lera(
    const VRPInstance& instance,
    const goc::GraphPath& path
);

/// Tree of delta functions
/// This is a tree of NDCPWLFs, where each node is a non-arc delta function.
/// The tree is built by performing the tree chain composition along the path.
/// More info, see Visser et al. 2020 & Blauth et al. 2024
/// It contains a hash map (subpath) -> NDCPWLF (associated delta function)
/// Such that evaluating a LS move can be done very fast.
class DeltaTree {
public:
    using Subpath = std::pair<goc::Vertex, goc::Vertex>;
    // Explicit hash: std::hash<std::pair> does not exist; GCC 15 rejects the
    // defaulted template argument at class instantiation (kayros vendored fix).
    struct SubpathHash {
        std::size_t operator()(const Subpath& s) const {
            return std::hash<goc::Vertex>()(s.first) * 1000003u ^ std::hash<goc::Vertex>()(s.second);
        }
    };
    using DeltaMap = std::unordered_map<Subpath, NDCPWLF, SubpathHash>;

    DeltaMap delta_map;
    goc::GraphPath path; // The (current) path.

    /// Constructs a DeltaTree from the given instance and path.
    DeltaTree(
        const VRPInstance& instance,
        const goc::GraphPath& path
    );
    
    /// Local Search Moves

};


} // namespace nyr