#include "nyr/vrp/delta.h"

namespace nyr {

ARTFs make_artfs(const VRPInstance& instance) {
    size_t nb_vertices = instance.D.NbVertices();
    ARTFs artfs(nb_vertices, nb_vertices);
    for (auto& e: instance.D.Arcs()) {
        // Get the arrival time function for arc e.
        // After Lera's preprocessing, this "arrival time function"
        // is the same an ARTF (Arc Ready Time Function).
        auto& arr = instance.arr[e.tail][e.head];
        artfs[e.tail][e.head] = NDCPWLF(arr);
    }
    return artfs;
}

// NDCPWLF perform_tree_chain_composition(
//     const VRPInstance& instance,
//     const ARTFs& deltas,
//     const goc::GraphPath& path
// ) {
//     #ifndef NDEBUG
//     if (path.size() - 1 <= 0) {
//         throw std::invalid_argument("Path must contain at least one arc.");
//     }
//     #endif

//     // First init loop done manually over refs of ARTFs
//     size_t nb_composed_functions = (size_t) path.size() / 2;
//     std::vector<nyr::NDCPWLF> composed_functions;
//     composed_functions.reserve(nb_composed_functions);
//     size_t k = (size_t) (path.size() - 1) / 2;
//     for (size_t i = 0; i < k; ++i) {
//         #ifndef NDEBUG
//         std::clog << "Composing functions: " 
//                   << "g: " << path[2*i] << " -> " << path[2*i + 1] 
//                   << " and f: " << path[2*i + 1] << " -> " << path[2*i + 2]
//                   << std::endl;
//         #endif
//         auto& g = deltas[path[2*i]][path[2*i + 1]];
//         auto& f = deltas[path[2*i + 1]][path[2*i + 2]];
//         composed_functions.push_back(
//             f.compose(g)
//         );
//     }
//     if ((path.size() - 1) % 2 == 1) {
//         // If odd, append the last element to the next list of composed functions
//         composed_functions.push_back(
//             // Copy of the last arc ARTF
//             deltas[path[path.size() - 2]][path[path.size() - 1]]
//         );
//     }
//     #ifndef NDEBUG
//     std::clog << "Initial composed functions size: " 
//               << composed_functions.size() 
//               << ", k = " << k
//               << std::endl;
//     #endif

//     // Loop of tree compositions
//     while (composed_functions.size() > 1) {
//         size_t nb_next_composed_functions = (composed_functions.size() + 1) / 2;
//         std::vector<nyr::NDCPWLF> next_composed_functions;
//         next_composed_functions.reserve(nb_next_composed_functions);
//         k = (composed_functions.size()) / 2;
//         for (size_t i = 0; i < k; ++i) {
//             #ifndef NDEBUG
//             std::clog << "Composing functions: "
//                       << "g: " << "[" << 2*i << "] " 
//                       << " and f: " << "[" << (2*i + 1) << "] "
//                       << std::endl;
//             #endif
//             auto& g = composed_functions[2*i];
//             auto& f = composed_functions[2*i + 1];
//             next_composed_functions.push_back(
//                 f.compose(g)
//             );
//         }
//         if (composed_functions.size() % 2 == 1) {
//             next_composed_functions.push_back(
//                 // Move last function into next vector
//                 std::move(composed_functions.back())
//             );
//         }
//         composed_functions = next_composed_functions; 
//     }

//     return composed_functions[0];
// }

NDCPWLF perform_sequential_chain_composition(
    const ARTFs& deltas,
    const goc::GraphPath& path
) {
    #ifndef NDEBUG
    if (path.size() - 1 <= 0) {
        throw std::invalid_argument("Path must contain at least one arc.");
    }
    #endif

    NDCPWLF composed = deltas[path[0]][path[1]];
    std::clog << "Initial composed function: " 
        << path[0] << " -> " << path[1] << std::endl;

    for (size_t k = 1; k < path.size() - 1; ++k) {
        goc::Vertex i = path[k], j = path[k + 1];
        auto& delta_ij = deltas[i][j];
        composed = delta_ij.compose(composed);
        std::clog << "Composing functions: "
                  << "f: " << i << " -> " << j
                  << " with g: " << path[0] << " ~~> " << i
                  << std::endl;
        if (composed.empty()) {
            // Early return
            return NDCPWLF();
        }
    }

    return composed;
}

std::pair<nyr::TimeUnit, nyr::TimeUnit> 
compute_optimal_departure_time_and_duration(
    const nyr::NDCPWLF& delta_path
) {
    // If the delta_path is empty, return INFTY
    if (delta_path.empty()) {
        return {goc::INFTY, goc::INFTY};
    }

    auto& xs = delta_path.get_xs();
    auto& ys = delta_path.get_ys();
    
    // Compute Delta = delta_path - Identity
    // Keep track of the minimum value of ys
    double min_y = goc::INFTY;
    double associated_x = goc::INFTY;
    for (size_t i = 0; i < ys.size(); ++i) {
        double y_curr = ys[i] - xs[i];
        if (y_curr < min_y) {
            min_y = y_curr;
            associated_x = xs[i];
        }
    }

    #ifndef NDEBUG
    if (goc::epsilon_bigger_equal(min_y, goc::INFTY)) {
        throw std::runtime_error("The minimum value of the delta path is INFTY.");
    }
    if (goc::epsilon_smaller(min_y, 0.0)) {
        throw std::runtime_error("The minimum value of the delta path is negative, which is not allowed.");
    }
    #endif 

    return {
        associated_x, // Optimal departure time
        min_y         // Optimal duration
    };
}

// The following code reuses the original Lera's code
RouteDuration compute_RouteDuration_lera(
    const VRPInstance& instance,
    const goc::GraphPath& path
) {
    goc::PWLFunction Delta = instance.arr[path[0]][path[0]];
	if (Delta.Empty()) return {{}, goc::INFTY, goc::INFTY};
	for (size_t k = 0; k < path.size() - 1; ++k)
	{
		goc::Vertex i = path[k], j = path[k+1];
		Delta = instance.arr[i][j].Compose(Delta);
		if (Delta.Empty()) return {{}, goc::INFTY, goc::INFTY};
	}
	Delta = Delta - goc::PWLFunction::IdentityFunction(dom(Delta));
	double min_img_Delta = std::min(img(Delta));
	return RouteDuration(
		path, 
		Delta.PreValue(min_img_Delta), 
		min_img_Delta
	);
}

} // namespace nyr