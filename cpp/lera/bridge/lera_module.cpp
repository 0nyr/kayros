// kayros-owned pybind11 module for the vendored Lera-Romero BPC.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <climits>
#include <string>
#include <vector>

#include "lera_bridge.h"

namespace py = pybind11;

PYBIND11_MODULE(_lera, m) {
    m.doc() = "Lera-Romero BPC for the TDVRPTW (KAYROS_WITH_LERA builds only; "
              "HiGHS LP backend by default, CPLEX optional at build time)";

    // Compile-time LP backend, surfaced for provenance (e.g. optimality
    // stamps naming the exact prover configuration).
#ifdef GOC_LP_BACKEND_HIGHS
    m.attr("LP_BACKEND") = "HiGHS";
#else
    m.attr("LP_BACKEND") = "CPLEX";
#endif

    m.def(
        "solve_duration_json",
        [](const std::string& payload, double time_limit_s, int cut_limit, int node_limit,
           int solution_limit, py::object on_incumbent,
           std::vector<std::vector<int>> initial_routes, double stab_alpha) {
            kayros::lera::SolveParams params;
            params.time_limit_s = time_limit_s;
            params.cut_limit = cut_limit;
            params.node_limit = node_limit;
            params.solution_limit = solution_limit;
            params.initial_routes = std::move(initial_routes);
            params.stab_alpha = stab_alpha;
            // The py::object must be captured while the GIL is still held (the
            // copy increfs); the hook itself re-acquires the GIL per call.
            if (!on_incumbent.is_none()) {
                params.on_incumbent = [on_incumbent](const std::string& incumbent_json) {
                    py::gil_scoped_acquire gil;
                    on_incumbent(incumbent_json);
                };
            }
            py::gil_scoped_release release;
            return kayros::lera::solve_duration_json(payload, params);
        },
        py::arg("payload"), py::arg("time_limit_s") = 7200.0, py::arg("cut_limit") = 100,
        py::arg("node_limit") = INT_MAX, py::arg("solution_limit") = 3000,
        py::arg("on_incumbent") = py::none(),
        py::arg("initial_routes") = std::vector<std::vector<int>>{},
        py::arg("stab_alpha") = 0.0);
}
