// kayros-owned pybind11 module for the vendored Lera-Romero BPC.
#include <pybind11/pybind11.h>

#include <string>

#include "lera_bridge.h"

namespace py = pybind11;

PYBIND11_MODULE(_lera, m) {
    m.doc() = "CPLEX-based Lera-Romero BPC for the TDVRPTW (KAYROS_WITH_LERA builds only)";

    m.def(
        "solve_duration_json",
        [](const std::string& payload, double time_limit_s, int cut_limit, int node_limit,
           int solution_limit) {
            kayros::lera::SolveParams params;
            params.time_limit_s = time_limit_s;
            params.cut_limit = cut_limit;
            params.node_limit = node_limit;
            params.solution_limit = solution_limit;
            return kayros::lera::solve_duration_json(payload, params);
        },
        py::arg("payload"), py::arg("time_limit_s") = 7200.0, py::arg("cut_limit") = 100,
        py::arg("node_limit") = INT_MAX, py::arg("solution_limit") = 3000,
        py::call_guard<py::gil_scoped_release>());
}
