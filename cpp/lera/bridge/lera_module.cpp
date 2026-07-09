// kayros-owned pybind11 module for the vendored Lera-Romero BPC.
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <climits>
#include <string>
#include <vector>

#include <nyr/math/ndcpwlf.h>

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

    // --- nyr NDCPWLF test surface (M5.9 exact stepwise labeling) -----------
    // Direct pytest access to the step-capable PWL primitives while they are
    // built and hardened against the checker's left-continuous semantics,
    // before they replace goc::PWLFunction inside the labeling.
    auto nyr_mod = m.def_submodule(
        "nyr", "nyr NDCPWLF step-capable PWL primitives (M5.9 test surface)");
    py::class_<nyr::NDCPWLF>(nyr_mod, "NDCPWLF")
        .def(py::init<std::vector<double>, std::vector<double>>(), py::arg("xs"),
             py::arg("ys"))
        .def("evaluate", &nyr::NDCPWLF::evaluate, py::arg("x"))
        .def("__call__", &nyr::NDCPWLF::operator(), py::arg("x"))
        .def("nb_pieces", &nyr::NDCPWLF::nb_pieces)
        .def("empty", &nyr::NDCPWLF::empty)
        .def("check_invariant", &nyr::NDCPWLF::check_invariant)
        .def("check_normalization", &nyr::NDCPWLF::check_normalization)
        .def("compose", &nyr::NDCPWLF::compose, py::arg("g"))
        .def("inverse", &nyr::NDCPWLF::inverse)
        .def("get_xs", &nyr::NDCPWLF::get_xs)
        .def("get_ys", &nyr::NDCPWLF::get_ys)
        .def_property_readonly("min_domain", &nyr::NDCPWLF::get_min_domain)
        .def_property_readonly("max_domain", &nyr::NDCPWLF::get_max_domain)
        .def_property_readonly("min_image", &nyr::NDCPWLF::get_min_image)
        .def_property_readonly("max_image", &nyr::NDCPWLF::get_max_image)
        .def("__eq__", &nyr::NDCPWLF::operator==, py::arg("other"));
}
