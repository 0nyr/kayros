#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

#include "core/exact_route_pricer.h"
#include "heuristics/reference_search.h"

namespace py = pybind11;
namespace td = mamut_routing::td_exact;

namespace {

py::int_ integer(const mpz_class& value) {
    std::string digits = value.get_str(16);
    PyObject* result = PyLong_FromString(digits.data(), nullptr, 16);
    if (!result) throw py::error_already_set();
    return py::reinterpret_steal<py::int_>(result);
}

py::tuple fraction(const td::Q& value) {
    return py::make_tuple(integer(value.get_num()), integer(value.get_den()));
}

}  // namespace

void bind_exact_routes(py::module_& m) {
    using namespace kayros;
    py::class_<ExactRoutePricer>(m, "ExactRoutePricer")
        .def(py::init<const Instance&, std::size_t>(), py::arg("instance"),
             py::arg("cache_knots") = ExactRoutePricer::default_budget, py::keep_alive<1, 2>())
        .def("route_timing", [](ExactRoutePricer& pricer, const Route& route) -> py::object {
            const auto timing = pricer.time(route);
            if (!timing) return py::none();
            return py::make_tuple(fraction(timing->duration), fraction(timing->departure));
        })
        .def("solution_cost", [](ExactRoutePricer& pricer, const Routes& routes) -> py::object {
            const auto cost = pricer.cost(routes);
            if (!cost) return py::none();
            return fraction(*cost);
        })
        .def_property_readonly("cached_knots", &ExactRoutePricer::cached_knots)
        .def_property_readonly("prepared_arcs", &ExactRoutePricer::prepared_arcs);

    py::class_<ReferenceResult>(m, "ReferenceResult")
        .def_readonly("routes", &ReferenceResult::routes)
        .def_property_readonly("exact_cost", [](const ReferenceResult& result) -> py::object {
            if (!result.exact_cost) return py::none();
            return fraction(*result.exact_cost);
        })
        .def_readonly("status", &ReferenceResult::status)
        .def_readonly("iterations_run", &ReferenceResult::iterations_run)
        .def_readonly("work_units", &ReferenceResult::work_units)
        .def_readonly("restarts", &ReferenceResult::restarts)
        .def_readonly("k_stats", &ReferenceResult::k_stats)
        .def_readonly("k_cap_stats", &ReferenceResult::k_cap_stats)
        .def_readonly("fleet_stats", &ReferenceResult::fleet_stats)
        .def_readonly("fd_stats", &ReferenceResult::fd_stats);

    m.def("solve_reference", [](const Instance& instance, const IlsParams& params, std::uint64_t seed,
                               double time_limit, py::object on_incumbent, const Routes& initial_routes,
                               bool fleet_cost_objective) {
        ReferenceCallback notify;
        if (!on_incumbent.is_none()) {
            notify = [&](const Routes& routes, const td::Q& cost, double seconds,
                         std::uint64_t iteration, std::int32_t origin) {
                py::gil_scoped_acquire gil;
                on_incumbent(routes, fraction(cost), seconds, iteration, origin);
            };
        }
        py::gil_scoped_release release;
        return solve_reference(instance, params, seed, time_limit, notify, initial_routes, fleet_cost_objective);
    }, py::arg("instance"), py::arg("params"), py::arg("seed"), py::arg("time_limit"),
       py::arg("on_incumbent") = py::none(), py::arg("initial_routes") = Routes{},
       py::arg("fleet_cost_objective") = false);
}
