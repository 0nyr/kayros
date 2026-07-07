#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "core/instance.h"
#include "heuristics/heuristics.h"
#include "ls/ls.h"
#include "pwlf/pwlf.h"

namespace py = pybind11;

#ifndef KAYROS_VERSION
#define KAYROS_VERSION "0.0.0"
#endif

namespace {

kayros::Pwlf make_pwlf(std::vector<double> xs, std::vector<double> ys) {
    if (xs.size() != ys.size()) {
        throw std::invalid_argument("xs and ys must have the same length");
    }
    return {std::move(xs), std::move(ys)};
}

using ArcSpec = std::tuple<std::int32_t, std::int32_t, std::vector<double>,
                           std::vector<double>>;

kayros::Instance make_instance(
    std::int32_t num_customers, std::optional<std::int32_t> num_vehicles,
    std::int64_t vehicle_capacity, std::pair<double, double> horizon,
    std::optional<std::vector<std::pair<double, double>>> time_windows,
    std::vector<std::int64_t> demands, std::vector<double> service_times,
    const std::vector<ArcSpec>& arcs) {
    kayros::Instance inst;
    inst.num_customers = num_customers;
    inst.num_vehicles = num_vehicles.value_or(-1);
    inst.vehicle_capacity = vehicle_capacity;
    inst.horizon_start = horizon.first;
    inst.horizon_end = horizon.second;
    const std::int64_t nv = inst.num_vertices();
    const std::size_t expected = static_cast<std::size_t>(nv);
    if (demands.size() != expected || service_times.size() != expected) {
        throw std::invalid_argument(
            "demands and service_times must have num_customers + 1 entries");
    }
    inst.demands = std::move(demands);
    inst.service_times = std::move(service_times);
    if (time_windows.has_value()) {
        if (time_windows->size() != expected) {
            throw std::invalid_argument(
                "time_windows must have num_customers + 1 entries");
        }
        inst.has_time_windows = true;
        inst.tw_earliest.reserve(expected);
        inst.tw_latest.reserve(expected);
        for (const auto& [earliest, latest] : *time_windows) {
            inst.tw_earliest.push_back(earliest);
            inst.tw_latest.push_back(latest);
        }
    }

    const std::int64_t num_arcs = nv * nv;
    std::vector<std::int64_t> lengths(static_cast<std::size_t>(num_arcs), 0);
    std::int64_t total = 0;
    for (const auto& [i, j, xs, ys] : arcs) {
        if (i < 0 || i >= nv || j < 0 || j >= nv || i == j) {
            throw std::invalid_argument("invalid arc endpoints");
        }
        if (xs.size() != ys.size() || xs.size() < 2) {
            throw std::invalid_argument("arc ATF must have >= 2 breakpoints");
        }
        const std::int64_t a = static_cast<std::int64_t>(i) * nv + j;
        if (lengths[static_cast<std::size_t>(a)] != 0) {
            throw std::invalid_argument("duplicate arc");
        }
        lengths[static_cast<std::size_t>(a)] =
            static_cast<std::int64_t>(xs.size());
        total += static_cast<std::int64_t>(xs.size());
    }
    inst.atf_offset.assign(static_cast<std::size_t>(num_arcs) + 1, 0);
    for (std::int64_t a = 0; a < num_arcs; ++a) {
        inst.atf_offset[static_cast<std::size_t>(a) + 1] =
            inst.atf_offset[static_cast<std::size_t>(a)] +
            lengths[static_cast<std::size_t>(a)];
    }
    inst.atf_xs.assign(static_cast<std::size_t>(total), 0.0);
    inst.atf_ys.assign(static_cast<std::size_t>(total), 0.0);
    for (const auto& [i, j, xs, ys] : arcs) {
        const std::int64_t a = static_cast<std::int64_t>(i) * nv + j;
        const std::int64_t b = inst.atf_offset[static_cast<std::size_t>(a)];
        std::copy(xs.begin(), xs.end(),
                  inst.atf_xs.begin() + static_cast<std::ptrdiff_t>(b));
        std::copy(ys.begin(), ys.end(),
                  inst.atf_ys.begin() + static_cast<std::ptrdiff_t>(b));
    }
    return inst;
}

}  // namespace

PYBIND11_MODULE(_core, m) {
    m.doc() =
        "kayros compiled core: NDCPWLF engine, POD instance/route model, "
        "heuristics";
    m.attr("__version__") = KAYROS_VERSION;

    // --- pwlf primitives (exposed for the checker-equivalence test suite) ---
    m.def("pwlf_identity", [](double low, double high) {
        kayros::Pwlf f = kayros::identity(low, high);
        return py::make_tuple(std::move(f.xs), std::move(f.ys));
    });
    m.def("pwlf_evaluate", [](std::vector<double> xs, std::vector<double> ys,
                              double x) {
        const kayros::Pwlf f = make_pwlf(std::move(xs), std::move(ys));
        return kayros::evaluate(kayros::view(f), x);
    });
    m.def("pwlf_compose", [](std::vector<double> f_xs, std::vector<double> f_ys,
                             std::vector<double> g_xs,
                             std::vector<double> g_ys) {
        const kayros::Pwlf f = make_pwlf(std::move(f_xs), std::move(f_ys));
        const kayros::Pwlf g = make_pwlf(std::move(g_xs), std::move(g_ys));
        kayros::Pwlf h = kayros::compose(kayros::view(f), kayros::view(g));
        return py::make_tuple(std::move(h.xs), std::move(h.ys));
    });
    m.def("pwlf_min_shifted_image",
          [](std::vector<double> xs, std::vector<double> ys) {
              const kayros::Pwlf f = make_pwlf(std::move(xs), std::move(ys));
              const kayros::MinShift s = kayros::min_shifted_image(kayros::view(f));
              return py::make_tuple(s.value, s.argmin_x);
          });
    m.def("pwlf_make_theta",
          [](double earliest, double latest, double service_time) {
              kayros::Pwlf f = kayros::make_theta(earliest, latest, service_time);
              return py::make_tuple(std::move(f.xs), std::move(f.ys));
          });

    // --- instance + route evaluation ---
    py::class_<kayros::Instance>(m, "Instance")
        .def(py::init(&make_instance), py::arg("num_customers"),
             py::arg("num_vehicles"), py::arg("vehicle_capacity"),
             py::arg("horizon"), py::arg("time_windows"), py::arg("demands"),
             py::arg("service_times"), py::arg("arcs"))
        .def_readonly("num_customers", &kayros::Instance::num_customers)
        .def_readonly("num_vehicles", &kayros::Instance::num_vehicles)
        .def_readonly("vehicle_capacity", &kayros::Instance::vehicle_capacity)
        .def_readonly("has_time_windows", &kayros::Instance::has_time_windows)
        .def("evaluate_route",
             [](const kayros::Instance& inst,
                const std::vector<std::int32_t>& route) {
                 const kayros::RouteEval r = kayros::evaluate_route(
                     inst, route.data(),
                     static_cast<std::int64_t>(route.size()));
                 return py::make_tuple(r.feasible, r.duration, r.departure);
             })
        .def("route_ready_time_function",
             [](const kayros::Instance& inst,
                const std::vector<std::int32_t>& route) {
                 kayros::Pwlf d = kayros::route_ready_time_function(
                     inst, route.data(),
                     static_cast<std::int64_t>(route.size()));
                 return py::make_tuple(std::move(d.xs), std::move(d.ys));
             });

    // --- heuristics ---
    py::class_<kayros::AcoParams>(m, "AcoParams")
        .def(py::init<>())
        .def_readwrite("max_iterations", &kayros::AcoParams::max_iterations)
        .def_readwrite("max_no_improvement", &kayros::AcoParams::max_no_improvement)
        .def_readwrite("nb_ants", &kayros::AcoParams::nb_ants)
        .def_readwrite("alpha", &kayros::AcoParams::alpha)
        .def_readwrite("beta", &kayros::AcoParams::beta)
        .def_readwrite("rho", &kayros::AcoParams::rho)
        .def_readwrite("tau_min", &kayros::AcoParams::tau_min)
        .def_readwrite("tau_0", &kayros::AcoParams::tau_0)
        .def_readwrite("tau_max", &kayros::AcoParams::tau_max)
        .def_readwrite("delta_pheromone_threshold",
                       &kayros::AcoParams::delta_pheromone_threshold)
        .def_readwrite("use_local_search", &kayros::AcoParams::use_local_search)
        .def_readwrite("ls_all_ants", &kayros::AcoParams::ls_all_ants)
        .def_readwrite("num_neighbours", &kayros::AcoParams::num_neighbours)
        .def_readwrite("weight_wait", &kayros::AcoParams::weight_wait);

    py::class_<kayros::Incumbent>(m, "Incumbent")
        .def_readonly("value", &kayros::Incumbent::value)
        .def_readonly("seconds", &kayros::Incumbent::seconds)
        .def_readonly("iteration", &kayros::Incumbent::iteration)
        .def_readonly("origin", &kayros::Incumbent::origin);

    py::enum_<kayros::SolveStatus>(m, "SolveStatus")
        .value("Finished", kayros::SolveStatus::Finished)
        .value("Converged", kayros::SolveStatus::Converged)
        .value("TimeLimit", kayros::SolveStatus::TimeLimit)
        .value("Infeasible", kayros::SolveStatus::Infeasible);

    py::class_<kayros::SolveResult>(m, "SolveResult")
        .def_readonly("routes", &kayros::SolveResult::routes)
        .def_readonly("value", &kayros::SolveResult::value)
        .def_readonly("incumbents", &kayros::SolveResult::incumbents)
        .def_readonly("status", &kayros::SolveResult::status)
        .def_readonly("iterations_run", &kayros::SolveResult::iterations_run);

    m.def("greedy_makespan", [](const kayros::Instance& inst) {
        std::vector<std::vector<std::int32_t>> routes;
        const bool ok = kayros::greedy_makespan(inst, routes);
        return py::make_tuple(ok, std::move(routes));
    });
    m.def("solution_duration", &kayros::solution_duration);
    // The callback caster re-acquires the GIL for each invocation, so the
    // solve loop itself can keep running with the GIL released.
    m.def("solve_aco", &kayros::solve_aco, py::arg("instance"),
          py::arg("params"), py::arg("seed"), py::arg("time_limit_seconds"),
          py::arg("on_incumbent") = kayros::IncumbentCallback{},
          py::call_guard<py::gil_scoped_release>());

    // --- TD-LS layer (M3.7): exposed for the gate tests and experiments ---
    m.def(
        "ls_local_search",
        [](const kayros::Instance& inst,
           std::vector<std::vector<std::int32_t>> routes,
           std::int32_t num_neighbours, double weight_wait) {
            kayros::LsStats stats;
            const kayros::NeighbourLists nb =
                kayros::build_neighbour_lists(inst, num_neighbours, weight_wait);
            const double value = kayros::local_search(inst, nb, routes, &stats);
            return py::make_tuple(std::move(routes), value, stats.applied,
                                  stats.reverted);
        },
        py::arg("instance"), py::arg("routes"), py::arg("num_neighbours") = 0,
        py::arg("weight_wait") = 0.2);
    m.def(
        "ls_neighbour_lists",
        [](const kayros::Instance& inst, std::int32_t num_neighbours,
           double weight_wait) {
            const kayros::NeighbourLists nb =
                kayros::build_neighbour_lists(inst, num_neighbours, weight_wait);
            std::vector<std::vector<std::int32_t>> lists(
                static_cast<std::size_t>(inst.num_customers) + 1);
            if (nb.restricted()) {
                for (std::int32_t i = 1; i <= inst.num_customers; ++i) {
                    lists[static_cast<std::size_t>(i)].assign(
                        nb.neighbours_begin(i), nb.neighbours_end(i));
                }
            }
            return lists;
        },
        py::arg("instance"), py::arg("num_neighbours"),
        py::arg("weight_wait") = 0.2);
    m.def(
        "ls_evaluate_splice",
        [](const kayros::Instance& inst, const std::vector<std::int32_t>& route1,
           std::int64_t i1, std::int64_t j1,
           const std::vector<std::int32_t>& route2, std::int64_t i2,
           std::int64_t j2) {
            kayros::RouteState r1, r2;
            if (!kayros::build_route_state(inst, route1, r1)) {
                throw std::invalid_argument("route1 is infeasible");
            }
            if (!kayros::build_route_state(inst, route2, r2)) {
                throw std::invalid_argument("route2 is infeasible");
            }
            const kayros::RouteEval e =
                kayros::evaluate_splice(inst, r1, i1, j1, r2, i2, j2);
            return py::make_tuple(e.feasible, e.duration, e.departure);
        },
        py::arg("instance"), py::arg("route1"), py::arg("i1"), py::arg("j1"),
        py::arg("route2"), py::arg("i2"), py::arg("j2"));
    m.def(
        "ls_evaluate_intra_relocate",
        [](const kayros::Instance& inst, const std::vector<std::int32_t>& route,
           std::int64_t i, std::int64_t p) {
            kayros::RouteState r;
            if (!kayros::build_route_state(inst, route, r)) {
                throw std::invalid_argument("route is infeasible");
            }
            const kayros::RouteEval e =
                kayros::evaluate_intra_relocate(inst, r, i, p);
            return py::make_tuple(e.feasible, e.duration, e.departure);
        },
        py::arg("instance"), py::arg("route"), py::arg("i"), py::arg("p"));
}
