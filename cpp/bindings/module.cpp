#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <chrono>
#include <optional>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include <random>

#include "core/instance.h"
#include "core/warp_eval.h"
#include "heuristics/heuristics.h"
#include "heuristics/warp_ils.h"
#include "ls/ls.h"
#include "ls/perturb.h"
#include "ls/warp_ls.h"
#include "pwlf/pwlf.h"
#include "pwlf/warp.h"

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

    // --- TD time-warp primitives (Stream 8; exposed for the warp gate suite) ---
    m.def("pwlf_add", [](std::vector<double> f_xs, std::vector<double> f_ys,
                         std::vector<double> g_xs, std::vector<double> g_ys) {
        const kayros::Pwlf f = make_pwlf(std::move(f_xs), std::move(f_ys));
        const kayros::Pwlf g = make_pwlf(std::move(g_xs), std::move(g_ys));
        kayros::Pwlf h = kayros::add(kayros::view(f), kayros::view(g));
        return py::make_tuple(std::move(h.xs), std::move(h.ys));
    });
    m.def("pwlf_make_theta_warp",
          [](double earliest, double latest, double service_time, double t_end) {
              kayros::ThetaWarp tw =
                  kayros::make_theta_warp(earliest, latest, service_time, t_end);
              return py::make_tuple(
                  py::make_tuple(std::move(tw.theta.xs), std::move(tw.theta.ys)),
                  py::make_tuple(std::move(tw.warp.xs), std::move(tw.warp.ys)));
          });
    m.def("pwlf_make_return_clamp", [](double due, double t_end) {
        kayros::Pwlf f = kayros::make_return_clamp(due, t_end);
        return py::make_tuple(std::move(f.xs), std::move(f.ys));
    });
    m.def("pwlf_make_return_warp", [](double due, double t_end) {
        kayros::Pwlf f = kayros::make_return_warp(due, t_end);
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
             })
        .def("warp_horizon",
             [](const kayros::Instance& inst) { return kayros::warp_horizon(inst); })
        .def("route_warp_functions",
             [](const kayros::Instance& inst,
                const std::vector<std::int32_t>& route, double t_end) {
                 kayros::WarpFunctions wf = kayros::warp_route_functions(
                     inst, route.data(),
                     static_cast<std::int64_t>(route.size()), t_end);
                 return py::make_tuple(
                     py::make_tuple(std::move(wf.rho.xs), std::move(wf.rho.ys)),
                     py::make_tuple(std::move(wf.warp.xs), std::move(wf.warp.ys)));
             },
             py::arg("route"), py::arg("t_end"))
        .def("evaluate_route_warp",
             [](const kayros::Instance& inst,
                const std::vector<std::int32_t>& route, double penalty,
                double t_end) {
                 const kayros::WarpRouteEval r = kayros::evaluate_route_warp(
                     inst, route.data(),
                     static_cast<std::int64_t>(route.size()), penalty, t_end);
                 py::dict d;
                 d["total"] = r.total;
                 d["feasible"] = r.feasible;
                 d["duration"] = r.duration;
                 d["departure"] = r.departure;
                 d["min_warp"] = r.min_warp;
                 d["penalised"] = r.penalised;
                 d["penalised_departure"] = r.penalised_departure;
                 return d;
             },
             py::arg("route"), py::arg("penalty"), py::arg("t_end"));

    // --- warp-augmented LS structures (Stream 8, P8.2; gate + bench surface) ---
    py::class_<kayros::WarpRouteState>(m, "WarpRouteState")
        .def_readonly("duration", &kayros::WarpRouteState::duration)
        .def_readonly("min_warp", &kayros::WarpRouteState::min_warp)
        .def_readonly("load", &kayros::WarpRouteState::load);
    m.def(
        "build_warp_route_state",
        [](const kayros::Instance& inst, std::vector<std::int32_t> vertices,
           double penalty, double t_end) -> std::optional<kayros::WarpRouteState> {
            kayros::WarpRouteState state;
            if (!kayros::build_warp_route_state(inst, std::move(vertices), penalty,
                                                t_end, state)) {
                return std::nullopt;
            }
            return state;
        },
        py::arg("instance"), py::arg("vertices"), py::arg("penalty"),
        py::arg("t_end"));
    m.def(
        "evaluate_splice_warp",
        [](const kayros::Instance& inst, const kayros::WarpRouteState& r1,
           std::int64_t i1, std::int64_t j1, const kayros::WarpRouteState& r2,
           std::int64_t i2, std::int64_t j2, double penalty, double t_end) {
            const kayros::WarpRouteEval r = kayros::evaluate_splice_warp(
                inst, r1, i1, j1, r2, i2, j2, penalty, t_end);
            py::dict d;
            d["total"] = r.total;
            d["feasible"] = r.feasible;
            d["duration"] = r.duration;
            d["min_warp"] = r.min_warp;
            d["penalised"] = r.penalised;
            return d;
        },
        py::arg("instance"), py::arg("r1"), py::arg("i1"), py::arg("j1"),
        py::arg("r2"), py::arg("i2"), py::arg("j2"), py::arg("penalty"),
        py::arg("t_end"));
    m.def(
        "warp_tree_full",
        [](const kayros::Instance& inst, const std::vector<std::int32_t>& route,
           double t_end) {
            kayros::WarpLcaTree tree;
            tree.build(kayros::warp_route_leaves(
                inst, route.data(), static_cast<std::int64_t>(route.size()), t_end));
            kayros::WarpSegment s = tree.query(0, tree.num_leaves() - 1);
            return py::make_tuple(
                py::make_tuple(std::move(s.rho.xs), std::move(s.rho.ys)),
                py::make_tuple(std::move(s.omega.xs), std::move(s.omega.ys)));
        },
        py::arg("instance"), py::arg("route"), py::arg("t_end"));
    m.def(
        "warp_tree_update_gate",
        [](const kayros::Instance& inst, const std::vector<std::int32_t>& route,
           double t_end, std::int64_t position, std::int32_t new_customer) {
            // Localized update_leaf ≡ fresh rebuild, bitwise, on every query
            // range (the td-route-trees P4.3 gate transposed to segments).
            const std::int64_t m = static_cast<std::int64_t>(route.size());
            if (position < 0 || position >= m) {
                throw std::invalid_argument("bad position");
            }
            std::vector<std::int32_t> modified(route);
            modified[static_cast<std::size_t>(position)] = new_customer;

            kayros::WarpLcaTree updated;
            updated.build(kayros::warp_route_leaves(inst, route.data(), m, t_end));
            std::vector<kayros::WarpSegment> fresh_leaves =
                kayros::warp_route_leaves(inst, modified.data(), m, t_end);
            // Changing r[position] touches leaf `position` (incoming arc +
            // theta) and leaf `position + 1` (outgoing arc; the return leaf
            // when position == m - 1).
            updated.update_leaf(position,
                                fresh_leaves[static_cast<std::size_t>(position)]);
            updated.update_leaf(position + 1,
                                fresh_leaves[static_cast<std::size_t>(position + 1)]);

            kayros::WarpLcaTree rebuilt;
            rebuilt.build(std::move(fresh_leaves));

            for (std::int64_t lo = 0; lo < rebuilt.num_leaves(); ++lo) {
                for (std::int64_t hi = lo; hi < rebuilt.num_leaves(); ++hi) {
                    const kayros::WarpSegment a = updated.query(lo, hi);
                    const kayros::WarpSegment b = rebuilt.query(lo, hi);
                    if (a.rho.xs != b.rho.xs || a.rho.ys != b.rho.ys ||
                        a.omega.xs != b.omega.xs || a.omega.ys != b.omega.ys) {
                        return false;
                    }
                }
            }
            return true;
        },
        py::arg("instance"), py::arg("route"), py::arg("t_end"),
        py::arg("position"), py::arg("new_customer"));
    m.def(
        "bench_splice_pair",
        [](const kayros::Instance& inst,
           const std::vector<std::vector<std::int32_t>>& routes,
           const std::vector<std::tuple<std::int64_t, std::int64_t, std::int64_t,
                                        std::int64_t, std::int64_t, std::int64_t>>&
               draws,
           double penalty, double t_end) {
            // Fair per-move cost: BOTH sides run over prebuilt trees, timed in
            // C++ (P8.2 microbench; bench-only surface like the gates above).
            std::vector<kayros::RouteState> base(routes.size());
            std::vector<kayros::WarpRouteState> warped(routes.size());
            for (std::size_t r = 0; r < routes.size(); ++r) {
                if (!kayros::build_route_state(inst, routes[r], base[r])) {
                    throw std::invalid_argument("bench route infeasible");
                }
                if (!kayros::build_warp_route_state(inst, routes[r], penalty, t_end,
                                                    warped[r])) {
                    throw std::invalid_argument("bench route hits a hard wall");
                }
            }
            double base_sum = 0.0, warp_sum = 0.0;
            const auto t0 = std::chrono::steady_clock::now();
            for (const auto& d : draws) {
                const kayros::RouteEval e = kayros::evaluate_splice(
                    inst, base[static_cast<std::size_t>(std::get<0>(d))],
                    std::get<1>(d), std::get<2>(d),
                    base[static_cast<std::size_t>(std::get<3>(d))], std::get<4>(d),
                    std::get<5>(d));
                if (e.feasible) base_sum += e.duration;
            }
            const auto t1 = std::chrono::steady_clock::now();
            for (const auto& d : draws) {
                const kayros::WarpRouteEval e = kayros::evaluate_splice_warp(
                    inst, warped[static_cast<std::size_t>(std::get<0>(d))],
                    std::get<1>(d), std::get<2>(d),
                    warped[static_cast<std::size_t>(std::get<3>(d))], std::get<4>(d),
                    std::get<5>(d), penalty, t_end);
                if (e.total) warp_sum += e.penalised;
            }
            const auto t2 = std::chrono::steady_clock::now();
            const double base_s = std::chrono::duration<double>(t1 - t0).count();
            const double warp_s = std::chrono::duration<double>(t2 - t1).count();
            return py::make_tuple(base_s, warp_s, base_sum, warp_sum);
        },
        py::arg("instance"), py::arg("routes"), py::arg("draws"),
        py::arg("penalty"), py::arg("t_end"));

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

    py::class_<kayros::IlsParams>(m, "IlsParams")
        .def(py::init<>())
        .def_readwrite("max_iterations", &kayros::IlsParams::max_iterations)
        .def_readwrite("num_neighbours", &kayros::IlsParams::num_neighbours)
        .def_readwrite("weight_wait", &kayros::IlsParams::weight_wait)
        .def_readwrite("min_perturbations",
                       &kayros::IlsParams::min_perturbations)
        .def_readwrite("max_perturbations",
                       &kayros::IlsParams::max_perturbations)
        .def_readwrite("history_length", &kayros::IlsParams::history_length)
        .def_readwrite("restart_no_improvement",
                       &kayros::IlsParams::restart_no_improvement)
        .def_readwrite("exhaustive_on_best",
                       &kayros::IlsParams::exhaustive_on_best);

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
    m.def("solve_ils", &kayros::solve_ils, py::arg("instance"),
          py::arg("params"), py::arg("seed"), py::arg("time_limit_seconds"),
          py::arg("on_incumbent") = kayros::IncumbentCallback{},
          py::arg("initial_routes") = std::vector<std::vector<std::int32_t>>{},
          py::call_guard<py::gil_scoped_release>());

    // --- Stream 8 P8.3-minimal penalised ILS (td-time-warp branch only) ---
    py::class_<kayros::WarpIlsParams>(m, "WarpIlsParams")
        .def(py::init<>())
        .def_readwrite("max_iterations", &kayros::WarpIlsParams::max_iterations)
        .def_readwrite("num_neighbours", &kayros::WarpIlsParams::num_neighbours)
        .def_readwrite("weight_wait", &kayros::WarpIlsParams::weight_wait)
        .def_readwrite("min_perturbations",
                       &kayros::WarpIlsParams::min_perturbations)
        .def_readwrite("max_perturbations",
                       &kayros::WarpIlsParams::max_perturbations)
        .def_readwrite("history_length", &kayros::WarpIlsParams::history_length)
        .def_readwrite("restart_no_improvement",
                       &kayros::WarpIlsParams::restart_no_improvement)
        .def_readwrite("penalty_init", &kayros::WarpIlsParams::penalty_init)
        .def_readwrite("penalty_increase",
                       &kayros::WarpIlsParams::penalty_increase)
        .def_readwrite("penalty_decrease",
                       &kayros::WarpIlsParams::penalty_decrease)
        .def_readwrite("target_feasible", &kayros::WarpIlsParams::target_feasible)
        .def_readwrite("penalty_window", &kayros::WarpIlsParams::penalty_window)
        .def_readwrite("penalty_min", &kayros::WarpIlsParams::penalty_min)
        .def_readwrite("penalty_max", &kayros::WarpIlsParams::penalty_max);
    m.def("solve_warp_ils", &kayros::solve_warp_ils, py::arg("instance"),
          py::arg("params"), py::arg("seed"), py::arg("time_limit_seconds"),
          py::arg("initial_routes") = std::vector<std::vector<std::int32_t>>{},
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
        "ls_perturb",
        [](const kayros::Instance& inst,
           std::vector<std::vector<std::int32_t>> routes, std::uint64_t seed,
           std::int32_t min_removals, std::int32_t max_removals,
           std::int32_t num_neighbours, double weight_wait) {
            const kayros::NeighbourLists nb =
                kayros::build_neighbour_lists(inst, num_neighbours, weight_wait);
            kayros::SearchState ss;
            if (!kayros::init_search_state(inst, routes, ss)) {
                throw std::invalid_argument("input solution is infeasible");
            }
            std::mt19937_64 rng(seed);
            kayros::PerturbParams params;
            params.min_removals = min_removals;
            params.max_removals = max_removals;
            const kayros::PerturbOutcome outcome =
                kayros::perturb(inst, nb, ss, rng, params);
            routes.clear();
            routes.reserve(ss.states.size());
            for (kayros::RouteState& s : ss.states) {
                routes.push_back(std::move(s.vertices));
            }
            const double value = kayros::solution_duration(inst, routes);
            return py::make_tuple(std::move(routes), value, outcome.applied,
                                  outcome.removed, outcome.redraws,
                                  outcome.new_routes);
        },
        py::arg("instance"), py::arg("routes"), py::arg("seed"),
        py::arg("min_removals") = 1, py::arg("max_removals") = 25,
        py::arg("num_neighbours") = 50, py::arg("weight_wait") = 0.2);
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
