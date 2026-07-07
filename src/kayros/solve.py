"""The kayros solve entry point (stage 1: greedy seed + TD-ACO)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

from mamut_routing_lib.models import BenchmarkSolution
from mamut_routing_lib.td import LoadedTDInstance, check_td_solution

from kayros import _core
from kayros.io import load_instance, to_core


class KayrosError(RuntimeError):
    pass


class InfeasibleError(KayrosError):
    """No feasible solution could be constructed."""


@dataclass
class Params:
    """Solver parameters. ``strategy`` picks the search: ``"aco"`` (TD-ACO,
    the historical default — M7.4's head-to-head campaign will re-pick the
    default from data), ``"ils"`` (single-trajectory TD-ILS: granular VND +
    ruin-and-recreate + late-acceptance, Stream 7), or ``"aco+ils"`` (an
    experimental budget split: ACO for ``aco_budget_fraction`` of the time
    limit, then ILS warm-started from the ACO best).

    ACO parameter defaults are the original tuned bp_heur values; ILS
    defaults follow PyVRP v0.14 (restart threshold scaled to kayros's
    ms-class iterations). ``num_neighbours``/``weight_wait`` (granular
    candidate lists, M7.0) apply to the local search of EVERY strategy —
    default-on since 0.4.0; set ``num_neighbours=0`` for the pre-0.4.0
    exhaustive scans."""

    strategy: str = "aco"
    max_iterations: int = 3000
    max_no_improvement: int = 20
    nb_ants: int = 8
    alpha: int = 15
    beta: int = 10
    rho: float = 0.02
    tau_min: float = 1e-6
    tau_0: float = 2.0
    tau_max: float = 10.0
    delta_pheromone_threshold: float = 1e-4
    # TD local search (M3.7): first-improvement descent (inter/intra relocate,
    # swap, 2-opt*) on the greedy seed and on each iteration's best ant, with
    # LCA-BST ranked move evaluation and checker-fold repriced commits.
    local_search: bool = True
    # LS scope (M3.5.4 round 2): apply TD-LS to every feasible ant instead of
    # the iteration-best only. More LS work per iteration, denser deposits.
    ls_all_ants: bool = False
    # Granular candidate lists (M7.0): per client, the num_neighbours nearest
    # others under a TD adaptation of the Vidal (2013) proximity (min ATF
    # travel duration + weight_wait * inevitable wait), restricting the LS
    # move enumeration. Default-on since 0.4.0 — a deliberate behavior change
    # vs 0.3.0's exhaustive scans; set num_neighbours=0 to restore them.
    num_neighbours: int = 50
    weight_wait: float = 0.2
    # TD-ILS (M7.1/M7.2): perturbation magnitude, LAHC history, restart-to-
    # best threshold, exhaustive polish on new global bests. ils_max_iterations
    # 0 means unbounded (the time limit is then the stopping criterion).
    min_perturbations: int = 1
    max_perturbations: int = 25
    lahc_history: int = 300
    restart_no_improvement: int = 20_000
    exhaustive_on_best: bool = True
    ils_max_iterations: int = 0
    # "aco+ils": fraction of the time limit given to the ACO phase.
    aco_budget_fraction: float = 0.5

    def _to_core(self) -> _core.AcoParams:
        params = _core.AcoParams()
        params.max_iterations = self.max_iterations
        params.max_no_improvement = self.max_no_improvement
        params.nb_ants = self.nb_ants
        params.alpha = self.alpha
        params.beta = self.beta
        params.rho = self.rho
        params.tau_min = self.tau_min
        params.tau_0 = self.tau_0
        params.tau_max = self.tau_max
        params.delta_pheromone_threshold = self.delta_pheromone_threshold
        params.use_local_search = self.local_search
        params.ls_all_ants = self.ls_all_ants
        params.num_neighbours = self.num_neighbours
        params.weight_wait = self.weight_wait
        return params

    def _to_ils_core(self) -> _core.IlsParams:
        params = _core.IlsParams()
        if self.ils_max_iterations > 0:
            params.max_iterations = self.ils_max_iterations
        params.num_neighbours = self.num_neighbours
        params.weight_wait = self.weight_wait
        params.min_perturbations = self.min_perturbations
        params.max_perturbations = self.max_perturbations
        params.history_length = self.lahc_history
        params.restart_no_improvement = self.restart_no_improvement
        params.exhaustive_on_best = self.exhaustive_on_best
        return params


@dataclass
class Incumbent:
    value: float
    seconds: float
    iteration: int
    origin: str  # "greedy" | "aco" | "ils"


_ORIGIN_NAMES = {0: "greedy", 1: "aco", 2: "ils"}


# Anytime hook: called synchronously from inside the solve loop on every new
# incumbent, with the incumbent record and its routes (customer ids, no depot).
IncumbentHook = Callable[[Incumbent, list[list[int]]], None]


@dataclass
class Solution:
    """A checker-validated solution: ``duration`` is always the value computed
    by ``mamut_routing_lib.td.check_td_solution`` (the reference objective)."""

    instance_name: str
    routes: list[list[int]]
    duration: float
    route_durations: list[float]
    route_departures: list[float]
    status: str  # "finished" | "converged" | "time_limit"
    iterations: int
    incumbents: list[Incumbent] = field(default_factory=list)

    @property
    def num_routes(self) -> int:
        return len(self.routes)

    def to_benchmark_solution(self) -> BenchmarkSolution:
        """MAMUT solution artifact (feeds the BKS pipeline)."""
        return BenchmarkSolution(
            instance_name=self.instance_name,
            routes=self.routes,
            cost=self.duration,
            metadata={
                "solver": "kayros",
                "route_durations": self.route_durations,
                "route_departure_times": self.route_departures,
            },
        )


_STATUS_NAMES = {
    _core.SolveStatus.Finished: "finished",
    _core.SolveStatus.Converged: "converged",
    _core.SolveStatus.TimeLimit: "time_limit",
}


def solve(
    instance: str | Path | LoadedTDInstance,
    params: Params | None = None,
    *,
    time_limit: float | None = None,
    seed: int = 0,
    on_incumbent: IncumbentHook | None = None,
) -> Solution:
    """Solve a MAMUT TD instance (TDVRPTW or TDVRP, Duration minimization).

    ``instance`` is a ``.vrp.json`` path or an already-loaded
    ``LoadedTDInstance``. The returned ``Solution.duration`` is priced by the
    reference checker; an internal/checker disagreement raises (it would be a
    kayros bug, never a rounding issue to tolerate).

    ``on_incumbent`` makes the solve anytime: it fires synchronously on every
    new incumbent (the greedy seed included) with the ``Incumbent`` record and
    the routes, so callers can checkpoint solutions while the colony keeps
    running. Keep the hook cheap — the solve loop blocks on it; an exception
    raised inside it aborts the solve and propagates.
    """
    loaded = instance if isinstance(instance, LoadedTDInstance) else load_instance(instance)
    core = to_core(loaded)
    params = params or Params()
    tl = 0.0 if time_limit is None else float(time_limit)

    if params.strategy not in ("aco", "ils", "aco+ils"):
        raise ValueError(f"unknown strategy {params.strategy!r}")
    if params.strategy == "ils" and tl <= 0.0 and params.ils_max_iterations <= 0:
        raise ValueError(
            "strategy='ils' needs a time_limit or ils_max_iterations > 0 "
            "(the ILS loop has no convergence stop)"
        )
    if params.strategy == "aco+ils" and tl <= 0.0:
        raise ValueError("strategy='aco+ils' needs a time_limit to split")

    def make_hook(offset_seconds: float = 0.0, below: float = float("inf")):
        if on_incumbent is None:
            return None

        def hook(inc, routes):  # thin _core -> API adapter
            if inc.value >= below:  # warm-start seed re-fires the phase-1 best
                return
            on_incumbent(
                Incumbent(inc.value, inc.seconds + offset_seconds,
                          inc.iteration, _ORIGIN_NAMES[inc.origin]),
                [list(route) for route in routes],
            )

        return hook

    incumbent_offset = 0.0
    if params.strategy == "aco":
        result = _core.solve_aco(core, params._to_core(), seed, tl, make_hook())
        extra_incumbents: list[Incumbent] = []
    elif params.strategy == "ils":
        result = _core.solve_ils(core, params._to_ils_core(), seed, tl,
                                 make_hook())
        extra_incumbents = []
    else:  # "aco+ils": ACO phase, then ILS warm-started from the ACO best.
        import time as _time

        aco_tl = tl * params.aco_budget_fraction
        t0 = _time.perf_counter()
        phase1 = _core.solve_aco(core, params._to_core(), seed, aco_tl,
                                 make_hook())
        incumbent_offset = _time.perf_counter() - t0
        if phase1.status == _core.SolveStatus.Infeasible or not phase1.routes:
            raise InfeasibleError(
                f"kayros could not construct a feasible solution for "
                f"{loaded.instance.instance_name}"
            )
        extra_incumbents = [
            Incumbent(i.value, i.seconds, i.iteration, _ORIGIN_NAMES[i.origin])
            for i in phase1.incumbents
        ]
        result = _core.solve_ils(
            core, params._to_ils_core(), seed, max(tl - incumbent_offset, 0.01),
            make_hook(incumbent_offset, below=phase1.value), phase1.routes,
        )
    if result.status == _core.SolveStatus.Infeasible or not result.routes:
        raise InfeasibleError(
            f"kayros could not construct a feasible solution for "
            f"{loaded.instance.instance_name}"
        )

    check = check_td_solution(
        loaded,
        BenchmarkSolution(
            instance_name=loaded.instance.instance_name,
            routes=[list(route) for route in result.routes],
            cost=result.value,
        ),
    )
    if not check.is_valid():
        raise KayrosError(
            f"internal solution rejected by the reference checker "
            f"({check.status}: {check.error_message}) — this is a kayros bug"
        )

    return Solution(
        instance_name=loaded.instance.instance_name,
        routes=[list(route) for route in result.routes],
        duration=check.routing_cost,
        route_durations=[e.duration for e in check.route_evaluations],
        route_departures=[e.departure_time for e in check.route_evaluations],
        status=_STATUS_NAMES[result.status],
        iterations=result.iterations_run,
        incumbents=extra_incumbents + [
            Incumbent(i.value, i.seconds + incumbent_offset, i.iteration,
                      _ORIGIN_NAMES[i.origin])
            for i in result.incumbents
            # In the split, the ILS warm-start seed re-fires the phase-1 best:
            # keep the merged stream strictly improving.
            if i.value < min((e.value for e in extra_incumbents),
                             default=float("inf"))
        ],
    )
