"""The kayros solve entry point (stage 1: greedy seed + TD-ACO)."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

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
    """TD-ACO parameters (defaults: the original tuned bp_heur values; a
    re-tuning sweep on MAMUT-format instances is scheduled at M3.5)."""

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
        return params


@dataclass
class Incumbent:
    value: float
    seconds: float
    iteration: int
    origin: str  # "greedy" or "aco"


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
) -> Solution:
    """Solve a MAMUT TD instance (TDVRPTW or TDVRP, Duration minimization).

    ``instance`` is a ``.vrp.json`` path or an already-loaded
    ``LoadedTDInstance``. The returned ``Solution.duration`` is priced by the
    reference checker; an internal/checker disagreement raises (it would be a
    kayros bug, never a rounding issue to tolerate).
    """
    loaded = instance if isinstance(instance, LoadedTDInstance) else load_instance(instance)
    core = to_core(loaded)
    result = _core.solve_aco(
        core,
        (params or Params())._to_core(),
        seed,
        0.0 if time_limit is None else float(time_limit),
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
        incumbents=[
            Incumbent(i.value, i.seconds, i.iteration, "greedy" if i.origin == 0 else "aco")
            for i in result.incumbents
        ],
    )
