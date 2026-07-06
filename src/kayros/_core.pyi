"""Typing stubs for the compiled kayros core."""

import enum
from typing import Callable

__version__: str

def pwlf_identity(low: float, high: float) -> tuple[list[float], list[float]]: ...
def pwlf_evaluate(xs: list[float], ys: list[float], x: float) -> float: ...
def pwlf_compose(
    f_xs: list[float], f_ys: list[float], g_xs: list[float], g_ys: list[float]
) -> tuple[list[float], list[float]]: ...
def pwlf_min_shifted_image(xs: list[float], ys: list[float]) -> tuple[float, float]: ...
def pwlf_make_theta(
    earliest: float, latest: float, service_time: float
) -> tuple[list[float], list[float]]: ...

class Instance:
    num_customers: int
    num_vehicles: int
    vehicle_capacity: int
    has_time_windows: bool
    def __init__(
        self,
        num_customers: int,
        num_vehicles: int | None,
        vehicle_capacity: int,
        horizon: tuple[float, float],
        time_windows: list[tuple[float, float]] | None,
        demands: list[int],
        service_times: list[float],
        arcs: list[tuple[int, int, list[float], list[float]]],
    ) -> None: ...
    def evaluate_route(self, route: list[int]) -> tuple[bool, float, float]: ...
    def route_ready_time_function(
        self, route: list[int]
    ) -> tuple[list[float], list[float]]: ...

class AcoParams:
    max_iterations: int
    max_no_improvement: int
    nb_ants: int
    alpha: int
    beta: int
    rho: float
    tau_min: float
    tau_0: float
    tau_max: float
    delta_pheromone_threshold: float
    use_local_search: bool

class Incumbent:
    value: float
    seconds: float
    iteration: int
    origin: int

class SolveStatus(enum.Enum):
    Finished = 0
    Converged = 1
    TimeLimit = 2
    Infeasible = 3

class SolveResult:
    routes: list[list[int]]
    value: float
    incumbents: list[Incumbent]
    status: SolveStatus
    iterations_run: int

def greedy_makespan(instance: Instance) -> tuple[bool, list[list[int]]]: ...
def ls_local_search(
    instance: Instance, routes: list[list[int]]
) -> tuple[list[list[int]], float, int, int]: ...
def ls_evaluate_splice(
    instance: Instance,
    route1: list[int],
    i1: int,
    j1: int,
    route2: list[int],
    i2: int,
    j2: int,
) -> tuple[bool, float, float]: ...
def ls_evaluate_intra_relocate(
    instance: Instance, route: list[int], i: int, p: int
) -> tuple[bool, float, float]: ...
def solution_duration(instance: Instance, routes: list[list[int]]) -> float: ...
def solve_aco(
    instance: Instance,
    params: AcoParams,
    seed: int,
    time_limit_seconds: float,
    on_incumbent: Callable[[Incumbent, list[list[int]]], None] | None = None,
) -> SolveResult: ...
