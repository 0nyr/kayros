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
        fixed_route_cost: float = 0.0,
    ) -> None: ...
    @property
    def fixed_route_cost(self) -> float: ...
    def evaluate_route(self, route: list[int]) -> tuple[bool, float, float]: ...
    def route_ready_time_function(
        self, route: list[int]
    ) -> tuple[list[float], list[float]]: ...

class ExactRoutePricer:
    def __init__(self, instance: Instance, cache_knots: int = 65536) -> None: ...
    def route_timing(
        self, route: list[int], /
    ) -> tuple[tuple[int, int], tuple[int, int]] | None: ...
    def solution_cost(self, routes: list[list[int]], /) -> tuple[int, int] | None: ...
    @property
    def cached_knots(self) -> int: ...
    @property
    def prepared_arcs(self) -> int: ...

class IlsParams:
    ils_max_iterations: int
    num_neighbours: int
    weight_wait: float
    min_perturbations: int
    max_perturbations: int
    dissolve_pct: int
    lahc_history: int
    restart_no_improvement: int
    exhaustive_on_best: bool
    fd_attempts: int
    fd_k_max: int
    fd_ep_budget: int
    fd_work_cap: int
    sq_work_cap: int
    sq_penalty: float
    sq_on_nodrop: bool
    sq_ladder: bool
    k_cap: int
    fd_period_work: int
    seed_k_factor: float
    restart_no_improvement_work: int

class Incumbent:
    value: float
    seconds: float
    iteration: int
    origin: int
    from_squeeze: bool

class SolveStatus(enum.Enum):
    Finished = 0
    TimeLimit = 2
    Infeasible = 3

class FleetKickStats:
    kicks_total: int
    kicks_applied: int
    redraws_sum: int
    dissolved_armed: int
    dissolve_undone_in_kick: int
    normal_kicks: int
    k_after_kick_lt: int
    k_after_kick_eq: int
    k_after_kick_gt: int
    k_after_descent_lt: int
    k_after_descent_eq: int
    k_after_descent_gt: int
    dissolved_accepted_lahc: int
    normal_accepted_lahc: int
    dissolved_new_best: int
    normal_new_best: int
    dissolved_new_best_k_lt: int

class FdStats:
    triggers: int
    attempts: int
    successes: int
    pops: int
    step1_inserts: int
    ejections: int
    rollbacks_deadend: int
    rollbacks_budget: int
    rollbacks_time: int
    rollbacks_work: int
    evaluated: int
    basin_evaluated: int
    squeeze_phases: int
    squeeze_evaluated: int
    squeeze_checkpoints: int
    squeeze_improved: int
    ladder_squeezes: int
    ladder_rescues: int

class KStats:
    k_seed: int
    k_final: int
    k_best_min: int
    k_best_max: int
    singleton_opens: int
    kicks_opening: int
    k_up_after_kick: int
    k_down_after_kick: int
    k_up_after_descent: int
    k_down_after_descent: int
    accepted_k_up: int
    accepted_k_down: int
    new_best_k_up: int
    new_best_k_down: int

class KCapStats:
    seed_drain_attempts: int
    reached_cap: int
    rejected_above_cap: int
    first_capped_work: int

class SolveResult:
    routes: list[list[int]]
    value: float
    incumbents: list[Incumbent]
    status: SolveStatus
    iterations_run: int
    work_units: int
    restarts: int
    k_stats: KStats
    k_cap_stats: KCapStats
    fleet_stats: FleetKickStats
    fd_stats: FdStats

class ReferenceResult:
    routes: list[list[int]]
    exact_cost: tuple[int, int] | None
    status: SolveStatus
    iterations_run: int
    work_units: int
    restarts: int
    k_stats: KStats
    k_cap_stats: KCapStats
    fleet_stats: FleetKickStats
    fd_stats: FdStats

def greedy_construct(instance: Instance) -> tuple[bool, list[list[int]]]: ...
def greedy_construct_lookahead(instance: Instance) -> tuple[bool, list[list[int]]]: ...
def split_to_k(
    instance: Instance, routes: list[list[int]], target_k: int
) -> tuple[bool, list[list[int]]]: ...
def ls_local_search(
    instance: Instance,
    routes: list[list[int]],
    num_neighbours: int = 0,
    weight_wait: float = 0.2,
) -> tuple[list[list[int]], float, int, int]: ...
def ls_neighbour_lists(
    instance: Instance, num_neighbours: int, weight_wait: float = 0.2
) -> list[list[int]]: ...
def ls_perturb(
    instance: Instance,
    routes: list[list[int]],
    seed: int,
    min_perturbations: int = 1,
    max_perturbations: int = 25,
    num_neighbours: int = 50,
    weight_wait: float = 0.2,
    dissolve_pct: int = 50,
) -> tuple[list[list[int]], float, bool, int, int, int, bool]: ...
def ls_fleet_descent(
    instance: Instance, routes: list[list[int]], seed: int,
    fd_k_max: int = 2, fd_ep_budget: int = 2000, num_neighbours: int = 50,
    weight_wait: float = 0.2, fd_work_cap: int = 0,
) -> tuple[list[list[int]], float, bool, int, int, int, int, int, int, int]: ...
def ls_squeeze_phase(
    instance: Instance, routes: list[list[int]], sq_penalty: float = 10.0,
    sq_work_cap: int = 1000000, num_neighbours: int = 50, weight_wait: float = 0.2,
) -> tuple[list[list[int]], bool, int, int, int]: ...
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
def solution_cost(instance: Instance, routes: list[list[int]]) -> float: ...
def solve_ils(
    instance: Instance,
    params: IlsParams,
    seed: int,
    time_limit: float,
    on_incumbent: Callable[[Incumbent, list[list[int]]], None] | None = None,
    initial_routes: list[list[int]] = ...,
    accept_incumbent: Callable[[list[list[int]], bool], bool] | None = None,
    on_unusable_warm_start: Callable[[float, bool], None] | None = None,
    fleet_cost_objective: bool = False,
) -> SolveResult: ...

def solve_reference(
    instance: Instance,
    params: IlsParams,
    seed: int,
    time_limit: float,
    on_incumbent: Callable[[list[list[int]], tuple[int, int], float, int, int], None] | None = None,
    initial_routes: list[list[int]] = ...,
    fleet_cost_objective: bool = False,
) -> ReferenceResult: ...
