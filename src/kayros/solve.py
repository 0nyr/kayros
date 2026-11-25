"""Time-dependent iterated local search."""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Sequence
from dataclasses import dataclass, field
from functools import cached_property
import logging
from fractions import Fraction
from pathlib import Path
from typing import Callable, Literal

from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.exact import prepare_exact
from mamut_routing_lib.exact_types import ExactRouteError, canonical_checked_routes
from mamut_routing_lib.models import BenchmarkSolution
from mamut_routing_lib.td import LoadedTDInstance

from kayros import _core
from kayros.io import canonical_objective, load_instance, to_core


class KayrosError(RuntimeError):
    pass


class InfeasibleError(KayrosError):
    """No feasible solution could be constructed."""


@dataclass
class Params:
    """Parameters for the TD iterated local search.

    The Duration objective minimizes route durations. FleetCostDuration also charges
    the instance's fixed cost for each used route. Set num_neighbours to zero for
    exhaustive local-search enumeration.
    """

    strategy: str = "ils"
    objective: str = "duration"
    # Candidate lists use minimum travel duration plus weighted unavoidable waiting.
    # Zero neighbours selects exhaustive local search.
    num_neighbours: int = 50
    weight_wait: float = 0.2
    # Minimum and maximum number of customers selected for removal in a perturbation.
    min_perturbations: int = 1
    max_perturbations: int = 25
    # Percentage of FleetCostDuration kicks that try dissolving a smallest route.
    dissolve_pct: int = 50
    lahc_history: int = 300
    # Restart from the incumbent after this many iterations without improvement.
    restart_no_improvement: int = 20_000
    exhaustive_on_best: bool = True
    # Fleet descent runs at restart and at its periodic work trigger for FleetCostDuration.
    # Zero attempts disables it. Each attempt limits the ejection window and pool pops.
    fd_attempts: int = 3
    fd_k_max: int = 2
    fd_ep_budget: int = 2000
    # Candidate pricings per fleet-descent trigger. Zero leaves the trigger uncapped.
    fd_work_cap: int = 6_500_000
    # Candidate pricings per post-drain squeeze phase. Zero disables the phase.
    sq_work_cap: int = 0
    # Duration penalty per unit of time warp during squeeze exploration.
    sq_penalty: float = 10.0
    # Also squeeze when the drain preserves the current route count.
    sq_on_nodrop: bool = True
    # Try a squeeze inside the ejection ladder after a feasible insertion fails.
    sq_ladder: bool = False
    # Positive caps require FleetCostDuration incumbents to use at most this many routes.
    k_cap: int = 0
    # Candidate evaluations between fleet-descent triggers. Zero disables periodic triggers.
    fd_period_work: int = 100_000_000
    # Candidate evaluations without improvement before restart. Zero disables this trigger.
    restart_no_improvement_work: int = 1_000_000_000
    # Under Duration, split the greedy seed toward this multiple of its route count.
    seed_k_factor: float = 2.0
    # Zero uses the time limit, or five iteration-based restart windows without a time limit.
    ils_max_iterations: int = 0
    # Check every proposed incumbent by default. Final-only checks at return.
    acceptance_mode: Literal["each_incumbent", "final_only"] = "each_incumbent"
    # Final-only keeps this many distinct prior route sets alongside the current one.
    incumbent_history_size: int = 6
    # Exact reference search evaluates feasible moves with the family profile.
    search_mode: Literal["binary64", "exact"] = "binary64"

    def _to_ils_core(self) -> _core.IlsParams:
        params = _core.IlsParams()
        if self.ils_max_iterations > 0:
            params.ils_max_iterations = self.ils_max_iterations
        params.num_neighbours = self.num_neighbours
        params.weight_wait = self.weight_wait
        params.min_perturbations = self.min_perturbations
        params.max_perturbations = self.max_perturbations
        params.dissolve_pct = self.dissolve_pct
        params.lahc_history = self.lahc_history
        params.restart_no_improvement = self.restart_no_improvement
        params.exhaustive_on_best = self.exhaustive_on_best
        params.fd_attempts = self.fd_attempts
        params.fd_k_max = self.fd_k_max
        params.fd_ep_budget = self.fd_ep_budget
        params.fd_work_cap = self.fd_work_cap
        params.sq_work_cap = self.sq_work_cap
        params.sq_penalty = self.sq_penalty
        params.sq_on_nodrop = self.sq_on_nodrop
        params.sq_ladder = self.sq_ladder
        params.k_cap = self.k_cap
        params.fd_period_work = self.fd_period_work
        params.restart_no_improvement_work = self.restart_no_improvement_work
        params.seed_k_factor = self.seed_k_factor
        return params


@dataclass
class Incumbent:
    value: float | None
    seconds: float
    iteration: int | None  # ILS iteration when available
    origin: str
    exact_cost: Fraction | None = None
    search_value: float | None = None


_ORIGIN_NAMES = {0: "greedy", 2: "ils", 3: "warm_start"}


# Anytime hook: called synchronously from inside the solve loop on every new
# incumbent, with the incumbent record and its routes (customer ids, no depot).
IncumbentHook = Callable[[Incumbent, list[list[int]]], None]


def _display_cost(cost: Fraction) -> float | None:
    try:
        return float(cost)
    except OverflowError:
        return None


class _Acceptance:
    """Price proposed incumbents with the shared GMP evaluator and check the returned routes through the Python library."""

    def __init__(self, loaded, objective, params, hook):
        self.loaded = loaded
        self.objective = ObjectiveFunction(objective)
        self.mode = params.acceptance_mode
        if self.mode not in ("each_incumbent", "final_only"):
            raise ValueError(f"unknown acceptance_mode {self.mode!r}")
        self.capacity = params.incumbent_history_size
        if type(self.capacity) is not int or self.capacity < 0:
            raise ValueError("incumbent_history_size must be a nonnegative integer")
        self.hook = hook
        self.best_exact = None
        self.verified_seed = None
        self.checked_fallback = None
        self.events = []
        self.history = OrderedDict()
        self.stats = dict(proposals=0, infeasible=0, nonimproving=0, squeeze_rejections=0,
                          final_rejections=0, fallback_attempts=0, recovered=0)

    @cached_property
    def prepared(self):
        return prepare_exact(self.loaded, objective_function=self.objective)

    @cached_property
    def core(self):
        return to_core(self.loaded, objective_function=self.objective)

    @cached_property
    def native_pricer(self):
        return _core.ExactRoutePricer(self.core)

    def proposal_cost(self, routes):
        snapshot = self.prepared
        ordered = canonical_checked_routes(routes, num_customers=snapshot.num_customers, demands=snapshot.demands,
                                           capacity=snapshot.capacity, fleet_limit=snapshot.fleet_limit)
        cost = self.native_pricer.solution_cost(ordered)
        return None if cost is None else Fraction(*cost)

    def validate_seed(self, routes):
        try:
            evaluation = self.prepared.evaluate(routes)
        except ExactRouteError as error:
            raise ValueError(f"invalid initial_routes ({error.status.value}): {error}") from error
        # Preserve the caller's route enumeration, which affects the order of local search.
        seed = [list(route) for route in routes]
        self.verified_seed = (seed, evaluation.cost, evaluation.display_cost)
        return seed

    def unusable_warm_start(self, seconds, eligible):
        routes, cost, display = self.verified_seed
        self.verified_seed = None
        if not eligible:
            return
        self.stats["proposals"] += 1
        self.checked_fallback = (tuple(tuple(route) for route in routes), cost)
        if self.mode == "each_incumbent":
            self.best_exact = cost
        event = Incumbent(display, seconds, 0, "warm_start", cost, None)
        self.events.append(event)
        if self.hook:
            self.hook(event, [list(route) for route in routes])

    def accept(self, routes, from_squeeze):
        self.stats["proposals"] += 1
        seed, self.verified_seed = self.verified_seed, None
        if self.mode == "final_only":
            return True
        try:
            cost = seed[1] if seed is not None and routes == seed[0] else self.proposal_cost(routes)
        except ExactRouteError:
            cost = None
        if cost is None:
            self.stats["infeasible"] += 1
        else:
            if self.best_exact is None or cost < self.best_exact:
                self.best_exact = cost
                return True
            self.stats["nonimproving"] += 1
        self.stats["squeeze_rejections"] += int(from_squeeze)
        return False

    def publish(self, inc, routes):
        exact = self.best_exact
        display = _display_cost(exact) if exact is not None else inc.value
        event = Incumbent(display, inc.seconds, inc.iteration, _ORIGIN_NAMES[inc.origin], exact, inc.value)
        self.events.append(event)
        if self.mode == "final_only" and self.capacity:
            snapshot = tuple(tuple(route) for route in routes)
            key = tuple(sorted(snapshot))  # route enumeration does not distinguish solutions
            self.history[key] = (inc.value, snapshot, inc.from_squeeze)
            self.history.move_to_end(key)
            if len(self.history) > self.capacity + 1:
                self.history.popitem(last=False)
        if self.hook:
            self.hook(event, routes)

    def publish_reference(self, routes, cost_pair, seconds, iteration, origin):
        cost = Fraction(*cost_pair)
        if self.best_exact is not None and cost >= self.best_exact:
            raise KayrosError("exact reference search published a nonimproving incumbent")
        self.best_exact = cost
        self.stats["proposals"] += 1
        event = Incumbent(_display_cost(cost), seconds, iteration, _ORIGIN_NAMES[origin], cost, None)
        self.events.append(event)
        if self.hook:
            self.hook(event, routes)

    def finish_reference(self, result):
        try:
            evaluation = self.prepared.evaluate(result.routes)
        except ExactRouteError as error:
            raise KayrosError("final exact evaluation rejected the reference search incumbent") from error
        if (result.exact_cost is None or evaluation.cost != Fraction(*result.exact_cost)
                or evaluation.cost != self.best_exact):
            raise KayrosError("final exact evaluation disagrees with the reference search incumbent")
        return result.routes, None, evaluation

    def finish(self, result):
        candidates = [(result.value, result.routes, result.incumbents[-1].from_squeeze)] if result.routes else []
        if self.history:
            # The current candidate is also the latest publication.
            self.history.popitem(last=True)
            candidates.extend(sorted(self.history.values(), key=lambda item: item[0]))
        for index, (value, routes, from_squeeze) in enumerate(candidates):
            if index:
                self.stats["fallback_attempts"] += 1
            try:
                evaluation = self.prepared.evaluate(routes)
            except ExactRouteError as error:
                self.stats["final_rejections"] += 1
                self.stats["squeeze_rejections"] += int(from_squeeze)
                logging.getLogger(__name__).warning("Exact final check rejected candidate %d: %s", index + 1, error)
                continue
            if self.best_exact is not None and evaluation.cost != self.best_exact:
                raise KayrosError("final exact evaluation disagrees with the accepted incumbent")
            if index:
                self.stats["recovered"] = 1
                logging.getLogger(__name__).warning("Recovered a checked incumbent on fallback attempt %d", index)
            if self.checked_fallback is None or evaluation.cost < self.checked_fallback[1]:
                return [list(route) for route in routes], value, evaluation
            break
        else:
            if self.checked_fallback is None:
                raise InfeasibleError(f"exact final check rejected all {len(candidates)} retained candidates")
            if candidates:
                self.stats["fallback_attempts"] += 1
                self.stats["recovered"] = 1
                logging.getLogger(__name__).warning("Recovered the checked warm start after %d final rejections", len(candidates))
        routes, cost = self.checked_fallback
        try:
            evaluation = self.prepared.evaluate(routes)
        except ExactRouteError as error:
            raise KayrosError("final exact evaluation rejected the checked warm start") from error
        if evaluation.cost != cost or (self.best_exact is not None and cost != self.best_exact):
            raise KayrosError("final exact evaluation disagrees with the checked warm start")
        return [list(route) for route in routes], None, evaluation


@dataclass
class Solution:
    """A solution checked by mamut-routing-lib under the td-binary64-exact-v1 profile.

    exact_cost is the rational objective and cost is its nearest binary64 display. FleetCostDuration includes the fixed charge for each used route. exact_route_durations and exact_route_departures give the rational duration and earliest minimizing departure for each route, in the order of routes. route_durations and route_departures contain their binary64 displays.

    search_value is the approximate objective of the selected routes under the binary64 search fold, or None for an exact reference search or a warm start that the binary64 fold cannot represent. exact_evaluation contains the library record, including the profile, backend and authoritative cost."""

    instance_name: str
    routes: list[list[int]]
    # Binary64 display of the checked objective, including the per-route charge under FleetCostDuration.
    cost: float | None
    route_durations: list[float]
    route_departures: list[float]
    status: str  # "finished" | "time_limit"
    iterations: int
    objective: str = "Duration"  # canonical lib value: "Duration" | "FleetCostDuration"
    exact_cost: Fraction | None = None
    exact_route_durations: list[Fraction] = field(default_factory=list)
    exact_route_departures: list[Fraction] = field(default_factory=list)
    search_value: float | None = None
    exact_evaluation: dict | None = None
    incumbents: list[Incumbent] = field(default_factory=list)
    # Route-dissolve kick counters from ILS, or None under Duration.
    fleet_stats: dict[str, int] | None = None
    # Fleet-descent counters for FleetCostDuration, or None under Duration.
    fd_stats: dict[str, int] | None = None
    # Route-count changes during search under either objective.
    k_stats: dict[str, int] = field(default_factory=dict)
    # Cap diagnostics for FleetCostDuration runs with a positive k_cap.
    k_cap_stats: dict[str, int] | None = None
    # Candidate pricings and restarts to the best accepted solution.
    work_units: int = 0
    restarts: int = 0
    acceptance_stats: dict[str, int] = field(default_factory=dict)

    @property
    def num_routes(self) -> int:
        return len(self.routes)

    def to_benchmark_solution(self) -> BenchmarkSolution:
        """MAMUT solution artifact (feeds the BKS pipeline)."""
        # The route witnesses are not written here: the library derives them
        # from the routes when it stores a solution, and reserves their keys.
        metadata = {"solver": "kayros"}
        if self.objective != "Duration":
            metadata["objective_function"] = self.objective
        return BenchmarkSolution(
            instance_name=self.instance_name,
            routes=self.routes,
            cost=self.cost,
            metadata=metadata,
        )


_STATUS_NAMES = {
    _core.SolveStatus.Finished: "finished",
    _core.SolveStatus.TimeLimit: "time_limit",
}

_FD_STATS_FIELDS = (
    "triggers",
    "attempts",
    "successes",
    "pops",
    "step1_inserts",
    "ejections",
    "rollbacks_deadend",
    "rollbacks_budget",
    "rollbacks_time",
    "rollbacks_work",
    "evaluated",
    "basin_evaluated",
    "squeeze_phases",
    "squeeze_evaluated",
    "squeeze_checkpoints",
    "squeeze_improved",
    "ladder_squeezes",
    "ladder_rescues",
)

_K_CAP_STATS_FIELDS = (
    "seed_drain_attempts",
    "reached_cap",
    "rejected_above_cap",
    "first_capped_work",
)

_K_STATS_FIELDS = (
    "k_seed",
    "k_final",
    "k_best_min",
    "k_best_max",
    "singleton_opens",
    "kicks_opening",
    "k_up_after_kick",
    "k_down_after_kick",
    "k_up_after_descent",
    "k_down_after_descent",
    "accepted_k_up",
    "accepted_k_down",
    "new_best_k_up",
    "new_best_k_down",
)

_FLEET_STATS_FIELDS = (
    "kicks_total",
    "kicks_applied",
    "redraws_sum",
    "dissolved_armed",
    "dissolve_undone_in_kick",
    "normal_kicks",
    "k_after_kick_lt",
    "k_after_kick_eq",
    "k_after_kick_gt",
    "k_after_descent_lt",
    "k_after_descent_eq",
    "k_after_descent_gt",
    "dissolved_accepted_lahc",
    "normal_accepted_lahc",
    "dissolved_new_best",
    "normal_new_best",
    "dissolved_new_best_k_lt",
)


def solve(
    instance: str | Path | LoadedTDInstance,
    params: Params | None = None,
    *,
    time_limit: float | None = None,
    seed: int = 0,
    on_incumbent: IncumbentHook | None = None,
    verify: bool = False,
    initial_routes: Sequence[Sequence[int]] | None = None,
) -> Solution:
    """Run iterated local search on a TDVRPTW or TDVRP instance. Duration is the default objective. Set params.objective to FleetCostDuration to price used routes.

    Pass a .vrp.json path or a LoadedTDInstance. For a path, verify=True checks the sidecar and materialized ATF digests against the instance declaration during loading. This check requires an additional pass over the arc data. An already-loaded instance must have been verified when it was loaded.

    initial_routes accepts a complete, feasible solution expressed as customer-only routes. The library checks the seed under the requested objective in both acceptance modes. Fleet descent can start above k_cap, but only solutions meeting the target enter the incumbent stream.

    If binary64 initialization cannot represent a validated in-cap seed, the callback immediately receives its exact cost and search_value=None in either mode. The search tries cold construction with the remaining time. The seed is retained on an exact tie.

    With search_mode="exact", construction, feasible moves and incumbent comparisons use exact rational arithmetic. Every published improvement has an exact cost and search_value=None in either acceptance mode. The reference search omits the penalized squeeze.

    In the default binary64 search, each candidate new best must pass exact feasibility and strictly improve the rational objective before publication. The synchronous callback receives its exact_cost, its binary64 display as value and its approximate search_value. Distinct rational costs can have the same display. The display is None when the rational cost overflows binary64.

    With binary64 search and acceptance_mode="final_only", subsequent search publications are provisional and carry exact_cost=None. If final verification rejects the last candidate, the solver tries up to incumbent_history_size earlier distinct candidates in increasing approximate cost order. Solution.routes and Solution.exact_cost describe the checked result."""
    if isinstance(instance, LoadedTDInstance):
        if verify:
            raise ValueError(
                "verify=True has nothing to verify on an already-loaded "
                "instance: pass the .vrp.json path to solve(), or call "
                "kayros.io.load_instance(path, verify=True) yourself"
            )
        loaded = instance
    else:
        loaded = load_instance(instance, verify=verify)
    params = params or Params()
    if params.search_mode not in ("binary64", "exact"):
        raise ValueError(f"unknown search_mode {params.search_mode!r}")
    reference = params.search_mode == "exact"
    if reference and (params.sq_work_cap > 0 or params.sq_ladder):
        raise ValueError("exact reference search requires sq_work_cap=0 and sq_ladder=False")
    objective = canonical_objective(params.objective)
    # The objective is always passed explicitly: the default of the library is
    # the published objective of the family, which is not "Duration" everywhere.
    if objective != "Duration":
        # FleetCostDuration requires the per-route charge in the instance.
        if getattr(loaded.instance, "fleet_fixed_cost", None) is None:
            raise KayrosError(
                f"instance {loaded.instance.instance_name} carries no "
                f"fleet_fixed_cost; the FleetCostDuration objective requires it"
            )
    acceptance = _Acceptance(loaded, objective, params, on_incumbent)
    seed_routes = acceptance.validate_seed(initial_routes) if initial_routes is not None else []
    core = acceptance.core
    tl = 0.0 if time_limit is None else float(time_limit)

    if params.strategy != "ils":
        raise ValueError(f"unknown strategy {params.strategy!r}")

    ils_core = params._to_ils_core()
    if tl <= 0.0 and params.ils_max_iterations <= 0:
        ils_core.ils_max_iterations = 5 * params.restart_no_improvement
    if reference:
        result = _core.solve_reference(
            core, ils_core, seed, tl, acceptance.publish_reference, initial_routes=seed_routes,
            fleet_cost_objective=objective == "FleetCostDuration",
        )
    else:
        result = _core.solve_ils(
            core, ils_core, seed, tl, acceptance.publish,
            initial_routes=seed_routes, accept_incumbent=acceptance.accept,
            on_unusable_warm_start=acceptance.unusable_warm_start if seed_routes else None,
            fleet_cost_objective=objective == "FleetCostDuration",
        )
    if (result.status == _core.SolveStatus.Infeasible or not result.routes) and acceptance.checked_fallback is None:
        if acceptance.stats["infeasible"]:
            raise InfeasibleError(f"no exact-feasible incumbent in {acceptance.stats['proposals']} proposals")
        if params.k_cap > 0 and objective != "Duration":
            # No seed or accepted candidate met the fleet cap.
            raise InfeasibleError(
                f"kayros found no solution within k_cap={params.k_cap} routes "
                f"for {loaded.instance.instance_name}"
            )
        raise InfeasibleError(
            f"kayros could not construct a feasible solution for "
            f"{loaded.instance.instance_name}"
        )

    if reference:
        routes, search_value, evaluation = acceptance.finish_reference(result)
    else:
        routes, search_value, evaluation = acceptance.finish(result)
    witnesses = dict(zip(evaluation.routes, evaluation.route_evaluations))
    evaluations = [witnesses[tuple(route)] for route in routes]

    return Solution(
        instance_name=loaded.instance.instance_name,
        routes=routes,
        cost=evaluation.display_cost,
        route_durations=[float(e.duration) for e in evaluations],
        route_departures=[float(e.departure_time) for e in evaluations],
        exact_cost=evaluation.cost,
        exact_route_durations=[e.duration for e in evaluations],
        exact_route_departures=[e.departure_time for e in evaluations],
        search_value=search_value,
        exact_evaluation=evaluation.as_dict(),
        status=_STATUS_NAMES[result.status],
        iterations=result.iterations_run,
        objective=objective,
        fleet_stats=None if objective == "Duration" else {
            name: getattr(result.fleet_stats, name)
            for name in _FLEET_STATS_FIELDS
        },
        fd_stats=None if objective == "Duration" else {
            name: getattr(result.fd_stats, name)
            for name in _FD_STATS_FIELDS
        },
        work_units=result.work_units,
        restarts=result.restarts,
        k_stats={**{name: getattr(result.k_stats, name) for name in _K_STATS_FIELDS},
                 "k_final": len(routes)},
        k_cap_stats=None if objective == "Duration" or params.k_cap == 0 else {
            name: getattr(result.k_cap_stats, name)
            for name in _K_CAP_STATS_FIELDS
        },
        incumbents=acceptance.events,
        acceptance_stats=acceptance.stats,
    )
