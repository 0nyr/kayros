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
    # Granular candidate lists (M7.0): per client, the num_neighbours nearest
    # others under a TD adaptation of the Vidal (2013) proximity (min ATF
    # travel duration + weight_wait * inevitable wait), restricting the LS
    # move enumeration. Default-on since 0.4.0 — a deliberate behavior change
    # vs 0.3.0's exhaustive scans; set num_neighbours=0 to restore them.
    num_neighbours: int = 50
    weight_wait: float = 0.2
    # TD-ILS (M7.1/M7.2): perturbation magnitude, LAHC history, restart-to-
    # best threshold, exhaustive polish on new global bests. ils_max_iterations
    # 0 means unbounded when a time limit is set (the TL is then the stopping
    # criterion); with neither, solve() falls back to five restart windows
    # (5 * restart_no_improvement iterations) so the default call stays finite.
    min_perturbations: int = 1
    max_perturbations: int = 25
    # Route-dissolve kick share in percent (M7 FleetCostDuration): a dissolve
    # kick removes one smallest route whole so its clients repair into the
    # rest. Inert under Duration (only armed when the objective prices
    # routes), so the default changes nothing for existing callers.
    dissolve_pct: int = 50
    lahc_history: int = 300
    restart_no_improvement: int = 20_000
    exhaustive_on_best: bool = True
    # Fleet-descent phase (Plan 12 M4, 1.3.0): an NBRMH-style ejection-ladder
    # route elimination run on the incumbent at every restart-to-best trigger
    # and after each fd_period_work candidate evaluations. Inert under
    # Duration (only armed when the objective prices routes). fd_attempts=0
    # disables it entirely (the pre-M4 behavior).
    fd_attempts: int = 3
    fd_k_max: int = 2
    fd_ep_budget: int = 2000
    # Per-trigger drain work cap in candidate pricings (fd_stats "evaluated"
    # units; 0 = uncapped). BREAKING in 1.6: replaces fd_time_cap_seconds
    # (10.0 s wall per trigger, removed outright, plan 15 D9/M0.2), the one
    # wall-clock decision path the solver had: it made FCD trajectories
    # machine-dependent (76.3 % of the plan-12 weekend cells had time-based
    # drain rollbacks, fully explaining the grvingt-vs-grappe divergence).
    # The default matches the removed cap's intent at the calibrated ~650k
    # pricings/s single-core rate (10 s = 6.5M). FCD trajectories differ
    # from 1.3.0-1.5.0; Duration streams are untouched (the branch is F-gated
    # dead code there).
    fd_work_cap: int = 6_500_000
    # Plan-15 S2 squeeze (INERT at the default 0; new in 1.6): a penalised
    # polish of the post-drain state on every fleet-descent trigger, on both
    # the post-drop and the no-drop (matched-K) paths. A granular descent on
    # Phi = duration + sq_penalty * time-warp may pass through transient
    # time-window violations; every exactly-zero-warp improvement is banked
    # and only a strictly better bank is ever adopted, so the published
    # stream stays feasible by construction. rng-free and F-gated: dead code
    # under Duration; sq_work_cap=0 recovers the pre-squeeze FCD streams
    # bitwise. sq_work_cap = phase budget in penalised candidate pricings;
    # sq_penalty = explore-leg warp weight (ms of duration per ms of warp);
    # sq_on_nodrop gates the matched-K half.
    sq_work_cap: int = 0
    sq_penalty: float = 10.0
    sq_on_nodrop: bool = True
    # Plan-15 S1 (inert at False, independent of sq_work_cap): the in-ladder
    # squeeze between feasible-insert failure and ejection, charging the
    # drain's fd_work_cap budget; the difficulty counter then increments only
    # after the squeeze also fails (reference RMH semantics).
    sq_ladder: bool = False
    # Plan-15 S3 (inert at 0, F-gated: dead code under Duration at any value):
    # confine the search to at most k_cap routes. A seed or warm start above
    # the cap is drained toward it before the loop opens; candidates never
    # raise K above max(k_cap, current K); incumbents publish only at
    # K <= k_cap, and a run that never reaches the cap comes back Infeasible
    # with an empty incumbent stream rather than an above-cap solution
    # (design memo section 10, D-S3.1): an empty cell is the honest outcome.
    # The cap is a plain route-count target with no anchoring to any
    # reference fleet; sub-reference caps are legitimate (plan 15 D10).
    k_cap: int = 0
    # Candidate evaluations between fleet-descent phases. Zero disables the periodic trigger.
    fd_period_work: int = 100_000_000
    # BREAKING default in 1.3.0: the work-based restart is ON by default
    # with a deliberately long ~25-minute stall window (the flat
    # restart_no_improvement=20000 above never fires on realistic budgets at
    # n >= 500, so stalled hours-scale searches never restarted; shorter
    # windows measurably hurt sub-half-hour budgets). Long Duration runs
    # diverge from 1.1.x AT DEFAULTS; pass restart_no_improvement_work=0 to
    # recover bitwise 1.1.x streams.
    restart_no_improvement_work: int = 1_000_000_000
    # K-diverse seeding (plan 13 I2, 1.5.0): split the greedy seed's routes
    # until the route count reaches seed_k_factor times the constructed one,
    # then search from there. 1.0 disables it and restores the 1.4.x seed
    # exactly. Under Duration the search sheds routes freely and effectively
    # never adds one, so the seed's route count decides the final one; on
    # fleet-starved instances starting higher and letting the search descend is
    # worth a great deal, and elsewhere it is inside seed noise. Armed under
    # Duration ONLY — under FleetCostDuration every extra route is priced, so
    # the core ignores this knob there whatever its value.
    # BREAKING default in 1.5.0: 2.0. Measured gain on fleet-starved instances
    # is -3.3/-4.2/-5.2/-5.4/-5.4 percent at 1.25/1.5/2/3/4, and flat noise
    # elsewhere; 2.0 takes 96 percent of it at the lowest route count, and is
    # the setting a full non-regression sweep covered. Duration trajectories
    # move at defaults versus 1.4.x; pass seed_k_factor=1.0 to recover them.
    seed_k_factor: float = 2.0
    ils_max_iterations: int = 0

    # Final-only keeps this many distinct prior route sets in addition to the current best.
    acceptance_mode: Literal["each_incumbent", "final_only"] = "each_incumbent"
    incumbent_history_size: int = 6

    def _to_ils_core(self) -> _core.IlsParams:
        params = _core.IlsParams()
        if self.ils_max_iterations > 0:
            params.max_iterations = self.ils_max_iterations
        params.num_neighbours = self.num_neighbours
        params.weight_wait = self.weight_wait
        params.min_perturbations = self.min_perturbations
        params.max_perturbations = self.max_perturbations
        params.dissolve_pct = self.dissolve_pct
        params.history_length = self.lahc_history
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
    value: float
    seconds: float
    iteration: int | None  # ILS iteration when available
    origin: str
    exact_cost: Fraction | None = None
    search_value: float | None = None


_ORIGIN_NAMES = {0: "greedy", 2: "ils", 3: "warm_start"}


# Anytime hook: called synchronously from inside the solve loop on every new
# incumbent, with the incumbent record and its routes (customer ids, no depot).
IncumbentHook = Callable[[Incumbent, list[list[int]]], None]


class _Acceptance:
    """The native loop calls accept before storing its incumbent, then publish after acceptance."""

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
        if self.mode == "each_incumbent":
            self.verified_seed = (seed, evaluation.cost)
        return seed

    def accept(self, routes, from_squeeze):
        self.stats["proposals"] += 1
        if self.mode == "final_only":
            return True
        seed, self.verified_seed = self.verified_seed, None
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
        event = Incumbent(float(exact) if exact is not None else inc.value, inc.seconds,
                          inc.iteration, _ORIGIN_NAMES[inc.origin], exact, inc.value)
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

    def finish(self, result):
        candidates = [(result.value, result.routes, result.incumbents[-1].from_squeeze)]
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
            return [list(route) for route in routes], value, evaluation
        raise InfeasibleError(f"exact final check rejected all {len(candidates)} retained candidates")


@dataclass
class Solution:
    """A solution checked under the exact profile of mamut-routing-lib
    (``td-binary64-exact-v1``).

    ``exact_cost`` is the authoritative rational value of ``objective``, as
    computed by the prepared exact evaluator of mamut-routing-lib. ``cost`` is its
    binary64 display (the nearest double). Under ``"FleetCostDuration"`` both
    include the ``fleet_fixed_cost * num_routes`` term. ``route_durations`` and
    ``route_departures`` are the displays of the exact per-route durations and
    earliest minimizing departures, in the order of ``routes``. Their rational
    values are in ``exact_route_durations`` and ``exact_route_departures``.
    ``search_value`` is the value of the same solution under the binary64 fold
    that the search uses. It is internal to the solver and can differ from
    ``cost`` in the last bits. ``exact_evaluation`` is the record of the
    library (profile, backend, authoritative cost as an integer pair)."""

    instance_name: str
    routes: list[list[int]]
    # Named `duration` before 2.0.0. Under FleetCostDuration it holds the full
    # objective value, including fleet_fixed_cost * num_routes.
    cost: float
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
    # Plan-12 M1 dissolve-kick lifecycle counters (ILS iterations only), as a
    # plain dict of the core FleetKickStats fields. None under "Duration",
    # where the dissolve is never armed and the counters carry no signal.
    fleet_stats: dict[str, int] | None = None
    # Plan-12 M4 fleet-descent phase diagnostics (core FdStats fields), same
    # contract: None under "Duration" (the phase is never armed there).
    fd_stats: dict[str, int] | None = None
    # Route-count movement over the solve (core KStats fields). Unlike
    # fleet_stats and fd_stats this is recorded under EVERY objective, because
    # the route count moves under Duration too and used to be invisible there:
    # every K counter the solver had lived inside the FleetCostDuration-gated
    # dissolve branch. Always a dict.
    k_stats: dict[str, int] = field(default_factory=dict)
    # Plan-15 S3 K-cap diagnostics (core KCapStats fields). None when the cap
    # is off or the objective is Duration (the cap is F-gated dead code there).
    k_cap_stats: dict[str, int] | None = None
    # Work-trigger diagnostics: total
    # LS work units spent (candidate pricings; work_units / wall seconds is
    # the machine's work rate, the calibration source for the *_work
    # thresholds) and restart-to-best count.
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
    """Solve a MAMUT TD instance (TDVRPTW or TDVRP; Duration minimization by
    default, FleetCostDuration via ``params.objective``).

    ``instance`` is a ``.vrp.json`` path or an already-loaded
    ``LoadedTDInstance``. The returned ``Solution.cost`` is the binary64
    display of the exact objective, with its rational value in ``Solution.exact_cost``.

    ``verify`` applies to the loading half only, and is forwarded to
    ``kayros.io.load_instance``: it re-derives the sidecar digests and the
    ``atf_sha256`` of the materialized arrival-time functions and pins them
    against what the instance file declares. Verify on test runs and on the
    first run over unfamiliar data, where the check catches a truncated
    download or a mismatched sidecar; leave it off afterwards, since
    materialization is deterministic and the check costs minutes and several
    gigabytes of peak memory at n = 1000. It is rejected when ``instance`` is
    an already-loaded ``LoadedTDInstance``, because by then the artifacts have
    been read and it is too late to verify them here.

    Pass customer-only routes in ``initial_routes`` to start from a complete feasible solution.
    The library validates the seed under the requested objective in both acceptance modes.
    An otherwise feasible seed may exceed the search target ``k_cap``. The fleet descent
    can use it, and only solutions meeting the target enter the incumbent stream.

    By default, a candidate new best must pass exact feasibility and strictly improve the
    rational objective before it replaces the incumbent. The synchronous callback receives
    its exact cost, its binary64 display as ``value``, and the approximate ``search_value``.
    Distinct exact costs can have the same display. With ``acceptance_mode="final_only"``,
    callback events and ``Solution.incumbents`` describe provisional search improvements
    and carry ``exact_cost=None``. If the final candidate fails, the solver tries up to
    ``incumbent_history_size`` prior distinct incumbents in increasing approximate cost
    order. ``Solution.routes`` and its cost always describe the checked result.
    """
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
    objective = canonical_objective(params.objective)
    # The objective is always passed explicitly: the default of the library is
    # the published objective of the family, which is not "Duration" everywhere.
    if objective != "Duration":
        # Contract guards up front (mirroring the checker's misuse guards):
        # the objective needs the instance field and a lib that scores it.
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
        ils_core.max_iterations = 5 * params.restart_no_improvement
    result = _core.solve_ils(
        core, ils_core, seed, tl, acceptance.publish,
        initial_routes=seed_routes, accept_incumbent=acceptance.accept,
    )
    if result.status == _core.SolveStatus.Infeasible or not result.routes:
        if acceptance.stats["infeasible"]:
            raise InfeasibleError(f"no exact-feasible incumbent in {acceptance.stats['proposals']} proposals")
        if params.k_cap > 0 and objective != "Duration":
            # Plan-15 S3 (D-S3.1): the run never reached the cap, so it
            # publishes nothing. An empty cell is the honest outcome.
            raise InfeasibleError(
                f"kayros found no solution within k_cap={params.k_cap} routes "
                f"for {loaded.instance.instance_name}"
            )
        raise InfeasibleError(
            f"kayros could not construct a feasible solution for "
            f"{loaded.instance.instance_name}"
        )

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
