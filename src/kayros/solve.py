"""Time-dependent iterated local search."""

from __future__ import annotations

from dataclasses import dataclass, field
from decimal import Decimal
from fractions import Fraction
from pathlib import Path
from typing import Callable

from mamut_routing_lib.enums import ObjectiveFunction
from mamut_routing_lib.models import BenchmarkSolution
from mamut_routing_lib.td import LoadedTDInstance, check_td_solution

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
    iteration: int
    origin: str  # "greedy" | "ils"


_ORIGIN_NAMES = {0: "greedy", 2: "ils"}


# Anytime hook: called synchronously from inside the solve loop on every new
# incumbent, with the incumbent record and its routes (customer ids, no depot).
IncumbentHook = Callable[[Incumbent, list[list[int]]], None]


@dataclass
class Solution:
    """A solution checked under the exact profile of mamut-routing-lib
    (``td-binary64-exact-v1``).

    ``exact_cost`` is the authoritative rational value of ``objective``, as
    computed by ``mamut_routing_lib.td.check_td_solution``. ``cost`` is its
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
) -> Solution:
    """Solve a MAMUT TD instance (TDVRPTW or TDVRP; Duration minimization by
    default, FleetCostDuration via ``params.objective``).

    ``instance`` is a ``.vrp.json`` path or an already-loaded
    ``LoadedTDInstance``. The returned ``Solution.cost`` is priced by the
    reference checker under the selected objective; an internal/checker
    disagreement raises (it would be a kayros bug, never a rounding issue to
    tolerate).

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

    ``on_incumbent`` makes the solve anytime: it fires synchronously on every
    new incumbent with the ``Incumbent`` record and the routes, so callers can
    checkpoint solutions while the search keeps running. The stream is strictly
    improving and opens on the ``"greedy"`` seed within a fraction of a second
    at every instance size (1.4.0), well before the first descent lands. Keep
    the hook cheap — the solve loop blocks on it; an
    exception raised inside it aborts the solve and propagates.
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
    check_kwargs: dict = {"objective_function": ObjectiveFunction(objective)}
    if objective != "Duration":
        # Contract guards up front (mirroring the checker's misuse guards):
        # the objective needs the instance field and a lib that scores it.
        if getattr(loaded.instance, "fleet_fixed_cost", None) is None:
            raise KayrosError(
                f"instance {loaded.instance.instance_name} carries no "
                f"fleet_fixed_cost; the FleetCostDuration objective requires it"
            )
    core = to_core(loaded, objective_function=objective)
    tl = 0.0 if time_limit is None else float(time_limit)

    if params.strategy != "ils":
        raise ValueError(f"unknown strategy {params.strategy!r}")

    def hook(inc, routes):
        on_incumbent(Incumbent(inc.value, inc.seconds, inc.iteration, _ORIGIN_NAMES[inc.origin]), routes)

    ils_core = params._to_ils_core()
    if tl <= 0.0 and params.ils_max_iterations <= 0:
        ils_core.max_iterations = 5 * params.restart_no_improvement
    result = _core.solve_ils(core, ils_core, seed, tl, hook if on_incumbent is not None else None)
    if result.status == _core.SolveStatus.Infeasible or not result.routes:
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

    # Final check under the exact profile of the family. No cost is declared:
    # the search value comes from a binary64 fold, and the reported cost is the
    # exact one. Feasibility is decided here by exact arithmetic.
    routes = [list(route) for route in result.routes]
    check = check_td_solution(
        loaded,
        BenchmarkSolution(
            instance_name=loaded.instance.instance_name,
            routes=routes,
            cost=None,
        ),
        **check_kwargs,
    )
    if not check.is_valid():
        raise KayrosError(
            f"final solution rejected by the exact checker "
            f"({check.status}: {check.error_message})"
        )
    # The checker returns its witnesses in canonical route order. Customers are
    # visited once, so the first customer identifies a route.
    witness = {e.route[0]: e for e in check.route_evaluations}
    evaluations = [witness[route[0]] for route in routes]
    exact_record = check.exact_evaluation
    # The integers of the record can exceed the digit limit of int(str) (6,009
    # digits on an instance with 1,000 customers). Decimal converts exactly.
    exact_cost = Fraction(
        int(Decimal(exact_record["authoritative_cost"]["numerator"])),
        int(Decimal(exact_record["authoritative_cost"]["denominator"])),
    )

    return Solution(
        instance_name=loaded.instance.instance_name,
        routes=routes,
        cost=check.routing_cost,
        route_durations=[float(e.duration) for e in evaluations],
        route_departures=[float(e.departure_time) for e in evaluations],
        exact_cost=exact_cost,
        exact_route_durations=[e.duration for e in evaluations],
        exact_route_departures=[e.departure_time for e in evaluations],
        search_value=result.value,
        exact_evaluation=exact_record,
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
        k_stats={
            name: getattr(result.k_stats, name) for name in _K_STATS_FIELDS
        },
        k_cap_stats=None if objective == "Duration" or params.k_cap == 0 else {
            name: getattr(result.k_cap_stats, name)
            for name in _K_CAP_STATS_FIELDS
        },
        incumbents=[
            Incumbent(i.value, i.seconds, i.iteration, _ORIGIN_NAMES[i.origin])
            for i in result.incumbents
        ],
    )
