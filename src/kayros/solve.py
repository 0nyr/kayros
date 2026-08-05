"""The kayros solve entry point (stage 1: greedy seed + TD-ACO)."""

from __future__ import annotations

from dataclasses import dataclass, field
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
    """Solver parameters. ``strategy`` picks the search: ``"ils"``
    (single-trajectory TD-ILS: granular VND + ruin-and-recreate +
    late-acceptance — the default since 0.4.0, picked by a 20,808-run
    head-to-head campaign where ILS beat ACO on 5714 of 6936 paired cells
    and lost 305, with the margin growing with instance size), ``"aco"``
    (TD-ACO, the historical default through 0.3.x), or ``"aco+ils"`` (an
    experimental budget split: ACO for ``aco_budget_fraction`` of the time
    limit, then ILS warm-started from the ACO best — statistically tied
    with pure ILS on small instances and dominated by it at scale).

    ACO parameter defaults are the original tuned bp_heur values; ILS
    defaults follow PyVRP v0.14 (restart threshold scaled to kayros's
    ms-class iterations). ``num_neighbours``/``weight_wait`` (granular
    candidate lists, M7.0) apply to the local search of EVERY strategy —
    default-on since 0.4.0; set ``num_neighbours=0`` for the pre-0.4.0
    exhaustive scans.

    ``objective`` picks the scored objective (1.2.0): ``"duration"`` (the
    default, unchanged behavior) or ``"fleet_cost_duration"``
    (FleetCostDuration: the same canonical duration fold plus
    ``fleet_fixed_cost * num_routes``, read from the instance; the lib
    ``ObjectiveFunction`` members are accepted too). Every strategy prices
    it through the same core fold; requires mamut-routing-lib >= 0.9.0 and
    an instance carrying ``fleet_fixed_cost``."""

    strategy: str = "ils"
    objective: str = "duration"
    # ACO-only parameters (strategy="aco" / "aco+ils"): every field from
    # max_iterations through ls_all_ants below is read by _to_core (the ACO
    # loop) only. The default "ils" strategy reads NONE of them (see
    # _to_ils_core); they are dormant in a default solve() and kept for the
    # opt-in ACO strategies. Do not infer the running algorithm from their
    # presence here.
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
    # Shared by EVERY strategy from here on (ILS included), until noted.
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
    # (and every fd_period iterations when fd_period > 0). Inert under
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
    fd_period: int = 0
    fd_route_choice: int = 0  # 0 uniform random victim, 1 smallest
    fd_pop_order: int = 0  # 0 LIFO, 1 difficult-first
    # Work-based triggers (session 44): thresholds in LS work units (candidate
    # pricings, see Solution.work_units). Deterministic wall-time proxies that
    # self-scale with instance size, where the flat iteration counts above fit
    # one size class only (Blauth2024 iteration velocity spans ~1100/s at n=10
    # to ~0.2/s at n=2000). 0 disables; each may be combined with its
    # iteration-count counterpart (whichever fires first). fd_period_work only
    # arms when the objective prices routes, like fd_period.
    # F-gated like fd_period: dead code under Duration at any value, so the
    # 1.3.0 default (the campaign-winning ~150 s cadence on one modern core)
    # only changes FleetCostDuration runs.
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
    # strategy="aco+ils" ONLY: fraction of the time limit given to the ACO
    # phase. Unused by the default "ils" strategy and by pure "aco".
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
        params.fd_period = self.fd_period
        params.fd_route_choice = self.fd_route_choice
        params.fd_pop_order = self.fd_pop_order
        params.fd_period_work = self.fd_period_work
        params.restart_no_improvement_work = self.restart_no_improvement_work
        params.seed_k_factor = self.seed_k_factor
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
    by ``mamut_routing_lib.td.check_td_solution`` for ``objective`` (the
    reference objective). Under ``"FleetCostDuration"`` that value includes
    the ``fleet_fixed_cost * num_routes`` term; ``route_durations`` stay pure
    per-route durations under every objective."""

    instance_name: str
    routes: list[list[int]]
    # TODO(2.0): rename to an objective-neutral `cost` (with a compat alias);
    # under FleetCostDuration this already holds the full objective value
    # including fleet_fixed_cost * num_routes (decided with Onyr, session 41).
    duration: float
    route_durations: list[float]
    route_departures: list[float]
    status: str  # "finished" | "converged" | "time_limit"
    iterations: int
    objective: str = "Duration"  # canonical lib value: "Duration" | "FleetCostDuration"
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
    # dissolve branch. Always a dict for an ILS solve; empty for pure ACO.
    k_stats: dict[str, int] = field(default_factory=dict)
    # Work-trigger diagnostics (session 44, ILS only; 0 for pure ACO): total
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
        metadata = {
            "solver": "kayros",
            "route_durations": self.route_durations,
            "route_departure_times": self.route_departures,
        }
        if self.objective != "Duration":
            metadata["objective_function"] = self.objective
        return BenchmarkSolution(
            instance_name=self.instance_name,
            routes=self.routes,
            cost=self.duration,
            metadata=metadata,
        )


_STATUS_NAMES = {
    _core.SolveStatus.Finished: "finished",
    _core.SolveStatus.Converged: "converged",
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
) -> Solution:
    """Solve a MAMUT TD instance (TDVRPTW or TDVRP; Duration minimization by
    default, FleetCostDuration via ``params.objective``).

    The search strategy is ``params.strategy`` and the DEFAULT is ``"ils"``
    (greedy seed, then single-trajectory TD-ILS; the default since 0.4.0,
    picked by the ILS-vs-ACO head-to-head campaign, see ``Params``). A
    default call (``params=None`` or ``Params()``) therefore involves no
    ant, colony or pheromone machinery: ACO only runs when explicitly
    requested via ``strategy="aco"`` or the experimental ``"aco+ils"``
    split.

    ``instance`` is a ``.vrp.json`` path or an already-loaded
    ``LoadedTDInstance``. The returned ``Solution.duration`` is priced by the
    reference checker under the selected objective; an internal/checker
    disagreement raises (it would be a kayros bug, never a rounding issue to
    tolerate).

    ``on_incumbent`` makes the solve anytime: it fires synchronously on every
    new incumbent with the ``Incumbent`` record and the routes, so callers can
    checkpoint solutions while the search keeps running. The stream is strictly
    improving and opens on the ``"greedy"`` seed within a fraction of a second
    at every instance size (1.4.0), well before the first descent lands. Keep
    the hook cheap — the solve loop blocks on it; an
    exception raised inside it aborts the solve and propagates.
    """
    loaded = instance if isinstance(instance, LoadedTDInstance) else load_instance(instance)
    params = params or Params()
    objective = canonical_objective(params.objective)
    check_kwargs: dict = {}
    if objective != "Duration":
        # Contract guards up front (mirroring the checker's misuse guards):
        # the objective needs the instance field and a lib that scores it.
        if getattr(loaded.instance, "fleet_fixed_cost", None) is None:
            raise KayrosError(
                f"instance {loaded.instance.instance_name} carries no "
                f"fleet_fixed_cost; the FleetCostDuration objective requires it"
            )
        try:
            check_kwargs["objective_function"] = ObjectiveFunction(objective)
        except ValueError:
            raise KayrosError(
                "objective 'fleet_cost_duration' requires mamut-routing-lib "
                ">= 0.9.0"
            ) from None
    core = to_core(loaded, objective_function=objective)
    tl = 0.0 if time_limit is None else float(time_limit)

    if params.strategy not in ("aco", "ils", "aco+ils"):
        raise ValueError(f"unknown strategy {params.strategy!r}")
    # The ILS loop has no convergence stop: without a time limit or an
    # explicit iteration budget it runs five restart windows (5 *
    # restart_no_improvement iterations, 100k with the defaults) so the
    # no-argument solve() stays finite.
    ils_fallback_iterations = 0
    if params.strategy == "ils" and tl <= 0.0 and params.ils_max_iterations <= 0:
        ils_fallback_iterations = 5 * params.restart_no_improvement
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
        ils_core = params._to_ils_core()
        if ils_fallback_iterations > 0:
            ils_core.max_iterations = ils_fallback_iterations
        result = _core.solve_ils(core, ils_core, seed, tl, make_hook())
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

    # The declared cost makes this a bitwise objective-equality gate: the
    # checker recomputes under `objective` and rejects any mismatch
    # (OBJECTIVE_VALUE_MISMATCH), Duration and FleetCostDuration alike.
    check = check_td_solution(
        loaded,
        BenchmarkSolution(
            instance_name=loaded.instance.instance_name,
            routes=[list(route) for route in result.routes],
            cost=result.value,
        ),
        **check_kwargs,
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
