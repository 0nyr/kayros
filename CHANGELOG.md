# Changelog

All notable changes to KAYROS are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project aims to follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html). Certificate semantics and benchmark provenance are documented in `README.md` and `cpp/lera/NOTICE.md`.

## [Unreleased]

### Changed

- **BREAKING: `Params.fd_time_cap_seconds` is removed outright and replaced by `Params.fd_work_cap` (default 6,500,000 candidate pricings per fleet-descent trigger; 0 = uncapped).** The wall-clock cap was the one wall-clock decision path left in the solver: it rolled the drain back after 10 s of wall time, so the same seed on the same instance took different FleetCostDuration trajectories on different machines (76.3 percent of the 3,692 plan-12 weekend cells had at least one time-based rollback, which fully explained a grvingt-versus-grappe divergence after 30 identical incumbents). The drain is now capped by its own work counter, machine-independently, following the `fd_period_work` pattern; the default matches the removed cap's intent at the calibrated ~650k pricings/s single-core rate. No deprecation alias: passing `fd_time_cap_seconds` now raises. **FleetCostDuration trajectories differ from 1.3.0-1.5.0 from this change on** (the cap fires at different points than the wall clock did); comparisons against pre-change FCD results are different-mechanism comparisons. Duration trajectories are bitwise unchanged at any setting: the whole branch is F-gated dead code there. The global `time_limit` still ends a drain mid-attempt at end-of-run (`fd_stats["rollbacks_time"]` keeps its name and now counts only that); the audit of the remaining wall-clock uses found none that takes a mid-run decision (the VND deadline in `ls_descend` and the `exhaustive_on_best` gates read the global time limit only).

### Added

- **The S3 K-cap** (plan 15, INERT by default): `Params.k_cap` (0 = off, F-gated: dead code under Duration at any value) confines the search to at most `k_cap` routes. A seed or warm start above the cap is drained toward it by back-to-back fleet-descent attempts before the loop opens (the campaign wave shape: warm-start at K, cap at K-1); a candidate may never raise the route count above `max(k_cap, current K)`, so the search is monotone toward the cap; incumbents are published only at `K <= k_cap`, and a run that never reaches the cap returns Infeasible with an EMPTY incumbent stream rather than an above-cap solution, so an above-cap result can never masquerade as a capped one. The cap is a plain route-count target: nothing anchors it to any reference fleet, and sub-reference caps are legitimate. Diagnostics in `Solution.k_cap_stats` (`seed_drain_attempts`, `reached_cap`, `rejected_above_cap`, `first_capped_work`); `None` when the cap is off or the objective is Duration.
- **The S2 squeeze** (plan 15, INERT by default): a penalised polish of the post-drain state on every fleet-descent trigger, on both the post-drop and the no-drop (matched-K) paths. A granular descent on `Phi = duration + sq_penalty * time-warp` may pass through transient time-window violations; every exactly-zero-warp improvement is banked (FleetCostDuration-priced, so mid-phase route drops are credited `F`), a dominating-repair tail leg drives residual warp back to zero, and only a strictly better bank is ever adopted, so the published stream stays feasible by construction and a successful route drop is never converted into a failure. Knobs: `sq_work_cap` (0 = off, the phase budget in penalised candidate pricings), `sq_penalty`, `sq_on_nodrop`. The in-ladder half, **S1** (`sq_ladder`, inert at False, independent of `sq_work_cap`): the squeeze slots between feasible-insert failure and ejection inside the fleet-descent drain (the RMH step the ladder omitted), force-inserting the popped client at the minimum-penalised position and repairing one warp-positive route at a time back to exactly-zero warp, charging the drain's `fd_work_cap` budget; the difficulty counter then increments only after the squeeze also fails. Diagnostics: `fd_stats["ladder_squeezes"/"ladder_rescues"]`. rng-free and F-gated: Duration streams are untouched at any setting, and `sq_work_cap=0` recovers the pre-squeeze FleetCostDuration streams bitwise. Diagnostics: `fd_stats["squeeze_phases"/"squeeze_evaluated"/"squeeze_checkpoints"/"squeeze_improved"]`.
- **The TD time-warp evaluation layer, extracted from the `td-time-warp` branch** (plan 15 D4): clamp-at-deadline warp PWLF builders with the safe flat-run dedup, the augmented `(rho, W)` route fold and accounting evaluators, and the `(rho, omega)` segment monoid with `WarpLcaTree` and the penalised splice evaluator, with their 115 gates (bitwise reduction to the checker on feasible routes, pure-Python twin bit-identity, tree-vs-fold and update==rebuild gates). Additive: no existing code path changes; the branch's prototype drivers stay behind.

- **Fleet-descent work accounting** (plan 15 M0.3): `fd_stats["rollbacks_work"]` (drain attempts rolled back by `fd_work_cap`), `fd_stats["evaluated"]` (drain candidate pricings: ranked insertion candidates, splice evaluations and fold-commit rebuilds, one route pricing each) and `fd_stats["basin_evaluated"]` (LS work of the post-drop basin descents, in `Solution.work_units` units). Together they give fleet descent's share of solver work: `(evaluated + basin_evaluated) / (work_units + evaluated)`.

## [1.5.2] — 2026-08-07

### Changed

- **Loading no longer verifies artifact digests by default, and the check becomes a visible `verify` argument.** `kayros.io.load_instance(path)` silently inherited `verify_sha256=True` from the benchmark library, so the public single-argument entry point `kayros.solve("instance.vrp.json")` re-derived the `atf_sha256` of the materialized arrival-time functions on every call, through a full canonical re-serialization of the entire ATF set in Python. At n = 1000 that is roughly 80 seconds and about 10 GB of extra peak memory before the search even starts (measured on a one-million-arc instance carrying 114 million breakpoints), which is what users were seeing as an unexplained slow loading phase. Materialization is deterministic and the digest is already declared by the instance file, so on the solver's hot path the check spends minutes re-establishing what a single earlier check already established. It is now off unless you ask for it, and asking for it is one keyword.

  **When to ask for it.** `load_instance(path, verify=True)`, or `kayros.solve(path, verify=True)`, runs the full check: the sidecar digests, and the ATF digest against the value the instance file declares. Verify on test runs, and on the first run over data you have not used before. That is where the check earns its cost, because it catches a truncated download, a sidecar that does not belong to the instance sitting next to it, or an artifact someone edited by hand, and every one of those failure modes is otherwise silent: the solver optimizes the wrong travel times and returns a perfectly plausible answer. Once a given pair of artifacts has passed, re-checking it on every solve tells you nothing new.

  KAYROS's own test suite passes `verify=True` at every instance load, so the verification path stays exercised on every run instead of decaying into untested code, and two new gates pin what the switch is worth: a load with `verify=True` refuses a sidecar whose arrival values were altered, and the default load, by design, accepts it without a word.

### Added

- **`verify` (default `False`) on `kayros.io.load_instance` and on `kayros.solve`**: the opt-in artifact integrity check described above. On `solve` it belongs to the path-accepting form; passing `verify=True` together with an already-loaded `LoadedTDInstance` raises rather than being ignored, since the artifacts have been read by then and there is nothing left for `solve` to verify.

## [1.5.1] — 2026-08-05

### Fixed

- **SOUNDNESS: the exact prover over-certified jump-free instances from 1.1.0 through 1.5.0.** On instances whose travel-time functions carry no value jump, cold solves could return `Optimum` at a value strictly above a checker-valid solution. That is a false certificate, not a loose bound. Six instances are known so far, all in the Vu2020 family: `n=59/Vu-A5-pA-d90-w40` certified 2764.379834028286 against a true 2760.1098340282865 (+4.27), `n=59/Vu-A5-pB-d90-w100` +9.88, `n=79/Vu-A1-pA-d98-w80` +3.12, `n=59/Vu-A2-pB-d98-w60` +1.36, `n=79/Vu-A2-pB-d90-w80` +0.79, `n=99/Vu-A1-pA-d90-w100` +0.29.

  The cause is a **false premise in how the M13.0 exact value-jump arithmetic was switched on**, not in the arithmetic itself. M13.0 selected the exact path per operand, on "does this function carry a vertical piece", documenting that "mollified and jump-free arcs carry none" so jump-free instances would keep the audited 1.0.0 arithmetic bit-identically. They do carry verticals. A vertical is not only a value jump: inverting a departure-function plateau produces a CHOICE vertical, and a label's own duration function is set-valued at departure choices, both of which are ordinary features of jump-free time-dependent data. On the instance above the predicate fired on 6560 extensions against 1387 legacy ones, with zero jump verticals anywhere. Two of M13.0's three layers were each independently sufficient to over-certify: the elapsed-time extension identity in the labeling, and the `Compose` / `operator+` vertical rules in the PWL arithmetic. The exact-path identity over-estimates at unattained jump-gap abscissae, which is the sound side *for step-carrying data* and simply wrong where the "gap" is a choice plateau of a continuous function.

  The fix gates both layers on the solve being step-carrying: the same per-instance `KAYROS_STEP_EXACT` decision the Python bridge already makes from the instance ATFs, now pinned for the whole solve as a scoped process-global in goc (`goc::step_exact_arithmetic`) instead of being re-derived per operand. Jump-free solves take the 1.0.0 arithmetic; the six instances return their stored certificate values bit-exactly, and the previously falsified `n=59/Vu-A2-pB-d98-w100` returns 1846.807 rather than the retracted 1847.240. Step-carrying solves are byte-for-byte the 1.5.0 behaviour: the M13.0 reproducer gates (Rifki-2, Rifki-17, Rifki-18), the step gates and the differential fuzzer are unchanged and green.

  **Validated at scale before release**: the 2026-08-05 re-certification campaign (Grid'5000, four independent solves per instance) re-derived 466 of the 468 stored optimality stamps at their exact stored values on this build (one instance needed a dedicated high-memory re-run, peak resident set near 16.5 GB); the two exceptions are the certificate retracted as falsified (its true optimum is re-proven by three of four arms) and one four-way disagreement refused by the protocol. Single-arm over-certifications observed during that campaign (one per labeling mode, on different instances) confirm that neither jump-free arithmetic is provably complete arm by arm; the four-solve agreement protocol is the soundness instrument, and certificates exist only where it passes.

  **Consequence for stored certificates.** Every jump-free optimality stamp minted by 1.1.0 through 1.5.0 was produced by this arithmetic and must be re-validated; a stamp is only trustworthy again once re-derived on a build carrying this fix. Certificates minted before 1.1.0, and every step-carrying certificate, are unaffected. `cpp/lera/NOTICE.md` item 9 carries the corresponding amendment. The pre-release re-certification campaign of 2026-08-05 therefore identifies its build by commit (`df05a37`), not by version; builds installing this release read 1.5.1.

### Changed

- **The reverse-side `continuize_value_jumps` was investigated for removal and deliberately retained; no behavior changes.** With the forward mollifier deleted in 1.1.0, the reverse-side helper in the labeling looked like the last piece of dead mollifier machinery, and it was documented as a no-op on jump-free data. It is neither. An instrumented build shows it rewriting 229 of 314 reverse arrival functions on `TDVRPTW/Vu2020/n=59/Vu-A5-pA-d90-w40` (158 choice verticals flattened, 128 bridges inserted) and 590 of 650 on `TDVRP/Dabia2013/n=25/C101`, with no value jump anywhere in either instance. What it actually removes are the CHOICE verticals that inverting a departure-function plateau creates and the reflection carries into the reverse arrival, and that is the invariant the audited 1.0.0 jump-free arithmetic depends on. Deleting it made cold solves certify above checker-valid solutions again: `n=59/Vu-A2-pB-d98-w60` closed at 1278.262 against a stored 1275.832, `n=59/Vu-A2-pB-d98-w100` at 1852.944 against 1846.807, both deterministic. That is the same failure the Unreleased fix above repairs, mirrored onto the reverse instance, so the helper stays until the reverse side is made vertical-safe under exact arithmetic, which is a project of its own. The `duration_at` domain clamps were assessed in the same pass and kept: 47552 instrumented calls across six solves never left the domain, and nothing enforces that they cannot. `cpp/lera/NOTICE.md` item 9 carries the full amendment; the code comments that described the helper as inert are corrected.

### Added

- **Two reverse-side soundness gates**, `test_jumpfree_reverse_side_certificate_is_not_above_the_truth` on `Vu-A2-pB-d98-w60` and `Vu-A2-pB-d98-w100`. The existing jump-free gate pins `Vu-A5-pA-d90-w40`, which keeps certifying its exact stored value on a build whose reverse arrivals are wrong, so it passes a regression it was meant to catch. These two are the instances that discriminate, and each also checks its returned routes price at the certified value under the reference checker.
## [1.5.0] — 2026-08-04

A fleet-sizing release, and a **deliberate change of default behaviour**: Duration results move versus 1.4.x. Under the Duration objective the search sheds routes freely and effectively never adds one, so the route count it finishes with is essentially the one the seed handed it. On instances where waiting dominates, the constructed seed lands near half the duration-optimal fleet and the search cannot climb out. 1.5.0 seeds above the constructed count and lets the search descend to its own.

### Added

- **`Params.seed_k_factor` (default 2.0): K-diverse seeding.** Before the search starts, the greedy seed's routes are split until the route count reaches `seed_k_factor` times the constructed one, each step taking the cheapest feasible split. Because the objective is a sum of per-route durations, a candidate costs two route evaluations rather than a whole-solution refold, so the whole construction is 0.36 s at n = 1000 and 0.10 s at n = 500 next to a 0.04 s seed. Set `seed_k_factor=1.0` to recover 1.4.x trajectories bit for bit.

  **Armed under Duration only.** Under FleetCostDuration every extra route is priced, so splitting is the wrong direction there; the core ignores the knob whatever its value when the instance prices routes.

  Measured, at a fixed time limit so the extra per-iteration cost of more routes is already paid for in the numbers: on a dense, tight-time-window family the gain runs -3.3, -4.2, -5.2, -5.4 and -5.4 percent for multipliers 1.25, 1.5, 2, 3 and 4, with individual instances reaching -27 percent. Across every other family the effect is flat noise inside a tenth of a percent with no trend in the multiplier (215-run two-arm sweep over 43 instances spanning five families and n = 10 to 200, five seeds each: non-affected families +0.014 percent mean, median exactly zero). 2.0 takes 96 percent of the available gain at the lowest route count, and is the setting that sweep covers.

- **`Solution.k_stats`: route-count movement, recorded under every objective.** Seed and final route counts, the range the incumbents spanned, the repair's singleton route openings, and how the count moved across each perturbation, across perturbation plus descent, and among the candidates that were accepted or became new bests. The existing `fleet_stats` counters answer the same question for FleetCostDuration only — every one of them sits inside a branch gated on routes being priced — so under Duration the route count used to be invisible in the record. Instrumentation only: integer increments and route-count reads, no rng draw and no priced value.

## [1.4.0] — 2026-08-04

An anytime-latency release. At scale KAYROS used to spend its first minutes silent — building a seed nobody could see — and both halves of that are gone: the seed construction is two orders of magnitude faster, and it is published the moment it exists. Final solutions are unchanged.

### Changed

- **The greedy seed no longer runs its multi-hop lookahead, and is roughly two orders of magnitude faster.** Through 1.3.0 every placement ran a full time-dependent Dijkstra over the remaining free customers and selected on the earliest MULTI-HOP ready time; the selection is now the earliest DIRECT ready time. This is the same rule up to ties: Dijkstra's first settled node is by construction the free customer with the smallest direct ready time, so the multi-hop minimum equals the direct minimum and is attained at the same customer. The two rules can only part where a detour reaches a customer in exactly the same total time, which needs a time window binding hard enough for the wait to absorb the detour, and there the lookahead is a different tie break rather than a better choice. The construction drops from O(n^3) to O(n^2): 0.03 s instead of 36 s at n = 1000, 0.007 s instead of 3.5 s at n = 500, about 60x at n = 100 (one modern core).

  Verified by enumeration rather than argument: identical route sequences on all 232 instances of a MAMUT-routing benchmark set spanning five families and n = 10 to 1000, and on the checkout's test families as a standing gate (`tests/test_construct.py`). The equality is therefore **empirical, not proven** — on an instance where the wait-flattened tie case is real, the seed, and hence the whole search trajectory, can differ from 1.3.0. The removed rule is retained as `_core.greedy_makespan_lookahead`, used by nothing but that test, so the claim stays falsifiable.
- **The seed is published to `on_incumbent` the moment it is built**, before the first local-search descent instead of after it. Combined with the construction speedup, the anytime stream now opens in well under a second at every instance size, where 1.3.0 published nothing for about 100 s at n = 1000 (36 s seed plus ~65 s first descent). The stream stays strictly improving: the descended seed is published as a second `origin="greedy"` record only when it actually improves on the raw one, so callers that count incumbents will see one more record per solve.
- The two publication paths (ILS and ACO) now go through one internal publication point that owns the strictly-decreasing invariant, instead of repeating the push-and-fire at each site.

## [1.3.0] — 2026-07-29

### Added

- **Dissolve-kick lifecycle counters** (`SolveResult.fleet_stats`, surfaced as `Solution.fleet_stats` under FleetCostDuration; `None` under Duration): per-solve ILS diagnostics of the route-dissolve kick (armed / applied / undone-by-singleton-reopen counts, route-count deltas after the kick and after the granular descent, LAHC acceptance and new-best counts split dissolved vs normal kicks). Instrumentation only: integer increments and route-count reads, no rng draw and no change to any priced value, so Duration and FleetCostDuration trajectories are bitwise those of 1.2.1. These counters feed the fleet-descent mechanism design (the first Blauth2024 campaign showed the fleet term is the entire competitive gap).
- **Fleet-descent phase** (Plan 12 M4): an NBRMH-style ejection-ladder route elimination run on the incumbent at every restart-to-best trigger and, when `fd_period` / `fd_period_work` is set, periodically during the search. One attempt dissolves a victim route into a LIFO ejection pool and drains it through a two-rung ladder (tree-ranked feasible best-insert, then contiguous-window insertion-with-ejection guided by NBRMH difficulty counters), commits through the checker fold only, and restores the pre-attempt solution exactly on any dead end. Knobs: `fd_attempts` (0 disables), `fd_k_max`, `fd_ep_budget`, `fd_time_cap_seconds`, `fd_period`, `fd_route_choice`, `fd_pop_order`; diagnostics in `Solution.fd_stats`. Armed only when the objective prices routes, all draws behind the F-gate: Duration trajectories stay bitwise 1.1.x.
- **Work-based triggers** (`fd_period_work`, `restart_no_improvement_work`): thresholds counted in LS work units (candidate pricings, reported as `Solution.work_units` alongside the new `Solution.restarts`). A deterministic wall-time proxy that self-scales with instance size, where flat iteration counts fit one size class only (Blauth2024 iteration velocity spans ~1100 it/s at n=10 to ~0.2 it/s at n=2000, so the stock `restart_no_improvement=20000` never fired at n ≥ 500 in a 3-5 h run). The work unit is near machine-rate-invariant across sizes (a 1.5× spread from n=500 to n=2000 where iteration velocity spreads 18×, measured on Grid'5000 gros).

### Changed

- **FleetCostDuration runs the fleet-descent phase every `fd_period_work = 100_000_000` work units by default** (~150 s cadence on one modern core, the campaign-winning trigger pressure: 23-0 strict fleet wins at n=500, -35 routes over 30 pairs at n=1000, and the first three best-known solutions beaten on Blauth2024). The knob is dead code under Duration at any value (the phase only arms when the objective prices routes), so Duration behavior is unchanged by this default.
- **BREAKING (long Duration runs at defaults): the work-based restart is ON by default** (`restart_no_improvement_work = 1_000_000_000`, a deliberate ~25-minute stall window on one modern core at any instance size; the flat `restart_no_improvement = 20000` remains as a backstop, whichever fires first wins). Rationale: the flat default was effectively dead code on realistic budgets at n ≥ 500 (it never fired once across two Blauth2024 campaign nights), so stalled hours-scale searches never restarted; a shorter 250M window was tried and measurably hurt sub-half-hour budgets, so the shipped window only acts on genuinely long stagnation. Duration streams in runs long enough to accumulate a 1G-unit stall window diverge from 1.1.x at default parameters; pass `restart_no_improvement_work=0` to recover bitwise 1.1.x trajectories (the cross-version hex gate runs exactly that leg).

## [1.2.1] — 2026-07-27

### Changed

- **`dissolve_pct` default raised from 25 to 50.** The first FleetCostDuration benchmarking campaign on the Blauth2024 family (Grid'5000, 586 runs) ran a `dissolve_pct × max_perturbations` tuning grid: 50% dissolve pressure ranked first at every tested setting, 25% mid-pack, and disabling the kick was decisively worst (about +6% mean cost). Duration solves are unaffected (the kick stays disarmed when the objective does not price routes); FleetCostDuration runs that want the previous behavior pass `Params(dissolve_pct=25)` explicitly.

## [1.2.0] — 2026-07-26

### Added

- **The FleetCostDuration objective in the heuristic stack** (ILS and ACO), selected with `Params(objective="fleet_cost_duration")`: minimizes the canonical duration fold plus `fleet_fixed_cost × num_routes`, with the fixed cost read from the instance (MAMUT TD instances carrying the normative `fleet_fixed_cost` field, first family: Blauth2024). The value contract is unchanged: the returned cost is priced by the reference checker under the selected objective and must match bitwise, never within an epsilon. Scoring needs `mamut-routing-lib` ≥ 0.9.0; Duration-only use keeps working against the older pins.
- Fleet-aware local search: the commit accountant prices `fixed_route_cost` per non-empty route, so moves are accepted on duration + F·Δ(route count) and a relocate that empties a route is credited F (duration-increasing merges worth up to F are now tried, PyVRP-style).
- **Route-dissolve perturbation kick** (`Params(dissolve_pct=...)`, default 25% of kicks): removes one smallest route whole so its clients repair into the remaining routes — the additive credit alone cannot cross the empty-a-route plateau (the paper6/PyVRP lesson). Armed only when the objective prices routes.
- `Solution.objective`, and `to_benchmark_solution()` declares non-Duration objectives in the artifact metadata (feeds the per-objective BKS pipeline).
- `Instance.fixed_route_cost` in the compiled core (default 0), `IlsParams.dissolve_pct`, and a `dissolved` flag in the `ls_perturb` outcome tuple.
- Wheels for CPython 3.14. `requires-python` is now capped at `<3.15` so the admitted interpreters never outrun the wheel matrix (3.14 installs used to fall back silently to an sdist source build). Maintainer contact e-mail added to the packaging metadata.

### Changed

- Duration solves are bit-for-bit unchanged: at the default objective the fixed-cost term is an exact fold no-op, the dissolve kick consumes no rng draw, and 1.1.x trajectories reproduce bitwise (asserted by a regression test on instances that carry `fleet_fixed_cost`).
- `_core.ls_perturb` returns a 7-tuple (the added `dissolved` flag) — `_core` is an internal module; the public `kayros.solve` API is backward compatible.

## [1.1.3] — 2026-07-20

Documentation-only release. No code, no build, and no solver behavior changes: the wheels are functionally identical to 1.1.0.

### Added

- **`AUTHORS.md`**, the authoritative statement of authorship, scientific supervision, funding context, vendored third-party code, and contributor policy. Required by the HAL / Software Heritage deposit process: the Inria open-archive moderation team asks that an archived source repository carry an `AUTHORS` file describing the software's authors, so the Software Heritage snapshot referenced from HAL can be validated.
- An extended `Acknowledgements` section in `README.md`, now cross-linking `AUTHORS.md`: the ANR-MAMUT members (Adrien Pichon, Marc Sevaux, Alexandru-Liviu Olteanu), the PyVRP contributors (Leon Lan, Niels Wouda, Wouter Kool), and Thibaut Vidal.

### Fixed

- Two typos in the `README.md` acknowledgements (a missing preposition, and the spelling of Pénélope Aguiar Melgarejo).

## [1.1.2] — 2026-07-18

Documentation-only release. No code, no build, and no solver behavior changes: the wheels are functionally identical to 1.1.0.

### Added

- `uv` install commands alongside pip in `README.md`.
- An `Acknowledgements` section in `README.md`: the PhD supervisors (Romain Billot, Christine Solnon, Lina Fahed); Romain Fontaine for his help with Grid'5000 and for the TDTSPTW lineage this PhD follows up on ([tdtsptw-ejor23](https://github.com/romainfontaine/tdtsptw-ejor23)); Gonzalo Lera-Romero for open-sourcing the branch-price-and-cut solver the exact component builds on.
- A `Funding` section in `README.md`: ANR MAMUT project, ANR-22-CE22-0016.
- The Lera-Romero et al. 2020 reference now links the companion repository ([gleraromero/networks2020](https://github.com/gleraromero/networks2020)) and the author's GitHub profile.
- `date-released` in `CITATION.cff` and a `Changelog` project URL in the PyPI metadata.

## [1.1.1] — 2026-07-18

Documentation-only release. No code, no build, and no solver behavior changes: the wheels are functionally identical to 1.1.0.

### Fixed

- **Corrected the attribution of the vendored branch-price-and-cut solver.** `README.md` credited it in two places to "Lera-Romero, Rönnqvist & Ljungqvist (2020)". Rönnqvist and Ljungqvist are not authors of that work. The correct reference, as `cpp/lera/NOTICE.md` has always stated, is Gonzalo Lera-Romero, Juan J. Miranda Bront and Francisco J. Soulignac, *Linear edge costs and labeling algorithms: The case of the time-dependent vehicle routing problem with time windows*, Networks 76(1):24–53, 2020 ([doi:10.1002/net.21937](https://doi.org/10.1002/net.21937)). Since `README.md` is the PyPI long description, this release exists so that the corrected attribution reaches the package page. Apologies to the authors.

### Added

- A `References` section in `README.md` giving the full citations, with DOIs, for the three works the solver builds on (Lera-Romero et al. 2020, Visser & Spliet 2020, Blauth et al. 2024).

## [1.1.0] — 2026-07-18

The theme is **exact stepwise pricing**: on stepwise (value-jump) travel-time functions the exact component now runs a mollifier-free labeling that carries the value jumps exactly, closing the last soundness caveat of the certification pipeline.

### Changed

- **The exact value-jump labeling is the production path on stepwise ATFs.** Instances whose travel-time functions carry duplicate-abscissa value jumps (e.g. the Rifki2020 families) are auto-detected per solve and priced with the steps' verticals as tagged first-class objects in the piecewise-linear machinery (jump vs departure-choice verticals, attained values preserved), instead of being bridged by near-vertical segments. Three completeness defects in the formerly dormant exact scaffolding were root-caused by checker-refereed witness tracing and fixed: the label-extension composite erased position-dependent mandatory waiting where a departure plateau meets a same-abscissa jump (now the elapsed-time identity `(D − Id) ∘ dep + Id` on step-carrying arcs); the solution pool's vertex-set dedup could shadow a checker-cheaper ordering of the same customers (now path-keyed on the exact path); and the PWL `Compose`/`operator+`/`operator*` dropped or collapsed verticals at operand exhaustion (now attained-endpoint pairing throughout). Non-stepwise instances are bit-identical to 1.0.0. Full history: `cpp/lera/NOTICE.md` item 9, closing amendment.
- **Stepwise optimality stamping is enabled.** The single-run stamp refusal on stepwise instances (a guard on the retired mollified path) is lifted: `optimality_metadata` stamps stepwise certificates like any other. Promotion was gated on a validation ladder, all green: pinned reproducers certifying their checker-validated optima cold == warm, a clean differential fuzz sweep in both labeling modes, bit-identity on jump-free instances, cross-platform certified-value agreement (13/13, NixOS vs gcc-13/Debian on identical payloads), a 778-run full-family Grid'5000 sweep and a 1444-run four-solve re-certification, re-confirming 93 stored certificates at their exact stored values with zero unsound certificates, zero checker-infeasible priced columns and zero cross-run disagreements.

### Removed

- The forward-side stepwise mollifier (`_continuize_breakpoints`, the 1e-3 steep-bridge under-approximation): stepwise breakpoints now reach the solver verbatim. The reverse-side continuization helper remains for jump-free functions only, where the exact path bypasses it (bit-identity with 1.0.0 verified).

## [1.0.0] — 2026-07-17

First beta (development status Alpha → Beta). The theme is **honest verdicts under every resource frontier**: the prover already returned honest time-limit verdicts with valid bounds; this release closes the one remaining case where it could die without an answer.

### Added

- **Memory self-guard (graceful OOM self-rejection).** Full-horizon TDVRP pricing can accumulate labels past any node's RAM (the Vu2020 n≥59 pathology: the process was OS-OOM-killed with no verdict). The solve now polls an RSS watermark at the same interruption points as the time-limit deadline and unwinds cleanly with `exact_log.status == "MemoryLimitReached"`, honest bounds, and no optimality stamp. On by default: `solve_duration(memory_limit_mb=None)` resolves the limit from the machine (own RSS + ~80% of available memory, capped by the cgroup limit); pass an explicit value to override or `0` to disable. The result carries a `memory` record (`limit_mb`, `peak_rss_mb`, `limit_reached`). An armed, untripped guard is pure observation: values are bit-identical to a guard-off run.

### Removed

- The no-op `lera` packaging extra (a compatibility alias from when the exact component became part of the default build). `pip install kayros` has shipped the full solver for many releases; `pip install "kayros[lera]"` now warns about an unknown extra and installs the same thing.

## [0.5.0] — 2026-07-15

The theme of this release is **sound, audited optimality certificates** for the exact branch-price-and-cut component. The certificates issued by earlier versions did not survive scrutiny; this release repairs the underlying defects, hardens the certification protocol, and re-establishes the certified best-known-solution store from scratch under it.

### Fixed (soundness)

- **Pricing-ladder termination.** The escalation over pricing levels (heuristic cost, heuristic elementarity, exact) could close a node without ever running the exact level, so a certificate could be issued with zero exact-pricing iterations. Escalation is now driven by the number of columns actually added, not by an empty pool. This was the decisive defect; it invalidated every certificate produced since the checker-exact-column change and forced a full re-certification.
- **Uninitialized labeling-mode flag.** The bidirectional labeling's symmetric-mode flag was never initialized (the vendoring dropped the upstream wiring), reading per-build stack garbage. This is the mechanism behind the build-dependent certification observed during the campaign. The flag is now explicit and the mode is a controlled experimental arm.
- **Set-valued duration at departure-choice plateaus.** On stepwise arrival-time functions a plateau makes the departure at a given arrival a set, so a partial path's duration is the minimum over that set; two sites priced an arbitrary representative instead of the minimum, surfacing as material four-way disagreements. Fixed with an explicit minimum over the covering pieces (`PWLFunction::MinValueAt`).
- **Checker-infeasible priced columns no longer crash a solve.** A column the reference checker rejects now poisons that run and is skipped (poison-and-continue), and a poisoned run can never support a certificate, instead of aborting the whole solve.
- **Hole-tolerant composition** on interior domain gaps of post-domination duration functions (previously crashed on some stepwise instances once the exact level ran).

### Added

- **Tagged verticals** in the PWL engine (JUMP vs CHOICE) with exact graph reflections (`FlipTime`/`FlipValue`) replacing the fragile mollifier composition on the reflected paths; the acceptance gate for the symmetric merge on stepwise data now passes.
- **Multi-gate certification protocol** and its offline analyzer: four solves per instance (cold and warm starts crossed with the two labeling modes), value agreement at one checker-exact value not above the store, an audited exact-pricing census, a pricing-integrity guard, cross-platform agreement on stepwise families, and a direction-aware exact-arm witness.
- **Randomized differential fuzzer** and a per-label trace harness used to find and pin the defects above (`tests/td_fuzz.py`, `tests/test_prover_fuzz_soundness.py`, `KAYROS_TRACE_PATH`).
- Dormant, unit-tested exact value-jump labeling scaffolding (the principled replacement for the mollifier on stepwise pricing), not yet on the default path.

### Changed

- **Optimality certificates on stepwise (value-jump) instances are refused from a single solve** and are only issued under the audited multi-run campaign protocol; the producer guard enforces this.
- The definitive re-certification (all 1352 instances of the four legacy MAMUT families, both problem types) established the store at **468 proven optima, 170 of them checker-valid strict improvements**, with zero four-way disagreements remaining and every stamp carrying audited-protocol provenance.
- Dependency floor raised: `mamut-routing-lib>=0.6.0` (needs the `OptimalityMetadata` schema; the tested stack).

### Packaging

- The vendored Lera-Romero MIT attribution (`cpp/lera/NOTICE.md`) is now carried into the wheel's `.dist-info/licenses/` alongside the root `LICENSE`.

## [0.4.0] — 2026-07-08

- Default anytime strategy switched to `"ils"` (single-trajectory iterated local search) after a large head-to-head campaign; `Params(strategy="aco")` restores the 0.3.x solver.
- Local search of every strategy uses granular candidate lists (`num_neighbours=50`) by default; `Params(num_neighbours=0)` restores exhaustive enumeration.

## [0.3.0] — 2026-07-07

- Exact branch-price-and-cut component (`kayros.lera`) ships in the default build on the open-source HiGHS LP backend (the CPLEX backend stays a source-build opt-in). Checker-exact column costs and honest time-limit gap reporting.

## [0.2.0] — 2026-07-06

- Anytime heuristic solver (`kayros.solve`): greedy construction, MAX-MIN TD ant colony, and the time-dependent granular local search over the NDCPWLF engine; streaming incumbents under a hard deadline.

## [0.1.0] — 2026-07-06

- Initial public release: the NDCPWLF composition engine, a bit-identical C++ port of the reference checker's arithmetic, gated by an equivalence suite over the full benchmark set.

[1.1.3]: https://github.com/0nyr/kayros/releases/tag/v1.1.3
[1.1.2]: https://github.com/0nyr/kayros/releases/tag/v1.1.2
[1.1.1]: https://github.com/0nyr/kayros/releases/tag/v1.1.1
[1.1.0]: https://github.com/0nyr/kayros/releases/tag/v1.1.0
[1.0.0]: https://github.com/0nyr/kayros/releases/tag/v1.0.0
[0.5.0]: https://github.com/0nyr/kayros/releases/tag/v0.5.0
[0.4.0]: https://github.com/0nyr/kayros/releases/tag/v0.4.0
[0.3.0]: https://github.com/0nyr/kayros/releases/tag/v0.3.0
[0.2.0]: https://github.com/0nyr/kayros/releases/tag/v0.2.0
[0.1.0]: https://github.com/0nyr/kayros/releases/tag/v0.1.0
