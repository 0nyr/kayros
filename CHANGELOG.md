# Changelog

All notable changes to KAYROS are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project aims to follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html). Certificate semantics and benchmark provenance are documented in `README.md` and `cpp/lera/NOTICE.md`.

## [Unreleased] (1.4.0.dev0)

Development version, not published to PyPI. It carries the anytime-latency work that came out of the plan-13 contender campaign, where the single lost n = 1000 panel turned out to be a first-incumbent latency artifact rather than a search deficit.

### Changed

- **The greedy seed no longer runs its multi-hop lookahead, and is roughly two orders of magnitude faster.** Through 1.3.0 every placement ran a full time-dependent Dijkstra over the remaining free customers and selected on the earliest MULTI-HOP ready time; the selection is now the earliest DIRECT ready time. This is the same rule up to ties: Dijkstra's first settled node is by construction the free customer with the smallest direct ready time, so the multi-hop minimum equals the direct minimum and is attained at the same customer. The two rules can only part where a detour reaches a customer in exactly the same total time, which needs a time window binding hard enough for the wait to absorb the detour, and there the lookahead is a different tie break rather than a better choice. Verified: identical route sequences on all 232 instances of the plan-13 campaign instance set, and on the checkout's test families as a standing gate. The construction drops from O(n^3) to O(n^2): 0.03 s instead of 36 s at n = 1000, 3.5 s to 0.007 s at n = 500, about 60x at n = 100. The removed rule is retained as `_core.greedy_makespan_lookahead`, used only as the tests' reference oracle.
- **The seed is published to `on_incumbent` the moment it is built**, before the first local-search descent instead of after it. Combined with the construction speedup, the anytime stream now opens in well under a second at every instance size, where 1.3.0 published nothing for about 100 s at n = 1000 (36 s seed plus ~65 s first descent). The stream stays strictly improving: the descended seed is published as a second `origin="greedy"` record only when it actually improves.
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
