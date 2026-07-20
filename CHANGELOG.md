# Changelog

All notable changes to KAYROS are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project aims to follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html). Certificate semantics and benchmark provenance are documented in `README.md` and `cpp/lera/NOTICE.md`.

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
