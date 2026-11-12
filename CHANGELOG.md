# Changelog

All notable changes to KAYROS are recorded here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the project aims to follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html). Certificate semantics and benchmark provenance are documented in `README.md` and `cpp/lera/NOTICE.md`.

## [1.7.1] — 2026-11-12

A maintenance release that changes no behaviour. Search, pricing, numerical code paths, default values and the keys of the result dictionaries are those of 1.7.0, and on the 15 Dabia2013 instances compared with both LP backends the exact component returns the same results as 1.7.0 (see Validation below). The release edits comments, documentation, test and build-target names and package metadata, and it adds tests and CI steps. The component study of the exact solver used 1.7.0, and its run records store the content hash of `cpp/` and `src/` of that version (`44c84cdb…bf69`). The hash of these trees in 1.7.1 is `0bd3a8d9…785b`, because comments and build targets changed.

### Added

- **The C++ unit tests of `tests/cpp` are built and run.** They were compiled by no target although `cpp/lera/NOTICE.md` cites them. Each test is a CMake target `kayros_test_<name>`, excluded from the default build and from the wheels, and `cmake --build <build dir> --target kayros_cpp_tests` builds and runs all of them for the configured LP backend. CI runs them on the HiGHS backend.
- **A unit test of `PWLDominationFunction::DominatePieces` on a decreasing piece** (`tests/cpp/test_partial_dominance.cpp`), the routine repaired in 1.7.0. It fails when the repaired line is reverted to `p1.image.right`.
- **The publish workflow solves an instance with each wheel before upload** (`tests/wheel_smoke.py`: an anytime solve and a cold exact solve of Dabia2013 C101 with 25 customers). It used to import the modules only. The wheel job of CI runs the same script after the test suite.

### Changed

- **Package metadata.** The 4 authors are listed in the same form, names without email, so that the `Author` field of the core metadata holds all of them in citation order. Florian Rascoussier is the maintainer, with the contact address. In 1.7.0 the first author was the only one with an email and appeared apart from the others, in `Author-email`.
- The CMake target of the CPLEX variable-domain test is renamed `kayros_test_cplex_variable_domains`.
- `README.md`: a short status statement and a `Versions` section with one entry per release, absolute links (the README is the PyPI description), and the version and commit used by the component study.

### Documentation

- A set `KAYROS_BPC_*` environment variable takes precedence over the matching argument of `solve_duration` without any warning. The recorded `effective_algorithm_settings["pricing_options"]` holds the values in force, under the names of the variables, so `he_cutblind` is the mode of `heuristic_cut_blind` and `lam_iterate` records the queue-iterating last-arc merge. The keys are unchanged.
- The mean customer dual that scales `near_rel` is the sum of the positive customer duals divided by the number of customers.
- `KAYROS_FIRST_BRANCH_TRACE`, a diagnostic that writes the first branching decision of a process to a file, is documented in `cpp/lera/main/include/bcp/branch_trace.h`.
- How to check that link-time optimization took effect in a source build (`README.md`, Install). CMake turns it off with a status line when the toolchain does not support it, and the solver records nothing about it.
- `cpp/lera/NOTICE.md` item 14 now describes the CPLEX batching correctly, as the restoration of variable types after an LP relaxation. The comment on `KAYROS_BPC_VERIFY_FINAL` describes the bit mask the code reads. Development labels are removed from comments.

### Known issues

- **The order of the returned routes is not reproducible.** `result["routes"]` and the routes of the incumbent records are listed in the iteration order of a hash container keyed by memory addresses, so the order can differ between two runs of the same build, and between 1.7.0 and 1.7.1 when the Python files allocate differently. The routes themselves are the same, and `value` is summed in the canonical route order. This predates 1.7.1 and is left unchanged here, since sorting the routes would change the output.

### Validation

- Equivalence with 1.7.0. Both versions were built in separate environments, with the CPLEX 12.10 backend and with the HiGHS 1.14.0 backend, and the exact component was run cold on 15 Dabia2013 TDVRPTW instances: C101, C104, C109, R101, R105, R111, RC102 and RC105 with 25 customers, and C101, C102, R103, R106, R110, RC101 and RC107 with 50 customers. All 30 pairs end with the status `Optimum` at the same value, and 9 of them branch (3 to 15 nodes). In every pair the result dictionaries have the same keys, and every value is equal (node counts, cut counts, pricing counters, label counts of each pricing iteration, bound events, columns) except timing and memory fields and the order of the returned routes (see Known issues).
- The comment-stripped C++ of every changed source file is identical to 1.7.0, the Python modules have the same syntax tree once documentation strings are removed, and the compile and link flags of `_core`, `lera` and `_lera` are those of 1.7.0 except the version string compiled into `_core`.
- Test suite on the CPLEX build with a full local copy of the benchmark store: 1,026 passed and 3 opt-in skipped, the same counts as 1.7.0 on the same store. Exact-component tests on the HiGHS build (`tests/test_lera*.py`, `tests/test_prover_fuzz_soundness.py`, `tests/test_goc_pwl_stepwise.py`): 66 passed and 3 opt-in skipped, the same as 1.7.0.
- `kayros_cpp_tests` passes on both backends (5 tests on HiGHS, 6 on CPLEX). `tests/wheel_smoke.py` passes on a locally built HiGHS wheel. The cibuildwheel jobs of `ci.yml` and `publish.yml` were not run locally.

## [1.7.0] — 2026-11-11

An exact-solver release. It repairs two soundness defects of the exact component, the partial-dominance routine inherited from the upstream branch-price-and-cut code and the addition of a scalar point to a duration function. It also turns the repaired label correction on and makes the proof phase faster. Certificates produced by earlier versions came from the defective routines, as the first two entries below explain. The heuristic component is unchanged.

### Fixed

- **Partial dominance could discard an undominated part of a label, so the exact solver could close above the optimum.** When a stored label dominates a candidate on an interval lying strictly inside one linear piece of the candidate's duration function, the piece is split and its right part rebuilt. The right part was built with the maximum of the piece as its value at the right end. That is only correct on a non-decreasing piece. On a decreasing piece, the common case where a later arrival means less waiting, the rebuilt part was too high, and a later comparison could erase it although it was not dominated. The Exact pricing level could then finish without reporting a route of negative reduced cost that exists, and a node bound could exceed the optimum. The defect is in the upstream code this component derives from. It only ever removes labels: every column the solver finds is feasible and exactly priced, and incumbents are unaffected. Bounds and optimality claims are what could be wrong. The reproducer is Dabia2013 with 50 customers, instance RC107, which closed at 10632.144948 on one pricing trajectory while the optimum is 10614.011180. `tests/test_lera_partial_dominance.py` pins it. On the instances re-run with both builds so far (33 Dabia2013 instances with 100 customers, and 72 smaller Vu2020, Ari2018 and Rifki2020 instances at a 45 s limit) no proven optimum changed. That is a sample, not a clearance of earlier certificates. A later study on the 56 Dabia2013 instances with 100 customers (5 runs of 7,200 s per configuration) proved again, at the stored value, the 25 instances of that set that hold a stored certificate. The other stored certificates have not been re-derived with this version.
- **Adding a scalar point to a duration function could discard attainable lower durations.** On jump-free instances a label duration can carry several stacked departure choices at one abscissa. The sum with a function reduced to a single point advanced past the first choice, so a tight extension lost a route and the solver could close above the optimum. The reproducer is Vu2020 `Vu-A2-pB-d98-w100` with 59 customers, which closed at 1847.239598 while a checker-valid solution costs 1846.807019. The sweep now holds the scalar operand while the other operand still has stacked choices at that abscissa. As for the entry above, incumbents are unaffected and bounds and optimality claims are what could be wrong.
- **The lower bound reported at a limit could ignore an interrupted branch.** When a time or memory limit stopped the search while a node was being branched, the children of that node were not in the queue, and `best_bound` was still raised to the best bound of the queue. The bound now advances from the queue only while no limit has been hit, and never above the incumbent value. A limit never yields the status `Optimum`, so optimality claims were not affected. A gap reported at a limit by an earlier version can be too small.

### Added

- **Near-zero reduced-cost columns** (`solve_duration(near_rel=0.10)`, `0` disables). An Exact pricing pass that found at least one new negative column also contributes up to 3,000 routes whose reduced cost lies in `[0, near_rel * mean customer dual)`. The labeling never prunes on reduced cost, so these routes cost only merges, and they spare Exact passes near convergence, where each pass used to return a handful of columns. A pass without a new negative column discards them, so "no new column" keeps its meaning for termination.
- **Cut-blind dominance at the heuristic elementarity level** (`solve_duration(heuristic_cut_blind=True)`). With many active subset-row cuts the cut-memory correction blocks nearly every dominance, and a heuristic pass became slower than an Exact one (from 2 s to several minutes per pass with about 200 cuts). The level is a heuristic, and the columns it finds are still priced exactly.
- **`KAYROS_BPC_VERIFY_FINAL`**, a diagnostic bit mask. Before a node is declared optimal, the Exact pass that found nothing is run again with the masked features changed (label correction, merge state, cost sorting, unreachable sets, partial dominance, limited extension, execution filters, relaxation). A column found by the second pass is a column the first one lost. This is how the defect above was located.
- **Progress events and search records.** `solve_duration(on_event=...)` takes a callback that receives the events of the search as dictionaries. The result gains `bound_events` (the lower bound over time), `closure_census` (counts of nodes by column-generation outcome, among them the nodes whose last pricing pass was not a finished Exact one), `exact_pricing` (Exact passes started and finished) and `effective_backend_settings` (the LP backend, its version and the settings in force).
- Experimental pricing levers, off by default and reachable through `KAYROS_BPC_*` environment variables only: restart at a heuristic level after a productive Exact pass, early cut separation, gating of these on the cost of the last Exact pass, and the Exact level as a decremental state-space relaxation. None changes the termination rule. None showed a consistent gain after the fix above.

### Changed

- **Faster Exact pricing with identical search.** A contiguous copy of each stored label's sort key and unreachable set serves the dominance and correction scans (packed dominance filter). A merge is rejected before its duration function is built when a lower bound including the shared cut penalties already fails the test. The branch-price-and-cut library is built with `-fno-semantic-interposition` and link-time optimization (CMake option `KAYROS_LERA_IPO`, on by default), which lets the compiler inline the small accessors of the piecewise-linear classes. Measured on Dabia2013 C104 with 100 customers: 358 s to 196 s per Exact pass for the filter, then 172 s to 143 s for the build settings, with identical label counts.
- **Label correction is repaired and on by default.** The upstream code left its correction flag uninitialized, and its correction step could free a label that was still a parent and kept stale sort keys in the merge index. In the repaired step only a label retained for processing corrects stored labels, removed labels stay owned until the pricing call ends, and the dominance and merge buckets are sorted again when keys change. It requires lazy extension. `KAYROS_BPC_CORRECTION=0` turns it off, and the effective value is recorded in `effective_algorithm_settings["label_correction"]`.
- **Reproducible branching and cut separation.** Arc flows and cut violations were summed in the iteration order of a hash-based container, which depends on memory allocation. A difference in the last bit could replace 4 of the 10 strong-branching candidates between two runs of the same instance. The sums now run in variable order.
- **Explicit LP backend settings.** Every LP and MIP call now sets its tolerances instead of inheriting the defaults of the backend. HiGHS runs with primal and dual feasibility tolerances of 1e-7, a MIP feasibility tolerance of 1e-6 and a relative MIP gap of 0. The opt-in CPLEX backend runs on 1 thread with feasibility and optimality tolerances of 1e-7, an integrality tolerance of 1e-6 and a relative MIP gap of 0, and restores variable types after an LP relaxation in one call instead of one call per variable.
- **The last-arc merge walks the leftover forward queue instead of popping it.** Same merges, in a different order, which can change which routes fill a saturated pool.
- The result records the effective options in `effective_algorithm_settings["pricing_options"]` and work counters in `pricing_counters`. Each pricing iteration of the root log carries `pricing_*` fields (negative and near-zero routes, extended labels).
- Package metadata (`pyproject.toml`, `CITATION.cff`) lists the four authors of `AUTHORS.md`.

### Known issues

- **Dabia2013 RC105 with 100 customers and the HiGHS backend.** Every run of this instance ended with the status `Optimum` at the stored optimum and with `checker_infeasible_columns` equal to 1, which means that the labeling priced one route that the reference checker rejects. Such a route is skipped and counted, and a result with a nonzero count must not be used as a certificate. The count was 0 on the same instance with the CPLEX backend. It was observed in 5 runs of 5 with this version and in a development build that preceded it. The cause has not been investigated, and we do not know whether earlier releases show it.

## [1.6.1] — 2026-10-14

A checker-compatibility patch. Solver algorithms and numerical evaluation are unchanged. New installations require `mamut-routing-lib>=0.6.0,<0.12`, matching the historical TD contract implemented by KAYROS.

### Fixed

- **Exclude incompatible reference checker contracts.** The previous open-ended dependency allowed mamut-routing-lib 0.12.0, whose `td-fold/2` evaluation can differ from KAYROS's `td-fold/1` cost by a final floating-point bit. KAYROS's strict final objective check then rejected internally generated solutions, including reported Dabia2013 C101 and C103 cases at n=25. The temporary upper bound excludes 0.12 and later releases until compatibility is validated. Existing installations should upgrade KAYROS or explicitly install `mamut-routing-lib==0.11.0`.
- **Re-evaluate stored routes in the checker-equivalence tests.** Stored BKS costs may belong to another checker contract. The ordinary and penalized evaluation tests now compare the C++ total with the installed compatible checker's evaluation of the same routes, rather than assuming the latest benchmark snapshot's declared cost uses that contract.

## [1.6.0] — 2026-08-08

A fleet-squeeze release: the penalty-tolerant time-window squeeze (the S2 phase polish and the S1 in-ladder rescue) and the K-cap confined search arrive as **opt-in campaign features, inert by default**. The inert default is a measured decision, not caution. In the plan-15 validation campaigns (paired off-versus-armed, bitwise-identical starts), arming the squeeze at the calibrated operating point (`sq_penalty=2.0`, `sq_work_cap=30000000`, `sq_on_nodrop=True`, `sq_ladder=True`) reached the BonnTour reference fleet in 4 of 30 cold 12 h cells at n=1000 where the default reached it in 0 of 30, removed a net 12 routes over the 30 pairs and improved the paired mean FleetCostDuration by 1.32 percent; but the same campaigns surfaced K-endgame variance in both directions (short 2 h cells at n=500 degraded the armed paired mean, and one 12 h pair ended with the armed run stalled one route above its twin), so there is no size at which arming is unconditionally safe and campaigns arm it explicitly. Default FleetCostDuration streams stay bitwise pre-squeeze (`sq_work_cap=0`), and Duration streams are untouched at any setting. Armed and warm-started from the stored records, this build's release campaign refreshed 29 of the 30 Blauth2024 FleetCostDuration best-known solutions (the thirtieth held), and 90 companion searches capped one vehicle below the records all ended Infeasible without ever attaining the reduced fleet, evidence that the stored route counts are tight at a 12 h single-core budget.

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
