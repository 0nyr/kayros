# NOTICE — vendored Lera-Romero BPC code (`cpp/lera/`)

This directory vendors the branch-price-and-cut (BPC) solver for the TDVRPTW originally written by **Gonzalo Lera Romero** (Grupo de Optimización Combinatoria, Departamento de Computación, Universidad de Buenos Aires), together with its supporting `goc` library, as adapted in Florian Rascoussier's `TDVRPTW-solver` repository.

The algorithm is described in: Lera-Romero, Miranda-Bront, Soulignac, *Dynamic programming for the time-dependent traveling salesman problem with time windows* (and the companion TDVRPTW BPC line of work published from the same codebase, Networks 2019 / INFORMS JOC).

## License

The upstream code is distributed under the **MIT License**, Copyright (c) 2019 Gonzalo Lera Romero (see `TDVRPTW-solver/LICENSE`, reproduced by kayros' own MIT `LICENSE` at the repository root). This vendored copy retains that license.

## Provenance

Vendored on 2026-07-06 from `TDVRPTW-solver` git commit `35f1e3b` (local master):

- `goc/` — Lera-Romero's `goc` library (`code/goc`, sans its `CMakeLists.txt`: kayros owns the build). Includes Onyr's later updates (Digraph API renames, PWL fixes, NDCPWLF-era changes).
- `goc/include/goc/vrp/` and `goc/src/vrp/` (`Route`, `VRPSolution`) — restored from commit `8329844` (2019-08-13, Lera-Romero's last "all features working" state); these files had been deleted in the 2025 objective-function refactor while the BPC still depends on them.
- `nyr/` — Florian Rascoussier's `nyr` layer (`code/nyr`): TD `VRPInstance` (tau/pretau/dep/arr PWL matrices, LDT, unreachability), objective-templated routes, NDCPWLF, time utilities.
- `main/` — the BPC proper (`code/main`): `bcp/` (BCP, SPF, pricing problem), `labeling/` (bi/monodirectional labeling, lazy labels, PWL domination, ng-neighborhoods), `preprocess/` (instance preprocessing). Heuristics and standalone `main_*.cpp` entry points were not vendored (kayros has its own).
- `magic_enum/` — vendored header-only dependency (MIT, Daniil Goncharov), as carried by upstream.

## Local modifications (kayros)

Kept deliberately minimal; the reference for algorithmic behavior is Lera-Romero's `8329844` plus Onyr's half-port intent:

1. `goc/include/goc/goc.h`: re-added the two `goc/vrp/*` includes (match `8329844`).
2. `main/include/bcp/bcp.h` + `main/src/bcp/bcp.cpp`: the half-ported `nyr::VrpSolutionRecord&` parameter (a type that no longer exists) was replaced by (a) restoring Lera's original `Run(goc::VRPSolution*)` signature, and (b) a new optional `on_incumbent` callback member invoked on every new upper bound (anytime surfacing, plan 2 Stream 5 M5.0/M5.2).
3. Build: this subtree is compiled by kayros' own CMake (`KAYROS_WITH_LERA=ON`), never by upstream's CMakeLists.
4. **HiGHS backend (M5.1, kayros-added)**: `goc/{include,src}/goc/linear_programming/highs/` — `MatrixFormulation` (solver-agnostic in-memory `Formulation`) + `highs::solve_lp`/`solve_bc` mirroring the cplex pair, stateless materialization per solve (duals postsolved by HiGHS; no presolve weakening needed; single-threaded per kayros' one-run-one-thread convention). Compile-time switch `LERA_LP_BACKEND=cplex|highs` in `lp_solver.cpp`/`bc_solver.cpp` (`GOC_LP_BACKEND_HIGHS`); unused cplex includes dropped from `cg_solver.cpp`; `variable.h` gained `friend class MatrixFormulation`. Unsupported in the HiGHS `solve_bc` (fail loudly, unused by the BPC path): MIP starts, branch priorities, cut callbacks, lazy constraints. Known perf gap vs CPLEX: no simplex-basis warm start across CG iterations (rebuild per solve) — revisit at M5.2+.
5. `main/src/labeling/bidirectional_labeling.cpp`: `RouteDuration` → `goc::Route` conversion at the solution-pool exit; `nyr/include/nyr/vrp/delta.h`: explicit `SubpathHash` (GCC 15).
6. **Stopping / anytime compliance (M5.2, kayros-added)**: new `goc/include/goc/time/deadline.h` (`goc::Deadline`, absolute wall-clock deadline). `BCP` derives one deadline from `time_limit` at `Run()` and every component takes its residual budget from it instead of nested stopwatch re-derivations: CG (`ProcessNode`), strong-branching LPs (`EstimateBound`, which previously ran **unlimited** — `solve_colgen` reset the shared `LPSolver.time_limit` to `Duration::Max()`), the freeze heuristic (previously got the **full** TL as residual), and the root cut loop. New interruption points: `SeparateCuts` O(n³) enumeration, `IterativeMerge`/`LastArcMerge` in the bidirectional labeling, and the labeling's solution-pool repricing (post-deadline tail cut). Correctness guards: an aborted strong-branching probe or truncated pricing pass stamps `TimeLimitReached` so a deadline hit can never masquerade as an optimality proof (including `LastArcMerge` truncation no longer reporting `Finished`, and the `z_lb = z_ub` gap-close now conditional on a fully explored tree); the freeze heuristic accepts a TL-truncated MILP incumbent when one exists. `solve_colgen` now takes the CG optimum from the last in-loop LP solve instead of an unconditional re-solve (which wasted a solve and leaked `Duration::Max()` into later LPs).
7. **Warm start (M5.3, kayros-added)**: `BCP::SetInitialIncumbent(double)` — public setter for an initial UB known by the caller (the value of warm-start columns the bridge adds via the pre-existing `SPF::AddRoute` injection point). The bound must be in the master's own arithmetic (checker-exact since item 8; a bound tighter by dust than the master's costs could prune the true optimum) and no incumbent callback fires for it.
8. **Checker-exact column costs (M5.6 stage A, kayros-added, bridge-side)**: every route entering the SPF (seeds, warm-start columns, priced columns) is repriced by the kayros checker-exact fold (`cpp/core/route_eval` over the payload's raw MAMUT ATFs — Lera's preprocessing folds service/waiting into travel times, so the fold runs on the verbatim data), making all master objective coefficients, LP bounds, UBs and the reported value checker-exact; the final value is re-summed in the checker's canonical route order (bit-identical to `compute_solution_cost`). Lera's `BestDurationRoute` remains the *search* arithmetic (pricing reduced costs, labeling); safety: pricing emits only reduced costs < −1e-6 (goc epsilon) while the Lera-vs-checker cost dust is ~1e-12, so priced columns stay strictly improving under checker costs, and a checker-infeasible priced column fails loudly (certification integrity). **Pricing completeness is still modulo Lera's epsilon arithmetic** (a checker-optimal route could in principle be dominated away by dust inside labeling) — removing that residue is M5.6 stage B (labeling on kayros `cpp/pwlf`), pending.

Behavioral changes inherited from the half-port (upstream `TDVRPTW-solver`, not introduced here): subset-row cut violation threshold raised from `> 0` to `> 0.1` in `BCP::SeparateCuts`.

## Conventions

This subtree is **vendored code**: kayros' plain-old-data / style conventions do not apply here. Keep diffs against upstream minimal and documented in this NOTICE.
