# KAYROS

[![PyPI](https://img.shields.io/pypi/v/kayros)](https://pypi.org/project/kayros/) [![SWH](https://archive.softwareheritage.org/badge/origin/https://github.com/0nyr/kayros/)](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros)

**KAYROS** is an exact & anytime solver for **duration-minimization time-dependent vehicle routing** problems, TDVRPTW (with time windows) and TDVRP, benchmarked on the canonical [MAMUT-routing](https://github.com/ANR-MAMUT/MAMUT-routing) TD instance families.

The name is a nod to [*Kairos*](https://en.wikipedia.org/wiki/Kairos), the ancient Greek notion of the *right, opportune moment*, fitting for a time-dependent solver where *when* each route departs is itself a decision. It is also a [recursive acronym](https://en.wikipedia.org/wiki/Recursive_acronym): **K**ayros **A**nytime-**Y**ielding **R**outing **O**ptimization **S**olver.

> Status: **beta (v1.7.1)**, developed as part of a PhD. KAYROS offers two solving modes on one exact time-dependent engine: an anytime iterated local search, which produced most of the best-known solutions of the MAMUT-routing store, and an exact branch-price-and-cut component (`kayros.lera`) that issues checker-exact optimality certificates. Every run ends with an honest verdict (optimum, time limit, or resource limit), never a silent kill.

## Install

```sh
pip install kayros    # or: uv add kayros / uv pip install kayros
```

This pulls everything, including the benchmark loaders and the reference checker ([`mamut-routing-lib`](https://pypi.org/project/mamut-routing-lib/)). Since version 1.6.1, KAYROS requires `mamut-routing-lib>=0.6.0,<0.12`: KAYROS implements the historical `td-fold/1` contract, while checker 0.12 introduced `td-fold/2`. A mixed environment can fail the final objective-equality check even when the values differ by only a final floating-point bit. Upgrade existing environments with `python -m pip install --upgrade kayros`, or pin `mamut-routing-lib==0.11.0` when retaining an older KAYROS release. Newer checker versions will be admitted after numerical compatibility is validated. For development:

```sh
git clone https://github.com/0nyr/kayros && cd kayros
pip install -e . --group dev    # pip >= 25.1 (or: uv pip install -e . --group dev)
```

Requirements: Python ≥ 3.11. Building from source (sdist or checkout) additionally needs a C++23 compiler, CMake ≥ 3.26 (fetched automatically by the build backend when missing) and Boost.Graph headers+library; the HiGHS LP solver is fetched and built statically by CMake when no install is found. The exact component is built with link-time optimization when the toolchain supports it (CMake option `KAYROS_LERA_IPO`, on by default). CMake turns it off with the status line `Lera BPC: link-time optimization unavailable` when the toolchain lacks it, and the solver gives no other sign. To check a source build, keep its build directory (e.g. `pip install . -C build-dir=build`) and look for `-flto` in `build/build.ninja`, or for that status line in the build output.

## Usage

Anytime heuristic solve: iterated local search over granular candidate lists, streaming every new incumbent:

```python
import kayros

# Any MAMUT-routing TD instance (.vrp.json with its .atf.json sidecar next to it)
instance_path = "benchmarks/TDVRPTW/Dabia2013/n=25/C101.vrp.json"
solution = kayros.solve(instance_path, time_limit=10.0, seed=42)
print(solution.duration, solution.num_routes, solution.status)

# Anytime: react to every new incumbent while the solve keeps running
def on_incumbent(incumbent, routes):
    print(f"[{incumbent.seconds:7.2f}s] {incumbent.value:.6f} ({incumbent.origin})")

solution = kayros.solve(instance_path, time_limit=60.0, on_incumbent=on_incumbent)
```

Instances are loaded without re-verifying their artifact digests, because materialization is deterministic and the check costs minutes and gigabytes of peak memory at n = 1000. Pass `verify=True` (to `kayros.solve` on a path, or to `kayros.io.load_instance`) on test runs and on the first run over data you have not used before: it re-derives the sidecar digests and the ATF digest and pins them against what the instance file declares, which is what catches a truncated download or a mismatched sidecar. Without it those failures are silent.

The default strategy is `"ils"` (single-trajectory iterated local search), picked over the alternatives in a 20,808-run head-to-head across five TD families at n=10..1000, with the margin growing with instance size. A MAX-MIN TD ant colony remains available as `Params(strategy="aco")`, and `Params(num_neighbours=0)` restores exhaustive (non-granular) local-search enumeration; both are alternatives for experimentation, not defaults.

Families that price vehicles (a normative `fleet_fixed_cost` field on the instance, e.g. [Blauth2024](https://github.com/ANR-MAMUT/MAMUT-routing)) can be solved under the **FleetCostDuration** objective: the same canonical duration fold plus `fleet_fixed_cost × num_routes`, priced bitwise by the reference checker. The fleet term is part of `solution.duration`; the local search then trades route dissolves against duration, helped by a route-dissolve perturbation kick.

```python
params = kayros.Params(objective="fleet_cost_duration")
solution = kayros.solve(instance_path, params, time_limit=10.0, seed=42)
print(solution.duration, solution.num_routes)  # cost includes the fleet term
```

Exact solve: branch-price-and-cut with checker-exact certificates, optionally warm-started from a known solution (the fast path when certifying near-optimal solutions, e.g. stored best-known ones):

```python
from mamut_routing_lib.td import load_td_instance
from kayros.lera import solve_duration

loaded = load_td_instance(instance_path)
result = solve_duration(loaded, time_limit_s=600.0,
                        initial_routes=[list(r) for r in solution.routes])
print(result["exact_log"]["status"], result["value"])
# status == "Optimum" with routes == [] means: the warm-start solution itself
# is proven optimal. On a time limit, result["exact_log"]["best_bound"] is a
# valid global lower bound when the root relaxation finished (absent otherwise).
```

`solution.duration` and `result["value"]` are always values computed by the reference checker (`mamut_routing_lib.td.check_td_solution`), never an internal approximation.

The exact component reads `KAYROS_BPC_*` environment variables, meant for experiments (they are documented in `cpp/lera/bridge/lera_bridge.cpp`). A set variable takes precedence over the matching argument of `solve_duration` without any warning, e.g. `KAYROS_BPC_NEAR_REL` over `near_rel` and `KAYROS_BPC_HE_CUTBLIND` over `heuristic_cut_blind`. The values in force are recorded in `result["effective_algorithm_settings"]["pricing_options"]`, under the names of the variables, so two keys differ from the parameter names: `he_cutblind` is the mode of `heuristic_cut_blind` and `lam_iterate` records the queue-iterating last-arc merge. The threshold of `near_rel` is relative to the mean customer dual, i.e. the sum of the positive customer duals divided by the number of customers.

Resource limits are verdicts, not crashes: besides the hard time-limit deadline, the exact solve carries a **memory self-guard**: an RSS watermark polled at the same interruption points as the deadline. When pricing would outgrow the machine (full-horizon TDVRP labeling can exceed any node's RAM), the solve unwinds cleanly with `exact_log.status == "MemoryLimitReached"`, honest bounds, and no certificate. The default limit is resolved from the machine (own RSS + ~80% of available memory, capped by the cgroup limit); `solve_duration(memory_limit_mb=...)` overrides it, `0` disables.

## Design

KAYROS is two solving modes on one exact time-dependent engine:

- **The engine** (`cpp/pwlf`, `cpp/core`) represents arrival times as non-decreasing continuous piecewise-linear functions (NDCPWLF) and evaluates routes by exact function composition: a bit-identical C++ port of the reference checker's arithmetic (gated by an equivalence suite over the full benchmark set).
- **The anytime stack** (`kayros.solve`): greedy construction and a single-trajectory TD iterated local search (granular ruin-and-recreate kicks, late-acceptance hill climbing, restart-to-best), with a TD ant colony as an alternative strategy, over one time-dependent local-search layer using LCA-BST move evaluation (Blauth et al. 2024) with granular candidate lists: tree-ranked relocate/swap/2-opt\* where every *accepted* move is repriced by the checker-identical fold before it counts.
- **The exact component** (`kayros.lera`): the branch-price-and-cut solver of Lera-Romero, Miranda Bront & Soulignac (2020), vendored under `cpp/lera/` (see its `NOTICE.md`) on the open-source [HiGHS](https://highs.dev/) LP backend, extended with deadline-compliant anytime behavior, warm starts through columns, TDVRP support, honest time-limit gap reporting, and the memory self-guard. Every column entering the master problem is repriced in the checker's arithmetic, so reported values (and optimality certificates) are checker-exact: *optimal under checker-exact route costs and standard LP/pricing tolerances, completeness modulo the search engine's epsilon dominance*. On stepwise (value-jump) travel-time functions the labeling switches automatically to an exact value-jump path: the steps' verticals travel through the piecewise-linear machinery as tagged first-class objects (jump vs departure-choice verticals, attained values preserved) instead of being smoothed into steep bridges, so pricing stays complete and stepwise certificates are single-run checker-exact certificates like any other. This path was promoted after a validation ladder ending in a 1444-run four-solve re-certification campaign that re-confirmed 93 stored certificates at their exact stored values with zero checker-infeasible priced columns (`cpp/lera/NOTICE.md` item 9 records the full history). Turning the LP dual bounds themselves into rigorous certificates (safe bounding) is future work.

## Core principles

- **The checker is the referee.** Every solution and every certificate is priced by the reference checker of `mamut-routing-lib`; the checker's value is the value.
- **Exact arithmetic.** Plain IEEE-754 doubles, no epsilon comparisons in the engine, no FMA contraction (`-ffp-contract=off`); results are bit-reproducible across machines.
- **Anytime first.** Time budgets are hard deadlines honored by every component (heuristics and exact search alike), and incumbents stream out as they are found; a solver that only answers at the end is not a solver you can interrupt.
- **Honest verdicts.** A run ends with an answer: optimum, time limit, or resource limit, with valid bounds where they exist. The solver is never OOM-killed mid-certificate, and it never claims more than its arithmetic supports.
- **One-command install, no proprietary dependency.** The default build, including the exact component, is pure open source; HiGHS is built statically into the wheels. The faster CPLEX backend for the BPC remains strictly a source-build opt-in (`-DLERA_LP_BACKEND=cplex`) and never ships in wheels.
- **One run is one thread.** No intra-run parallelism; parallelism belongs to the experiment layer above.
- **POD core.** The fresh C++ is plain structs, flat arrays and free functions: optimization-kernel style, no framework (the vendored BPC keeps its upstream style, contained under `cpp/lera/`).

## Versions

Most recent first. The [changelog](https://github.com/0nyr/kayros/blob/main/CHANGELOG.md) gives the details and the evidence behind each entry.

- **1.7.1** (2026-11-12). Documentation, tests and packaging metadata only. Solver behaviour and results are identical to 1.7.0.
- **1.7.0** (2026-11-11). Repairs two defects under which the exact component could close above the optimum: a partial-dominance step inherited from the upstream branch-price-and-cut code, and the addition of a scalar point to a duration function. The lower bound reported at a limit no longer ignores an interrupted branch. Label correction is repaired and on by default, branching is reproducible, and Exact pricing is faster (near-zero reduced-cost columns, cut-blind dominance at the heuristic elementarity level, a packed dominance filter, link-time optimization).
- **1.6.1** (2026-10-14). Requires `mamut-routing-lib>=0.6.0,<0.12`, the checker versions that implement the `td-fold/1` contract of KAYROS.
- **1.6.0** (2026-08-08). An opt-in penalty-tolerant time-window squeeze and a K-capped confined search, both inert by default. Armed and warm-started, this build refreshed 29 of the 30 Blauth2024 FleetCostDuration best-known solutions, and all 30 are now held by KAYROS. The fleet-descent drain is capped by work instead of wall-clock time (breaking: `fd_time_cap_seconds` is replaced by `fd_work_cap`).
- **1.5.2** (2026-08-07). Loading no longer re-verifies the artifact digests by default. `verify=True` runs the check.
- **1.5.1** (2026-08-05). Repairs an over-certification of jump-free instances by the exact component of 1.1.0 to 1.5.0. A re-certification campaign on the repaired build re-derived 466 of the 468 stored certificates at their stored values. The 2 others are a certificate retracted after a checker-valid counterexample and an instance refused by the protocol.
- **1.5.0** (2026-08-04). K-diverse seeding (`seed_k_factor=2.0`). Under Duration the search starts from a fleet twice as large as the constructed one and shrinks it. At a fixed time limit on a dense family with tight time windows, this lowers the duration by 5.2 % on average, with gains of up to 27 % on single instances.
- **1.4.0** (2026-08-04). The first solution is published in well under a second at every instance size, where 1.3.0 published nothing for about 100 s at n = 1000.
- **1.3.0** (2026-07-29). A fleet-descent route-elimination phase for FleetCostDuration, the mechanism behind the first best-known solutions beaten on Blauth2024, and work-based triggers.
- **1.2.0 and 1.2.1** (2026-07-26 and 2026-07-27). The FleetCostDuration objective (a fleet fixed cost plus duration, as priced by the Blauth2024 family) with a route-dissolve perturbation, and wheels for CPython 3.14.
- **1.1.0 to 1.1.3** (2026-07-18 to 2026-07-20). Exact pricing on stepwise travel-time functions (1.1.0), then documentation releases (corrected attribution of the vendored solver, references, `AUTHORS.md`).
- **1.0.0** (2026-07-17). First beta, with the memory self-guard of the exact component.
- **0.5.0** (2026-07-15). Audited optimality certificates and the multi-gate certification protocol, under which the store was re-certified at 468 proven optima.
- **0.1.0 to 0.4.0** (2026-07-06 to 2026-07-08). The engine, the anytime solver, the exact component on HiGHS, and iterated local search as the default strategy.

The proven optima of the MAMUT-routing store (704 as of 2026-08-08, among them the first 238 certificates of the Poryos2026 families) were issued by versions before 1.7.0, and they have not all been re-derived since the 1.7.0 repairs (see the 1.7.0 entry of the changelog).

## Archival and reproducibility

`kayros` is archived by [Software Heritage](https://www.softwareheritage.org/); the badge above tracks the archive status of the GitHub origin ([archived origin](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros), [archival visits](https://archive.softwareheritage.org/browse/origin/visits/?origin_url=https://github.com/0nyr/kayros)). For academic referencing, prefer Software Heritage identifiers (SWHIDs) of the exact archived revision or release tag over the moving repository origin; when reporting computational results, cite both the kayros release used and the MAMUT-routing benchmark artifacts it was run on.

The component study of the exact solver consists of 1,400 runs on the 56 Dabia2013 instances with 100 customers, 1,120 of them with KAYROS 1.7.0 and the others with the repaired upstream solver. The KAYROS run records identify the sources by commit `25d8f00`, whose `cpp/` and `src/` trees are those of the `v1.7.0` release. Version 1.7.1 changes no behaviour of the solver. Its changes to these trees are comments, documentation strings, names of tests and of build targets, and new tests, and the results of the exact component are identical to those of 1.7.0 on the instances compared (see the 1.7.1 entry of the changelog).

## Branches

Development happens on `main`. One long-lived branch is deliberately **not** merged: `td-time-warp` prototypes penalised exploration of the time-infeasible region (time-warp moves in the local search). A controlled head-to-head found it a no-go for the default solver (parity at best, worse on several families at equal time budgets), so it is kept reachable behind `Params` for reproducibility rather than merged; the negative result is written up in full as a thesis appendix.

## Provenance

KAYROS is developed by [Florian Rascoussier (Onyr)](https://github.com/0nyr) as part of a [PhD in Informatics and Operations Research](https://theses.fr/s386454) (IMT Atlantique / INSA Lyon), with scientific supervision by Romain Billot, Lina Fahed and Christine Solnon. The authors, in citation order, are Florian Rascoussier, Romain Billot, Lina Fahed and Christine Solnon. Their ORCID identifiers are recorded in [CITATION.cff](https://github.com/0nyr/kayros/blob/main/CITATION.cff). The NDCPWLF composition engine follows Visser & Spliet (2020)'s move-evaluation theorems; the local-search move evaluation follows Blauth et al. (2024); the exact component vendors the branch-price-and-cut solver of Lera-Romero, Miranda Bront & Soulignac (2020, MIT-licensed; provenance and local modifications documented in `cpp/lera/NOTICE.md`); the TD-ACO is a rewrite of Florian Rascoussier's heuristic layer originally built on that same solver. MIT license.

### References

- Gonzalo Lera-Romero, Juan J. Miranda Bront, Francisco J. Soulignac. *Linear edge costs and labeling algorithms: The case of the time-dependent vehicle routing problem with time windows.* **Networks** 76(1):24–53, 2020. [doi:10.1002/net.21937](https://doi.org/10.1002/net.21937). Companion code: [gleraromero/networks2020](https://github.com/gleraromero/networks2020) by [Gonzalo Lera-Romero](https://github.com/gleraromero), the solver vendored under `cpp/lera/`. As of 2026-07-18 its only GitHub star is [ours](https://github.com/0nyr): sadly little recognition for such an important project in the time-dependent routing world. Go give it a second one.
- Thomas R. Visser, Remy Spliet. *Efficient move evaluations for time-dependent vehicle routing problems.* **Transportation Science** 54(4):1091–1112, 2020. [doi:10.1287/trsc.2019.0938](https://doi.org/10.1287/trsc.2019.0938)
- Jannis Blauth, Stephan Held, Dirk Müller, Niklas Schlomberg, Vera Traub, Thorben Tröbst, Jens Vygen. *Vehicle routing with time-dependent travel times: Theory, practice, and benchmarks.* **Discrete Optimization** 53:100848, 2024. [doi:10.1016/j.disopt.2024.100848](https://doi.org/10.1016/j.disopt.2024.100848)

## Acknowledgements

Full authorship, supervision, and contributor policy: [`AUTHORS.md`](https://github.com/0nyr/kayros/blob/main/AUTHORS.md).

- **[Romain Billot](https://www.imt-atlantique.fr/fr/personne/romain-billot), [Christine Solnon](http://perso.citi.insa-lyon.fr/csolnon/) and [Lina Fahed](https://www.imt-atlantique.fr/fr/personne/lina-fahed)**, coauthors of KAYROS who supervise the PhD this solver is built for. A special mention to **Christine** for her guidance and for her decade-long involvement around Time-Dependent routing problems that goes as far back as the [PhD of Pénélope Aguiar Melgarejo](https://hal.science/hal-01514369) (more info on [Christine's page](http://perso.citi.insa-lyon.fr/csolnon/)).
- **[Adrien Pichon](https://github.com/Anzury), [Marc Sevaux](http://www-labsticc.univ-ubs.fr/~sevaux/) and [Alexandru-Liviu Olteanu](http://www-labsticc.univ-ubs.fr/~olteanu/)** as members of the [ANR-MAMUT project](https://anr.fr/Projet-ANR-22-CE22-0016) and its [MAMUT-routing benchmark catalog](https://github.com/ANR-MAMUT/MAMUT-routing).
- **[Romain Fontaine](https://github.com/romainfontaine)**, for his help with Grid'5000, where every KAYROS validation and certification campaign runs. This PhD is a multi-vehicle follow-up to [his TDTSPTW thesis](https://hal.science/tel-04697323v1) and his dynamic-programming solver, [tdtsptw-ejor23](https://github.com/romainfontaine/tdtsptw-ejor23).
- **[Gonzalo Lera-Romero](https://github.com/gleraromero)**, who made his branch-price-and-cut solver [open source](https://github.com/gleraromero/networks2020): a major milestone on the road to this thesis and the direct inspiration for KAYROS's exact component.
- **[Leon Lan](https://github.com/leonlan), [Niels Wouda](https://github.com/N-Wouda), [Wouter Kool](https://github.com/wouterkool)** and other contributors to [PyVRP](https://github.com/PyVRP/PyVRP) who made and kept open-source the PyVRP framework, which is another major inspiration for Kayros's anytime heuristic layer, in 3 aspects: the realization that a single-trajectory iterated local search (ILS) is a strong, simple and scalable anytime metaheuristic for vehicle routing (away from [Thibaut Vidal](https://github.com/vidalt)'s [HGS](https://github.com/vidalt/HGS-CVRP)), for the overall C++/Python architecture, and for the additional competitive drive of the day I read [this post by Niels](https://github.com/PyVRP/PyVRP/issues/867#issuecomment-3045950768). At that time, I was already on this research stream for a year and realized I was not alone on this problem which was very motivating.
- **[Thibaut Vidal](https://github.com/vidalt)**, who initiated the movement of open-source vehicle routing solvers and whose [HGS-CVRP](https://github.com/vidalt/HGS-CVRP) is a reference anytime metaheuristic for vehicle routing. I have learned a lot from his work, and I am grateful for the open-source spirit he has fostered in the community.

## Funding

This work is funded by the [French National Research Agency (ANR)](https://anr.fr) as part of the [MAMUT project](https://anr.fr/Projet-ANR-22-CE22-0016), ANR-22-CE22-0016 "Machine learning et matheuristiques pour le transport urbain" (Machine learning And Matheuristics algorithms for Urban Transportation).
