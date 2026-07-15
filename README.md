# KAYROS

[![PyPI](https://img.shields.io/pypi/v/kayros)](https://pypi.org/project/kayros/) [![SWH](https://archive.softwareheritage.org/badge/origin/https://github.com/0nyr/kayros/)](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros)

**KAYROS** is an exact & anytime solver for **duration-minimization time-dependent vehicle routing** problems — TDVRPTW (with time windows) and TDVRP — benchmarked on the canonical [MAMUT-routing](https://github.com/ANR-MAMUT/MAMUT-routing) TD instance families.

The name is a nod to [*Kairos*](https://en.wikipedia.org/wiki/Kairos), the ancient Greek notion of the *right, opportune moment*, fitting for a time-dependent solver where *when* each route departs is itself a decision. It is also a [recursive acronym](https://en.wikipedia.org/wiki/Recursive_acronym): **K**ayros **A**nytime-**Y**ielding **R**outing **O**ptimization **S**olver.

> Status: **alpha**, under active development as part of a PhD. v0.5.0 makes the exact branch-price-and-cut component's (`kayros.lera`) optimality certificates sound and audited. After two self-caught falsification episodes reshaped the protocol, a multi-gate re-certification (four independent solves per instance, cold and warm starts crossed with two labeling modes, an audited exact-pricing census, a pricing-integrity guard, and cross-platform agreement) certified **468** of the MAMUT store's best-known solutions proven optimal, 170 of them checker-valid strict improvements. The two anytime heuristics that produced the large majority of those best-known solutions ship unchanged: TD-ILS (single-trajectory iterated local search, the default) and TD-ACO, both over the same time-dependent granular local search.

## Install

```sh
pip install kayros
```

This pulls everything, including the benchmark loaders and the reference checker ([`mamut-routing-lib`](https://pypi.org/project/mamut-routing-lib/)). For development:

```sh
git clone https://github.com/0nyr/kayros && cd kayros
pip install -e . --group dev    # pip >= 25.1 (or: uv pip install -e . --group dev)
```

Requirements: Python ≥ 3.11. Building from source (sdist or checkout) additionally needs a C++23 compiler, CMake ≥ 3.26 (fetched automatically by the build backend when missing) and Boost.Graph headers+library; the HiGHS LP solver is fetched and built statically by CMake when no install is found.

## Usage

Anytime heuristic solve — construction + ant colony + local search, streaming every new incumbent:

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

# Strategy (0.4.0): "ils" (single-trajectory iterated local search — the
# default), "aco" (the historical default through 0.3.x), or "aco+ils"
# (budget split, experimental).
solution = kayros.solve(instance_path, kayros.Params(strategy="aco"), time_limit=60.0)
```

> **Behavior changes in 0.4.0**: (1) the default strategy is now `"ils"` — a 20,808-run head-to-head campaign (5 TD families, n=10..1000, equal time limits and seeds) had ILS beat ACO on 5714 of 6936 paired cells vs 305 losses, with the margin growing with instance size (−4.5% to −5.4% on n=200..1000); set `Params(strategy="aco")` to restore the 0.3.0 solver. Without a time limit the ILS run is bounded at five restart windows (`5 * restart_no_improvement` iterations). (2) The local search of *every* strategy now enumerates moves over granular candidate lists (`num_neighbours=50`, a time-dependent Vidal-style proximity) instead of exhaustive scans — up to ~13x faster descents at n=1000 for a sub-percent quality gap. Set `Params(num_neighbours=0)` to restore the 0.3.0 exhaustive enumeration.

Exact solve — branch-price-and-cut with checker-exact certificates, optionally warm-started from a known solution (the fast path when certifying near-optimal solutions, e.g. stored best-known ones):

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

`solution.duration` and `result["value"]` are always values computed by the reference checker (`mamut_routing_lib.td.check_td_solution`) — never an internal approximation.

## Design

KAYROS is two solving modes on one exact time-dependent engine:

- **The engine** (`cpp/pwlf`, `cpp/core`) represents arrival times as non-decreasing continuous piecewise-linear functions (NDCPWLF) and evaluates routes by exact function composition — a bit-identical C++ port of the reference checker's arithmetic (gated by an equivalence suite over the full benchmark set).
- **The anytime stack** (`kayros.solve`): greedy construction and two search strategies — a MAX-MIN TD ant colony and a single-trajectory TD iterated local search (granular ruin-and-recreate kicks, late-acceptance hill climbing, restart-to-best) — over one time-dependent local-search layer using LCA-BST move evaluation (Blauth et al. 2024) with granular candidate lists: tree-ranked relocate/swap/2-opt\* where every *accepted* move is repriced by the checker-identical fold before it counts.
- **The exact component** (`kayros.lera`): the branch-price-and-cut solver of Lera-Romero, Rönnqvist & Ljungqvist (2020), vendored under `cpp/lera/` (see its `NOTICE.md`) on the open-source [HiGHS](https://highs.dev/) LP backend, extended with deadline-compliant anytime behavior, warm starts through columns, TDVRP support, and honest time-limit gap reporting. Every column entering the master problem is repriced in the checker's arithmetic, so reported values — and optimality certificates — are checker-exact: *optimal under checker-exact route costs and standard LP/pricing tolerances, completeness modulo the search engine's epsilon dominance*. On stepwise (value-jump) travel-time functions that epsilon dominance can misprice by a full step height, so single-run optimality stamping is refused there by construction: such families are certified only under the audited multi-run campaign protocol (four solves per instance, cold and warm starts crossed with the two labeling modes, all agreeing on one checker-exact value with an audited exact-pricing phase and no checker-infeasible priced column), never by a single solve. Turning the LP dual bounds themselves into rigorous certificates (safe bounding) is future work.

## Core principles

- **The checker is the referee.** Every solution and every certificate is priced by the reference checker of `mamut-routing-lib`; the checker's value is the value.
- **Exact arithmetic.** Plain IEEE-754 doubles, no epsilon comparisons in the engine, no FMA contraction (`-ffp-contract=off`); results are bit-reproducible across machines.
- **Anytime first.** Time budgets are hard deadlines honored by every component (heuristics and exact search alike), and incumbents stream out as they are found — a solver that only answers at the end is not a solver you can interrupt.
- **One-command install, no proprietary dependency.** The default build — including the exact component — is pure open source; HiGHS is built statically into the wheels. The faster CPLEX backend for the BPC remains strictly a source-build opt-in (`-DLERA_LP_BACKEND=cplex`) and never ships in wheels.
- **One run is one thread.** No intra-run parallelism; parallelism belongs to the experiment layer above.
- **POD core.** The fresh C++ is plain structs, flat arrays and free functions — optimization-kernel style, no framework (the vendored BPC keeps its upstream style, contained under `cpp/lera/`).

## Archival and reproducibility

`kayros` is archived by [Software Heritage](https://www.softwareheritage.org/); the badge above tracks the archive status of the GitHub origin ([archived origin](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros), [archival visits](https://archive.softwareheritage.org/browse/origin/visits/?origin_url=https://github.com/0nyr/kayros)). For academic referencing, prefer Software Heritage identifiers (SWHIDs) of the exact archived revision or release tag over the moving repository origin — when reporting computational results, cite both the kayros release used and the MAMUT-routing benchmark artifacts it was run on.

## Branches

Development happens on `main`. One long-lived branch is deliberately **not** merged: `td-time-warp` prototypes penalised exploration of the time-infeasible region (time-warp moves in the local search). A controlled head-to-head found it a no-go for the default solver (parity at best, worse on several families at equal time budgets), so it is kept reachable behind `Params` for reproducibility rather than merged; the negative result is written up in full as a thesis appendix.

## Provenance

KAYROS is developed by [Florian Rascoussier (Onyr)](https://github.com/0nyr) as part of a PhD in operations research (IMT Atlantique / INSA Lyon), under the supervision of Romain Billot, Christine Solnon and Lina Fahed. The NDCPWLF composition engine follows Visser & Spliet (2020)'s move-evaluation theorems; the local-search move evaluation follows Blauth et al. (2024); the exact component vendors the branch-price-and-cut solver of Lera-Romero, Rönnqvist & Ljungqvist (2020, MIT-licensed — provenance and local modifications documented in `cpp/lera/NOTICE.md`); the TD-ACO is a rewrite of the author's heuristic layer originally built on that same solver. MIT license.
