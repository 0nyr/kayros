# KAYROS

[![PyPI](https://img.shields.io/pypi/v/kayros)](https://pypi.org/project/kayros/) [![SWH](https://archive.softwareheritage.org/badge/origin/https://github.com/0nyr/kayros/)](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros)

**KAYROS** is an exact & anytime solver for **duration-minimization time-dependent vehicle routing** problems — TDVRPTW (with time windows) and TDVRP — benchmarked on the canonical [MAMUT-routing](https://github.com/ANR-MAMUT/MAMUT-routing) TD instance families.

The name is a nod to [*Kairos*](https://en.wikipedia.org/wiki/Kairos), the ancient Greek notion of the *right, opportune moment*, fitting for a time-dependent solver where *when* each route departs is itself a decision. It is also a [recursive acronym](https://en.wikipedia.org/wiki/Recursive_acronym): **K**ayros **A**nytime-**Y**ielding **R**outing **O**ptimization **S**olver.

> Status: **alpha**, under active development as part of a PhD. v0.3.0 ships both solving modes: the anytime heuristic stack (TD-ACO + time-dependent local search), which produced the large majority of the MAMUT store's best-known solutions, and the exact branch-price-and-cut component (`kayros.lera`), which certified 334 of them optimal.

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
```

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
- **The anytime stack** (`kayros.solve`): greedy construction, a MAX-MIN TD ant colony, and a time-dependent local-search layer using LCA-BST move evaluation (Blauth et al. 2024) — tree-ranked relocate/swap/2-opt\* where every *accepted* move is repriced by the checker-identical fold before it counts.
- **The exact component** (`kayros.lera`): the branch-price-and-cut solver of Lera-Romero, Rönnqvist & Ljungqvist (2020), vendored under `cpp/lera/` (see its `NOTICE.md`) on the open-source [HiGHS](https://highs.dev/) LP backend, extended with deadline-compliant anytime behavior, warm starts through columns, TDVRP support, and honest time-limit gap reporting. Every column entering the master problem is repriced in the checker's arithmetic, so reported values — and optimality certificates — are checker-exact: *optimal under checker-exact route costs and standard LP/pricing tolerances, completeness modulo the search engine's epsilon dominance*.

## Core principles

- **The checker is the referee.** Every solution and every certificate is priced by the reference checker of `mamut-routing-lib`; the checker's value is the value.
- **Exact arithmetic.** Plain IEEE-754 doubles, no epsilon comparisons in the engine, no FMA contraction (`-ffp-contract=off`); results are bit-reproducible across machines.
- **Anytime first.** Time budgets are hard deadlines honored by every component (heuristics and exact search alike), and incumbents stream out as they are found — a solver that only answers at the end is not a solver you can interrupt.
- **One-command install, no proprietary dependency.** The default build — including the exact component — is pure open source; HiGHS is built statically into the wheels. The faster CPLEX backend for the BPC remains strictly a source-build opt-in (`-DLERA_LP_BACKEND=cplex`) and never ships in wheels.
- **One run is one thread.** No intra-run parallelism; parallelism belongs to the experiment layer above.
- **POD core.** The fresh C++ is plain structs, flat arrays and free functions — optimization-kernel style, no framework (the vendored BPC keeps its upstream style, contained under `cpp/lera/`).

## Archival and reproducibility

`kayros` is archived by [Software Heritage](https://www.softwareheritage.org/); the badge above tracks the archive status of the GitHub origin ([archived origin](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros), [archival visits](https://archive.softwareheritage.org/browse/origin/visits/?origin_url=https://github.com/0nyr/kayros)). For academic referencing, prefer Software Heritage identifiers (SWHIDs) of the exact archived revision or release tag over the moving repository origin — when reporting computational results, cite both the kayros release used and the MAMUT-routing benchmark artifacts it was run on.

## Provenance

KAYROS is developed by [Florian Rascoussier (Onyr)](https://github.com/0nyr) as part of a PhD in operations research (IMT Atlantique / INSA Lyon), under the supervision of Romain Billot, Christine Solnon and Lina Fahed. The NDCPWLF composition engine follows Visser & Spliet (2020)'s move-evaluation theorems; the local-search move evaluation follows Blauth et al. (2024); the exact component vendors the branch-price-and-cut solver of Lera-Romero, Rönnqvist & Ljungqvist (2020, MIT-licensed — provenance and local modifications documented in `cpp/lera/NOTICE.md`); the TD-ACO is a rewrite of the author's heuristic layer originally built on that same solver. MIT license.
