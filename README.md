# KAYROS

[![SWH](https://archive.softwareheritage.org/badge/origin/https://github.com/0nyr/kayros/)](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros)

**KAYROS** is an exact & anytime solver for **duration-minimization time-dependent vehicle routing** problems — TDVRPTW (with time windows) and TDVRP — benchmarked on the canonical [MAMUT-routing](https://github.com/ANR-MAMUT/MAMUT-routing) TD instance families.

> Status: **alpha**. v0.2.0 ships an anytime time-dependent Ant Colony Optimization heuristic with a time-dependent local-search layer (LCA-BST move evaluation, Blauth et al. 2024) on an exact non-decreasing continuous piecewise-linear (NDCPWLF) arrival-time engine, benchmarked on all four MAMUT TD families — it produced the large majority of the store's current best-known solutions. See the roadmap below.

## Design principles

- **One command install, no proprietary dependency.** The default build is pure open source (C++23 + pybind11). The optional exact branch-price-and-cut component of Lera-Romero et al. (requires CPLEX) is strictly opt-in (`-DKAYROS_WITH_LERA=ON`, source build only) and never ships in wheels.
- **Exact arithmetic, checker-refereed.** Route durations are computed on NDCPWLF arrival-time functions with exact doubles — no epsilon comparisons. Every solution kayros reports is priced by the reference checker of [`mamut-routing-lib`](https://github.com/ANR-MAMUT/MAMUT-routing); the checker's value is the value.
- **POD core.** The C++ core is plain structs, flat arrays and free functions — optimization-kernel style, no framework.

## Install

```sh
pip install kayros
```

This pulls everything, including the benchmark loaders and the reference checker ([`mamut-routing-lib`](https://pypi.org/project/mamut-routing-lib/)). For development:

```sh
git clone https://github.com/0nyr/kayros && cd kayros
pip install -e . --group dev    # pip >= 25.1 (or: uv pip install -e . --group dev)
```

Requirements: Python ≥ 3.11; building from source (sdist or checkout) additionally needs a C++23 compiler and CMake ≥ 3.26 (fetched automatically by the build backend when missing).

## Usage

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

# Feed the MAMUT BKS pipeline: solution.to_benchmark_solution() returns the
# artifact accepted by mamut_routing_lib.td.bks.save_td_solution_as_bks_if_improved.
```

`solution.duration` is always the value computed by the reference checker (`mamut_routing_lib.td.check_td_solution`) — never an internal approximation.

## Roadmap (stage 1)

- [x] M3.0 — package scaffold, CI, PyPI wiring
- [x] M3.1 — NDCPWLF composition engine + POD instance/route core
- [x] M3.2 — exact equivalence gate against the reference checker (Dabia2013): 513 tests, 336 instances, zero divergences
- [x] M3.3 — `kayros.solve()`: greedy construction + TD-ACO
- [x] M3.4 — all four MAMUT TD families × {TDVRPTW, TDVRP}
- [x] M3.5 — large-scale runs on Grid'5000: seeded the initial best-known solutions for all 1352 MAMUT TD instances
- [x] M3.6 — anytime API (`on_incumbent`, time budgets) + **v0.1.0 on PyPI**
- [x] M3.7 — time-dependent local search layer (LCA-BST move evaluation, Blauth et al. 2024; **v0.2.0**): tree-ranked relocate/swap/2-opt\* moves, every accepted move repriced by the checker-identical fold; on by default (`Params.local_search`)
- Later: ACO re-tuning under local search; optional exact BPC (`kayros[lera]`)

## Archival and reproducibility

`kayros` is archived by [Software Heritage](https://www.softwareheritage.org/); the badge above tracks the archive status of the GitHub origin ([archived origin](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros), [archival visits](https://archive.softwareheritage.org/browse/origin/visits/?origin_url=https://github.com/0nyr/kayros)). For academic referencing, prefer Software Heritage identifiers (SWHIDs) of the exact archived revision or release tag over the moving repository origin — when reporting computational results, cite both the kayros release used and the MAMUT-routing benchmark artifacts it was run on.

## Provenance

KAYROS is developed by [Florian Rascoussier (Onyr)](https://github.com/0nyr) as part of a PhD in operations research (IMT Atlantique / INSA Lyon), under the supervision of Romain Billot, Christine Solnon and Lina Fahed. The NDCPWLF composition engine follows Visser & Spliet (2020)'s move-evaluation theorems; the TD-ACO is a rewrite of the author's heuristic layer originally built on the TDVRPTW solver of Lera-Romero, Rönnqvist & Ljungqvist (2020, MIT-licensed). MIT license.
