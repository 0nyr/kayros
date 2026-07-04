# KAYROS

**KAYROS** is an exact & anytime solver for **duration-minimization time-dependent vehicle routing** problems — TDVRPTW (with time windows) and TDVRP — benchmarked on the canonical [MAMUT-routing](https://github.com/ANR-MAMUT/MAMUT-routing) TD instance families.

> Status: **pre-alpha** (stage 1 under construction). The first usable release (v0.1.0) ships a time-dependent Ant Colony Optimization heuristic on an exact non-decreasing continuous piecewise-linear (NDCPWLF) arrival-time engine. See the roadmap below.

## Design principles

- **One command install, no proprietary dependency.** The default build is pure open source (C++23 + pybind11). The optional exact branch-price-and-cut component of Lera-Romero et al. (requires CPLEX) is strictly opt-in (`-DKAYROS_WITH_LERA=ON`, source build only) and never ships in wheels.
- **Exact arithmetic, checker-refereed.** Route durations are computed on NDCPWLF arrival-time functions with exact doubles — no epsilon comparisons. Every solution kayros reports is priced by the reference checker of [`mamut-routing-lib`](https://github.com/ANR-MAMUT/MAMUT-routing); the checker's value is the value.
- **POD core.** The C++ core is plain structs, flat arrays and free functions — optimization-kernel style, no framework.

## Install

One command, directly from GitHub, inside your Python environment:

```sh
pip install git+https://github.com/0nyr/kayros
```

This pulls everything, including the benchmark loaders and the reference checker (`mamut-routing-lib`, pinned to the MAMUT-routing `td` branch until it is released on PyPI). For development:

```sh
git clone https://github.com/0nyr/kayros && cd kayros
pip install -e . --group dev    # pip >= 25.1 (or: uv pip install -e . --group dev)
```

Requirements: Python ≥ 3.11, a C++23 compiler and CMake ≥ 3.26 (fetched automatically by the build backend when missing).

## Roadmap (stage 1)

- [x] M3.0 — package scaffold, CI, PyPI wiring
- [x] M3.1 — NDCPWLF composition engine + POD instance/route core
- [x] M3.2 — exact equivalence gate against the reference checker (Dabia2013): 513 tests, 336 instances, zero divergences
- [x] M3.3 — `kayros.solve()`: greedy construction + TD-ACO
- [x] M3.4 — all four MAMUT TD families × {TDVRPTW, TDVRP}
- [ ] M3.5 — large-scale runs, best-known-solution seeding, ACO tuning (in progress)
- [ ] M3.6 — anytime API (`on_incumbent`, time budgets) + **v0.1.0 on PyPI**
- Later: time-dependent local search layer; optional exact BPC (`kayros[lera]`)

## Provenance

KAYROS is developed by [Florian Rascoussier (Onyr)](https://github.com/0nyr) as part of a PhD in operations research (IMT Atlantique / INSA Lyon), under the supervision of Romain Billot, Christine Solnon and Lina Fahed. The NDCPWLF composition engine follows Visser & Spliet (2020)'s move-evaluation theorems; the TD-ACO is a rewrite of the author's heuristic layer originally built on the TDVRPTW solver of Lera-Romero, Rönnqvist & Ljungqvist (2020, MIT-licensed). MIT license.
