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

Behavioral changes inherited from the half-port (upstream `TDVRPTW-solver`, not introduced here): subset-row cut violation threshold raised from `> 0` to `> 0.1` in `BCP::SeparateCuts`.

## Conventions

This subtree is **vendored code**: kayros' plain-old-data / style conventions do not apply here. Keep diffs against upstream minimal and documented in this NOTICE.
