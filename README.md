# KAYROS

[![PyPI](https://img.shields.io/pypi/v/kayros)](https://pypi.org/project/kayros/) [![SWH](https://archive.softwareheritage.org/badge/origin/https://github.com/0nyr/kayros/)](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros)

**KAYROS** solves time-dependent vehicle routing problems with or without customer time windows (TDVRPTW and TDVRP). The heuristic minimizes total route duration or fleet cost plus duration. The branch-price-and-cut solver minimizes duration. Both read the time-dependent families of [MAMUT-routing](https://github.com/ANR-MAMUT/MAMUT-routing).

The name is a nod to [*Kairos*](https://en.wikipedia.org/wiki/Kairos), the ancient Greek notion of the *right, opportune moment*, fitting for a time-dependent solver where *when* each route departs is itself a decision. It is also a [recursive acronym](https://en.wikipedia.org/wiki/Recursive_acronym): **K**ayros **A**nytime-**Y**ielding **R**outing **O**ptimization **S**olver.

**Version 2.0 is in development.** Its heuristic checks solutions under the MAMUT profile `td-binary64-exact-v1`. The branch-price-and-cut solver's internal stopping condition and floating-point bound are diagnostics while proof verification for this profile is in development.

## Install

The published 1.7.1 release is available with `pip install kayros==1.7.1`. To reproduce published runs with that release, use `mamut-routing-lib==0.11.0` and the [archived benchmark layout](https://github.com/ANR-MAMUT/MAMUT-routing-archive-2026-11).

For the 2.0 development version, install library 0.13 and Kayros in the same Python environment with pip 25.1 or later.

```sh
python -m pip install "mamut-routing-lib @ git+https://github.com/ANR-MAMUT/MAMUT-routing-lib.git@0ead6a19f3688b60edc4fd66da073a07813c3216"
git clone https://github.com/0nyr/kayros && cd kayros
python -m pip install --group dev -e .
```

The development build supports CPython 3.11 to 3.14. Building both projects from source requires a C++23 compiler, Boost.Graph and GMP development files. The Kayros build backend provides CMake 3.26 or later. CMake builds HiGHS statically when it cannot find an installed copy. Link-time optimization for the branch-price-and-cut solver is enabled on supported toolchains (`KAYROS_LERA_IPO`).

The examples use paths relative to a MAMUT-routing checkout with the corresponding family submodule and sidecars installed. [Benchmark collections](https://github.com/ANR-MAMUT/MAMUT-routing)

## Usage

Use `kayros.solve` for an anytime iterated local search. An optional callback receives each incumbent while the search runs.

```python
import kayros

instance_path = "benchmarks/Dabia2013/TDVRPTW/n=25/C101.vrp.json"
solution = kayros.solve(instance_path, time_limit=10.0, seed=42)
print(solution.cost, solution.num_routes, solution.status)

# Anytime: react to every new incumbent while the solve keeps running
def on_incumbent(incumbent, routes):
    print(f"[{incumbent.seconds:7.2f}s] {incumbent.value:.6f} ({incumbent.origin})")

solution = kayros.solve(instance_path, time_limit=60.0, on_incumbent=on_incumbent)
```

Pass `verify=True` on the first load of unfamiliar data to check the sidecar and materialized arrival-function digests against the instance metadata. Repeated runs can reuse the loaded instance. Digest verification is optional because it can take minutes and several gigabytes on large instances.

To polish a known solution, pass `initial_routes=previous.routes` to `kayros.solve`. Each route contains customer IDs and omits the depot. Kayros validates the seed under the selected objective, then starts the local search from the supplied route order. For Blauth2024 record polishing, set `objective="FleetCostDuration"` and `k_cap=len(previous.routes)` in `Params`.

The ILS uses granular candidate lists. Set `Params(num_neighbours=0)` to enumerate the whole neighborhood.

Blauth2024 instances carry a fixed cost per vehicle. With `FleetCostDuration`, the checked objective is the sum of route durations plus the fixed cost times the number of routes. The search can dissolve routes and polish their durations under this objective.

```python
berlin = "benchmarks/Blauth2024/TDVRPTW/n=10/Blauth-berlin.vrp.json"
params = kayros.Params(objective="FleetCostDuration")
fleet_solution = kayros.solve(berlin, params, time_limit=10.0, seed=42)
print(fleet_solution.cost, fleet_solution.num_routes)
```

`solution.cost` is the binary64 display of the exact cost computed by the library (profile `td-binary64-exact-v1`), and `solution.exact_cost` is that cost as a rational. The value of the search fold is kept in `solution.search_value`.

With the default `Params(acceptance_mode="each_incumbent")`, Kayros checks each candidate new best for feasibility and strict improvement of the rational objective before installing it as the incumbent. The callback exposes the rational `incumbent.exact_cost`, its binary64 display `incumbent.value` and the approximate `incumbent.search_value`. Two strictly improving rational costs can have the same display.

With `Params(acceptance_mode="final_only", incumbent_history_size=6)`, the callback reports provisional search improvements with `incumbent.exact_cost=None`. If the final candidate fails the exact check, Kayros tries the last 6 distinct prior route sets in increasing order of approximate cost. Set the history size to 0 to check only the final candidate. The solver logs final rejections and recovery and raises `InfeasibleError` if all retained candidates fail. `solution.acceptance_stats` gives counts of infeasible proposals, nonimproving proposals, squeeze rejections, final rejections and fallback attempts.

## Branch-price-and-cut

The Duration solver uses the implementation of Lera-Romero, Miranda Bront and Soulignac (2020). It can start from customer-only incumbent routes. A time limit and a configurable resident-memory limit allow it to return when pricing exhausts its budget. Its search log records pricing and node closure diagnostics. Under the 2.0 evaluation profile, verifying an incumbent establishes its objective value. An optimality claim also requires a verified lower bound.

The same Dabia incumbent from the heuristic example can seed the exact solver. A checked incumbent has `result.status == "feasible"`. Its routes and rational objective are in `result.routes` and `result.exact_cost`. The separate `result.termination_reason` distinguishes a closed search from a time, node or memory limit.

```python
from kayros.exact import solve_exact

result = solve_exact(instance_path, time_limit=600.0, initial_routes=solution.routes)
print(result.status, result.termination_reason, result.exact_cost)
print(result.reported_best_bound, result.certified_dual_bound)
```

## Implementation

The C++ local search represents arc arrival times as piecewise-linear functions and evaluates moves in binary64 arithmetic. On instances with time windows, it caches the composed arc functions used by the move evaluator. It uses greedy construction, ruin-and-recreate perturbations, late acceptance and restarts from the incumbent. The MAMUT library prices the reported route set and computes exact minimizing departure times.

The branch-price-and-cut solver lives in `cpp/lera/`. Its default LP backend is HiGHS. Source builds can select CPLEX with `-DLERA_LP_BACKEND=cplex`. Each solver run uses one thread. [Solver provenance and modifications](https://github.com/0nyr/kayros/blob/main/cpp/lera/NOTICE.md)

## Migrating from 1.7.1

Instances now live under `benchmarks/<family>/<problem_type>/`. Read `solution.cost` for the objective display and `solution.exact_cost` for the rational value. The ILS is the supported heuristic. Fleet descent uses `fd_period_work` to count evaluated moves between periodic triggers. [Changelog and migration table](https://github.com/0nyr/kayros/blob/main/CHANGELOG.md)

## Archival and reproducibility

`kayros` is archived by [Software Heritage](https://www.softwareheritage.org/); the badge above tracks the archive status of the GitHub origin ([archived origin](https://archive.softwareheritage.org/browse/origin/?origin_url=https://github.com/0nyr/kayros), [archival visits](https://archive.softwareheritage.org/browse/origin/visits/?origin_url=https://github.com/0nyr/kayros)). For academic referencing, prefer Software Heritage identifiers (SWHIDs) of the exact archived revision or release tag over the moving repository origin; when reporting computational results, cite both the kayros release used and the MAMUT-routing benchmark artifacts it was run on.

The component study of the exact solver consists of 1,400 runs on the 56 Dabia2013 instances with 100 customers, 1,120 of them with KAYROS 1.7.0 and the others with the [repaired upstream solver](https://github.com/0nyr/lera-tdvrptw-bpc-repaired) (tag `component-study-as-run`). Its run records and analysis are published as a [dataset](https://doi.org/10.5281/zenodo.24153428), and its harness is the repository [kayros-ablation](https://github.com/0nyr/kayros-ablation). The KAYROS run records identify the sources by commit `25d8f00`, whose `cpp/` and `src/` trees are those of the `v1.7.0` release. Version 1.7.1 changes no behaviour of the solver. Its changes to these trees are comments, documentation strings, names of tests and of build targets, and new tests, and the results of the exact component are identical to those of 1.7.0 on the instances compared (see the 1.7.1 entry of the changelog).

## Provenance

KAYROS is developed by [Florian Rascoussier (Onyr)](https://github.com/0nyr) as part of a [PhD in Informatics and Operations Research](https://theses.fr/s386454) (IMT Atlantique / INSA Lyon), with scientific supervision by Romain Billot, Lina Fahed and Christine Solnon. The authors, in citation order, are Florian Rascoussier, Romain Billot, Lina Fahed and Christine Solnon. Their ORCID identifiers are recorded in [CITATION.cff](https://github.com/0nyr/kayros/blob/main/CITATION.cff). The NDCPWLF composition engine follows Visser & Spliet (2020)'s move-evaluation theorems; the local-search move evaluation follows Blauth et al. (2024); the exact component vendors the branch-price-and-cut solver of Lera-Romero, Miranda Bront & Soulignac (2020, MIT-licensed; provenance and local modifications documented in `cpp/lera/NOTICE.md`). MIT license.

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
