"""Random TD-instance generation for differential prover testing (M5.9).

The exact BPC prover is verified against the ILS heuristic on *random* small
instances, the instance-level analogue of the ``pwlf_compare`` random-curve
tests. Random instances are built by **subsampling real benchmark instances**:
a random subset of customers is kept and relabelled, and the per-arc ATFs are
reused verbatim. This preserves genuine value-jump (stepwise) structure exactly
-- the feature that breaks the prover -- without synthesising ATFs from scratch.

Two soundness invariants are checked per instance (see
``test_prover_fuzz_soundness``):

  (S1) a certified optimum must not exceed any checker-valid feasible cost
       (the ILS solution); a proof cannot be worse than a known solution.
  (S2) the certified value must not depend on the warm start (cold == warm).

Both are violated exactly when the labeling misprices value jumps.
"""
from __future__ import annotations

import random
from pathlib import Path

from mamut_routing_lib.td import InstanceATFs, LoadedTDInstance, load_td_instance

# Rifki2020 = genuine value-jump ATFs (exercises the bug); the others are the
# jump-free control that must stay sound.
STEPWISE_POOL = [("Rifki2020", "n=10"), ("Rifki2020", "n=20")]
JUMPFREE_POOL = [("Dabia2013", "n=25"), ("Solomon1987", "n=25")]


def instances_in(root: Path, family: str, size: str, problem: str = "TDVRPTW") -> list[Path]:
    d = Path(root) / problem / family / size
    return sorted(d.glob("*.vrp.json")) if d.is_dir() else []


def subsample(loaded: LoadedTDInstance, keep_customers: list[int], name: str) -> LoadedTDInstance:
    """Restrict `loaded` to `keep_customers` (1-based), relabelled depot=0, 1..k.

    Surviving arc ATFs are reused unchanged, so all value jumps are preserved.
    """
    inst = loaded.instance
    old_nodes = [0] + list(keep_customers)
    remap = {old: new for new, old in enumerate(old_nodes)}
    k = len(keep_customers)

    def pick(seq):
        return [seq[o] for o in old_nodes]

    new_inst = inst.model_copy(update={
        "instance_name": name,
        "num_customers": k,
        "coordinates": pick(inst.coordinates),
        "demands": pick(inst.demands),
        "service_times": pick(inst.service_times),
        "time_windows": pick(inst.time_windows),
    })
    new_arcs = {
        (remap[i], remap[j]): f
        for (i, j), f in loaded.atfs.arcs.items()
        if i in remap and j in remap
    }
    new_atfs = InstanceATFs(
        instance_name=name,
        benchmark_name=loaded.atfs.benchmark_name,
        horizon=loaded.atfs.horizon,
        num_customers=k,
        arcs=new_arcs,
        generator=dict(loaded.atfs.generator),
        format_version=loaded.atfs.format_version,
    )
    return LoadedTDInstance(
        instance=new_inst, atfs=new_atfs,
        instance_path=loaded.instance_path, atf_path=loaded.atf_path,
        categories_path=None,
    )


def random_case(rng: random.Random, root: Path, *, stepwise_prob: float = 0.7,
                full_prob: float = 0.3):
    """A random instance: subsampled (k<=8) or, with ``full_prob``, the FULL
    source instance. Returns (family, size, name, LoadedTDInstance).

    Full-instance cases exist because the 2026-07-10 campaign showed defects
    that only express with full jump mass (the k<=8 subsamples all passed
    while full n=10 instances misclosed): a fuzz round without full instances
    under-tests the labeling.
    """
    pool = STEPWISE_POOL if rng.random() < stepwise_prob else JUMPFREE_POOL
    family, size = rng.choice(pool)
    paths = instances_in(root, family, size)
    if not paths:
        return None
    src = rng.choice(paths)
    loaded = load_td_instance(src)
    n = loaded.instance.num_customers
    stem = src.name.removesuffix(".vrp.json")
    if rng.random() < full_prob:
        return family, size, f"{stem}~full", loaded
    k = rng.randint(4, min(8, n))
    keep = sorted(rng.sample(range(1, n + 1), k))
    name = f"{stem}~{'.'.join(map(str, keep))}"
    return family, size, name, subsample(loaded, keep, name)


def run_ils(loaded: LoadedTDInstance, seed: int, tl: float):
    """ILS feasible reference. Returns (checker-priced cost, routes)."""
    from kayros.solve import Params, solve

    sol = solve(loaded, Params(strategy="ils"), time_limit=tl, seed=seed)
    routes = [list(r) for r in sol.routes]
    return sol.duration, routes


def run_exact(loaded: LoadedTDInstance, tl: float, warm=None):
    from kayros.lera import solve_duration

    return solve_duration(loaded, time_limit_s=tl, initial_routes=warm)


def soundness(loaded: LoadedTDInstance, *, tl_ils: float = 5.0, tl_exact: float = 30.0,
              seed: int = 0) -> dict:
    """Run ILS + cold + warm and evaluate the two soundness invariants.

    Returns a dict with costs/status and booleans ``s1`` (cold <= ILS) and
    ``s2`` (cold == warm). ``sound`` is their conjunction.
    """
    ils_cost, ils_routes = run_ils(loaded, seed=seed, tl=tl_ils)
    cold = run_exact(loaded, tl_exact)
    warm = run_exact(loaded, tl_exact, warm=ils_routes)
    cs, cv = cold["exact_log"]["status"], cold["value"]
    ws, wv = warm["exact_log"]["status"], warm["value"]
    s1 = not (cs == "Optimum" and cv > ils_cost + 1e-6)
    s2 = not (cs == "Optimum" and ws == "Optimum" and abs(cv - wv) > 1e-6)
    return {
        "ils_cost": ils_cost, "ils_routes": ils_routes,
        "cold_status": cs, "cold_value": cv,
        "warm_status": ws, "warm_value": wv,
        "s1": s1, "s2": s2, "sound": s1 and s2,
    }
