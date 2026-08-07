"""Shared benchmark-resolution helpers for the test suite.

Suite-wide convention: every ``kayros.io.load_instance`` call in these tests
passes ``verify=True`` explicitly. The library default is ``False`` because it
is the solver's hot path, but the test suite is exactly where the check belongs:
it keeps the verification code path exercised on every run, and it turns a
corrupted or mismatched benchmark artifact into a test failure instead of a
silently wrong gate.
"""

import os
from pathlib import Path

import pytest


def benchmarks_root() -> Path | None:
    """MAMUT-routing benchmarks tree, from the standard lib env vars."""
    for var in ("MAMUT_ROUTING_BENCHMARKS_ROOT", "MAMUT_ROUTING_ROOT"):
        value = os.environ.get(var)
        if value:
            root = Path(value)
            if var == "MAMUT_ROUTING_ROOT":
                root = root / "benchmarks"
            if root.is_dir():
                return root
    return None


def family_instances(problem_type: str, family: str, size_dirs: list[str]) -> list:
    """Instance paths for pytest parametrization; empty when benchmarks are absent."""
    root = benchmarks_root()
    if root is None:
        return []
    paths: list[Path] = []
    for size_dir in size_dirs:
        directory = root / problem_type / family / size_dir
        if directory.is_dir():
            paths.extend(sorted(directory.glob("*.vrp.json")))
    return paths


def blauth2024_instances(size_dir: str) -> list:
    """Converted Blauth2024 instances (M7 FleetCostDuration gates).

    The family is CC-BY-NC and not vendored with kayros: it resolves through
    KAYROS_BLAUTH2024_DIR (a directory of ``Blauth-<city>.vrp.json`` + ATF
    sidecars, e.g. a size dir of the converted preview tree) or, once the
    satellite repo is mounted, through the standard benchmarks tree. Empty
    when neither is reachable (tests skip).
    """
    env = os.environ.get("KAYROS_BLAUTH2024_DIR")
    if env:
        directory = Path(env)
        if directory.is_dir():
            return sorted(directory.glob("*.vrp.json"))
    return family_instances("TDVRPTW", "Blauth2024", [size_dir])


def require_benchmarks() -> None:
    if benchmarks_root() is None:
        pytest.skip(
            "MAMUT-routing benchmarks not found: set MAMUT_ROUTING_BENCHMARKS_ROOT "
            "(or MAMUT_ROUTING_ROOT) to a MAMUT-routing checkout on the td branch"
        )
