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


def require_benchmarks() -> None:
    if benchmarks_root() is None:
        pytest.skip(
            "MAMUT-routing benchmarks not found: set MAMUT_ROUTING_BENCHMARKS_ROOT "
            "(or MAMUT_ROUTING_ROOT) to a MAMUT-routing checkout on the td branch"
        )
