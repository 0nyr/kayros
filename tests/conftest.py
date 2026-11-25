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
    if os.environ.get("KAYROS_REQUIRE_BENCHMARKS") == "1":
        raise pytest.UsageError("CI requires a MAMUT-routing benchmarks tree")
    return None


def instance_path(problem_type: str, family: str, size_dir: str, name: str) -> Path:
    """Path of one instance in the family-first layout of library 0.13: benchmarks/<family>/<problem_type>/<size_dir>/<name>.vrp.json. Returns the path even when the file is absent (callers skip on `.exists()`), and skips the test when no benchmarks tree is configured."""
    root = benchmarks_root()
    if root is None:
        pytest.skip("MAMUT-routing benchmarks not found")
    return root / family / problem_type / size_dir / f"{name}.vrp.json"


def family_instances(problem_type: str, family: str, size_dirs: list[str]) -> list:
    """Instance paths for pytest parametrization; empty when benchmarks are absent."""
    root = benchmarks_root()
    if root is None:
        return []
    paths: list[Path] = []
    for size_dir in size_dirs:
        # Family-first layout of the MAMUT-routing collections (library 0.13).
        directory = root / family / problem_type / size_dir
        instances = sorted(directory.glob("*.vrp.json"))
        if not instances and os.environ.get("KAYROS_REQUIRE_BENCHMARKS") == "1":
            raise pytest.UsageError(f"CI benchmark directory has no instances: {directory}")
        paths.extend(instances)
    return paths


def blauth2024_instances(size_dir: str) -> list:
    """Locate converted Blauth2024 instances under KAYROS_BLAUTH2024_DIR or the installed family collection. The CC-BY-NC data are distributed separately from Kayros."""
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
            "(or MAMUT_ROUTING_ROOT) to a MAMUT-routing checkout with its family collections"
        )


@pytest.hookimpl(wrapper=True)
def pytest_runtest_makereport(item, call):
    report = yield
    if (
        os.environ.get("KAYROS_REQUIRE_BENCHMARKS") == "1"
        and report.skipped
        and item.get_closest_marker("skipif") is None
    ):
        report.outcome = "failed"
        report.longrepr = f"Unexpected skip in the required CI corpus: {report.longrepr}"
    return report


def pytest_terminal_summary(terminalreporter) -> None:
    """Print the largest fold-versus-exact distances seen during the session."""
    from exact_reference import DISTANCES, distance_summary_lines

    if DISTANCES:
        terminalreporter.section("fold distance versus exact evaluation")
        for line in distance_summary_lines():
            terminalreporter.write_line(line)
