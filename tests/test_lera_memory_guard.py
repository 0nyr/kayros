"""M13.2 memory self-guard gates: the prover always returns an honest verdict.

The guard is an RSS watermark polled at the same loop heads as the time-limit
deadline. Contract under test: (1) a crossed watermark yields a clean
``MemoryLimitReached`` status (never an OS OOM kill, never a stamp), (2) an
armed-but-untripped guard perturbs nothing: values are bit-identical to a
guard-off run, (3) the trip fires inside the labeling on the real pathology
class (full-horizon TDVRP label accumulation, the Vu2020 re-certification
INCOMPLETE class).
"""

import pytest
from mamut_routing_lib.td import load_td_instance

from conftest import family_instances


def pick(paths, names):
    return [p for p in paths if p.name.removesuffix(".vrp.json") in names]


C101_25 = pick(family_instances("TDVRPTW", "Dabia2013", ["n=25"]), {"C101"})
VU_TDVRP_59 = family_instances("TDVRP", "Vu2020", ["n=59"])[:1]


@pytest.mark.parametrize("instance_path", C101_25, ids=lambda p: "C101")
def test_memory_guard_trips_cleanly(instance_path) -> None:
    # A watermark below the process's standing RSS trips at the first poll:
    # the solve must unwind with an honest status and refuse a stamp.
    from kayros.lera import optimality_metadata, solve_duration

    loaded = load_td_instance(instance_path)
    result = solve_duration(loaded, time_limit_s=60.0, memory_limit_mb=50)
    assert result["exact_log"]["status"] == "MemoryLimitReached"
    assert result["memory"]["limit_reached"] is True
    assert result["memory"]["limit_mb"] == 50.0
    assert optimality_metadata(result) is None


@pytest.mark.parametrize("instance_path", C101_25, ids=lambda p: "C101")
def test_memory_guard_default_is_bit_identical(instance_path) -> None:
    # Default-on guard (auto limit) is pure observation: same status and a
    # bit-identical objective vs an explicit guard-off run.
    from kayros.lera import solve_duration

    loaded = load_td_instance(instance_path)
    off = solve_duration(loaded, time_limit_s=120.0, memory_limit_mb=0)
    default = solve_duration(loaded, time_limit_s=120.0)
    assert off["exact_log"]["status"] == "Optimum"
    assert default["exact_log"]["status"] == "Optimum"
    assert default["value"] == off["value"]
    assert default["memory"]["limit_mb"] > 0  # auto limit resolved on Linux
    assert default["memory"]["limit_reached"] is False
    assert off["memory"]["limit_mb"] == 0.0


def test_optimality_metadata_none_on_memory_limit() -> None:
    from kayros.lera import optimality_metadata

    assert optimality_metadata({"exact_log": {"status": "MemoryLimitReached"}}) is None


@pytest.mark.parametrize("instance_path", VU_TDVRP_59, ids=lambda p: p.name.removesuffix(".vrp.json"))
def test_memory_guard_trips_in_labeling(instance_path) -> None:
    # The real M13.2 pathology: full-horizon (trivial-TW) label accumulation.
    # A headroom watermark (current RSS + 512 MB) trips inside the labeling
    # within seconds on Vu2020 TDVRP n=59, the class whose unguarded runs
    # exceeded 96 GB and were OS-killed with no verdict in the 2026-07
    # re-certification campaign. The time limit is generous so a pass can
    # only come from the memory verdict.
    from kayros.lera import _read_own_rss_bytes, solve_duration

    loaded = load_td_instance(instance_path)
    rss_mb = _read_own_rss_bytes() / 1048576.0
    result = solve_duration(loaded, time_limit_s=600.0, memory_limit_mb=rss_mb + 512)
    assert result["exact_log"]["status"] == "MemoryLimitReached"
    assert result["memory"]["limit_reached"] is True
    # The stride-bounded overshoot stays well under the auto limit's headroom.
    assert result["memory"]["peak_rss_mb"] < rss_mb + 512 + 2048
