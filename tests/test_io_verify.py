"""The artifact verification switch: it is off by default, and it works.

``kayros.io.load_instance`` defaults to ``verify=False`` because the hot path
re-derives a digest that costs minutes at n = 1000 for data that materializes
deterministically. That default is only defensible if the opt-in check is real,
so these gates tamper with a sidecar and require the verified load to refuse it
while the unverified load, by design, does not notice.
"""

import json
import shutil

import pytest
from mamut_routing_lib.td import ATFFormatError

import kayros
from kayros.io import load_instance

from conftest import family_instances

DABIA_25 = family_instances("TDVRPTW", "Dabia2013", ["n=25"])


@pytest.fixture
def tampered_instance(tmp_path):
    """A copy of a real instance whose sidecar carries one altered arc.

    Every arrival value of the first arc is shifted by one second, which keeps
    the function non-decreasing, keeps arrivals after departures and keeps the
    horizon spanned, so nothing but the declared ``atf_sha256`` can catch it.
    """
    if not DABIA_25:
        pytest.skip("MAMUT-routing benchmarks not found")
    source = DABIA_25[0]
    instance_path = tmp_path / source.name
    shutil.copyfile(source, instance_path)
    sidecar_name = json.loads(source.read_text())["td"]["atf_path"]
    sidecar_path = tmp_path / sidecar_name
    sidecar = json.loads((source.parent / sidecar_name).read_text())
    i, j, xs, ys = sidecar["arcs"][0]
    sidecar["arcs"][0] = [i, j, xs, [y + 1.0 for y in ys]]
    sidecar_path.write_text(json.dumps(sidecar))
    return instance_path


def test_verified_load_refuses_a_tampered_sidecar(tampered_instance) -> None:
    with pytest.raises(ATFFormatError, match="sha256"):
        load_instance(tampered_instance, verify=True)


def test_unverified_load_accepts_a_tampered_sidecar(tampered_instance) -> None:
    """The cost of the default: an altered sidecar loads without a word.

    This is the behaviour the docstrings warn about, pinned here so it stays a
    documented trade-off rather than a surprise.
    """
    loaded = load_instance(tampered_instance)
    assert loaded.atfs.arcs, "the tampered sidecar loads with verification off"


def test_solve_forwards_verify_to_the_loader(tampered_instance) -> None:
    with pytest.raises(ATFFormatError, match="sha256"):
        kayros.solve(tampered_instance, verify=True)


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_verified_load_accepts_an_intact_instance(instance_path) -> None:
    """Both settings must agree on untampered data, arc for arc."""
    verified = load_instance(instance_path, verify=True)
    plain = load_instance(instance_path)
    assert verified.atfs.arcs.keys() == plain.atfs.arcs.keys()


@pytest.mark.parametrize(
    "instance_path", DABIA_25[:1], ids=lambda p: p.name.removesuffix(".vrp.json")
)
def test_solve_rejects_verify_on_an_already_loaded_instance(instance_path) -> None:
    """Verification happens at load time; asking for it afterwards is a
    misuse, and silently ignoring it would be a false sense of checking."""
    loaded = load_instance(instance_path, verify=True)
    with pytest.raises(ValueError, match="already-loaded"):
        kayros.solve(loaded, verify=True)
