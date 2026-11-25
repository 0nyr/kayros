"""Build the pinned native library for the interpreter that will build Kayros."""

import os
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from zipfile import ZipFile

root = Path(sys.argv[1]).resolve()
wheelhouse = root / "wheels"
wheelhouse.mkdir(exist_ok=True)
with TemporaryDirectory(dir=wheelhouse) as staging:
    # An earlier interpreter in the release matrix may have staged its wheel.
    env = os.environ.copy()
    env.pop("PIP_CONSTRAINT", None)
    env.pop("PIP_BUILD_CONSTRAINT", None)
    subprocess.run([
        sys.executable, "-m", "pip", "wheel", "--no-deps", "--wheel-dir", staging,
        str(root / "MAMUT-routing-lib"),
    ], env=env, check=True)
    wheel, = Path(staging).glob("mamut_routing_lib-*.whl")
    with ZipFile(wheel) as archive:
        if not any(name.startswith("mamut_routing_lib/_td_exact.") and name.endswith(".so")
                   for name in archive.namelist()):
            raise RuntimeError(f"Native MAMUT extension missing from {wheel.name}")
    staged = wheel.replace(wheelhouse / wheel.name)

# A newer index release must not supersede the gitlinked source in CI.
(wheelhouse / "constraints.txt").write_text(f"mamut-routing-lib @ {staged.as_uri()}\n")
