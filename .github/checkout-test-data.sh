#!/usr/bin/env bash
# Resolve the library and family gitlinks from one edition of MAMUT-routing.
set -euo pipefail

root=${1:?usage: checkout-test-data.sh DESTINATION [full|smoke|library]}
mode=${2:-full}
umbrella=6c56d1b043801094c4f02ded4083a6c7d3dc956a
org=https://github.com/ANR-MAMUT

fetch_tree() {
    local directory=$1 url=$2 revision=$3
    git init -q "$directory"
    git -C "$directory" remote add origin "$url"
    git -C "$directory" fetch -q --depth=1 --filter=blob:none origin "$revision"
}

fetch_tree "$root" "$org/MAMUT-routing.git" "$umbrella"

checkout_family() {
    local family=$1
    shift
    local directory="$root/benchmarks/$family"
    local revision
    revision=$(git -C "$root" rev-parse "$umbrella:benchmarks/$family")
    fetch_tree "$directory" "$org/MAMUT-routing-$family.git" "$revision"
    git -C "$directory" sparse-checkout set --cone "$@"
    git -C "$directory" checkout -q --detach "$revision"
}

lib_revision=$(git -C "$root" rev-parse "$umbrella:MAMUT-routing-lib")
fetch_tree "$root/MAMUT-routing-lib" "$org/MAMUT-routing-lib.git" "$lib_revision"
git -C "$root/MAMUT-routing-lib" checkout -q --detach "$lib_revision"
mkdir -p "$root/wheels"
touch "$root/wheels/constraints.txt"

if [[ $mode == library ]]; then
    exit 0
elif [[ $mode == smoke ]]; then
    checkout_family Dabia2013 TDVRPTW/n=25 sidecars/n=25
elif [[ $mode == full ]]; then
    checkout_family Dabia2013 TDVRP TDVRPTW sidecars
    checkout_family Ari2018 {TDVRP,TDVRPTW,sidecars}/{n=15,n=40}
    checkout_family Vu2020 {TDVRP,TDVRPTW,sidecars}/{n=59,n=99}
    checkout_family Rifki2020 {TDVRP,TDVRPTW,sidecars}/{n=10,n=20,n=30,n=40,n=50}
    checkout_family Blauth2024 TDVRPTW/n=10 sidecars/n=10
else
    echo "Unknown test data selection: $mode" >&2
    exit 2
fi
