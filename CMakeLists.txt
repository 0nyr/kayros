cmake_minimum_required(VERSION 3.26)
project(kayros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Local-benchmarking builds only: never enabled for shipped wheels/sdists.
option(KAYROS_NATIVE "Build with -march=native (local benchmarking only)" OFF)
# Lera-Romero exact BPC component (M5.8: part of the default install). The
# default LP backend is HiGHS (open source, fetched and built statically when
# no install is found); CPLEX remains a source-build opt-in via
# -DLERA_LP_BACKEND=cplex. The default build stays free of any proprietary
# dependency.
option(KAYROS_WITH_LERA "Build the exact BPC component (HiGHS by default)" ON)

find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_core
    cpp/bindings/module.cpp
    cpp/pwlf/pwlf.cpp
    cpp/core/route_eval.cpp
    cpp/heuristics/greedy_makespan.cpp
    cpp/heuristics/aco.cpp
    cpp/ls/lca_tree.cpp
    cpp/ls/leaves.cpp
    cpp/ls/splice.cpp
    cpp/ls/local_search.cpp
)
target_include_directories(_core PRIVATE cpp)
target_compile_definitions(_core PRIVATE KAYROS_VERSION="${SKBUILD_PROJECT_VERSION_FULL}")
# Bit-identical agreement with the Python reference checker requires plain
# IEEE-754 double arithmetic: no FMA contraction, ever.
target_compile_options(_core PRIVATE -ffp-contract=off)

if(KAYROS_NATIVE)
    target_compile_options(_core PRIVATE -march=native)
endif()

if(KAYROS_WITH_LERA)
    add_subdirectory(cpp/lera)
endif()

install(TARGETS _core DESTINATION kayros)
