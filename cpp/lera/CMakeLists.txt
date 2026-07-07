# Build for the vendored Lera-Romero BPC component (see NOTICE.md).
# Only entered when KAYROS_WITH_LERA=ON. LP/MIP backend is compile-time:
#   -DLERA_LP_BACKEND=highs (default) — fully open-source (HiGHS); kayros
#                                       stays one-line installable
#   -DLERA_LP_BACKEND=cplex           — optional fast backend, needs a local
#                                       CPLEX install (nix shell env)
# Boost is required either way (goc uses Boost.Graph).

set(LERA_LP_BACKEND "highs" CACHE STRING "LP/MIP backend for the Lera BPC: highs | cplex")
if(NOT LERA_LP_BACKEND MATCHES "^(cplex|highs)$")
    message(FATAL_ERROR "LERA_LP_BACKEND must be 'highs' or 'cplex' (got: ${LERA_LP_BACKEND})")
endif()
if(LERA_LP_BACKEND STREQUAL "highs" AND NOT "$ENV{CPLEX_BIN}" STREQUAL "")
    message(STATUS "Lera BPC: building the default HiGHS backend. A CPLEX install was detected (CPLEX_BIN set); rebuild with -DLERA_LP_BACKEND=cplex to use it.")
endif()

# Boost.Graph (goc uses it). BOOST_INCLUDE/BOOST_BIN (exported by the nix dev
# shell) act as hints when set; otherwise the standard find_package discovery
# applies (system boost-devel in CI/wheel images).
if(NOT "$ENV{BOOST_INCLUDE}" STREQUAL "")
    set(BOOST_ROOT "$ENV{BOOST_INCLUDE}/..")
    set(Boost_INCLUDE_DIR "$ENV{BOOST_INCLUDE}")
endif()
if(NOT "$ENV{BOOST_BIN}" STREQUAL "")
    set(BOOST_LIBRARYDIR "$ENV{BOOST_BIN}")
endif()
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost REQUIRED COMPONENTS graph)

file(GLOB_RECURSE LERA_GOC_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/goc/src/*.cpp)
file(GLOB_RECURSE LERA_NYR_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/nyr/src/*.cpp)
file(GLOB_RECURSE LERA_MAIN_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/main/src/*.cpp)

# Backend-specific sources: exactly one backend is compiled in.
file(GLOB_RECURSE LERA_CPLEX_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/goc/src/linear_programming/cplex/*.cpp)
file(GLOB_RECURSE LERA_HIGHS_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/goc/src/linear_programming/highs/*.cpp)
if(LERA_LP_BACKEND STREQUAL "highs")
    list(REMOVE_ITEM LERA_GOC_SOURCES ${LERA_CPLEX_SOURCES})
else()
    list(REMOVE_ITEM LERA_GOC_SOURCES ${LERA_HIGHS_SOURCES})
endif()

add_library(lera STATIC ${LERA_GOC_SOURCES} ${LERA_NYR_SOURCES} ${LERA_MAIN_SOURCES})

# kayros' C++23 baseline; the vendored code predates it (upstream used C++26
# permissively, but nothing needs it — 23 keeps GCC 13 toolchains buildable).
target_compile_features(lera PUBLIC cxx_std_23)
set_target_properties(lera PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Same bit-identical arithmetic policy as kayros' _core.
target_compile_options(lera PRIVATE -ffp-contract=off)

target_include_directories(lera PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/goc/include
    ${CMAKE_CURRENT_SOURCE_DIR}/nyr/include
    ${CMAKE_CURRENT_SOURCE_DIR}/main/include
    ${CMAKE_CURRENT_SOURCE_DIR}/magic_enum/include
)
target_include_directories(lera SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(lera PUBLIC ${Boost_LIBRARIES} dl m)

if(LERA_LP_BACKEND STREQUAL "highs")
    # Prefer an installed HiGHS (nix shell, -Dhighs_DIR=...); otherwise fetch
    # and build it statically so wheels are self-contained (M5.8). Pinned to
    # the release validated by the M5.7 certification campaign.
    find_package(highs CONFIG QUIET)
    if(NOT highs_FOUND)
        message(STATUS "Lera BPC: no installed HiGHS found — fetching v1.14.0 (static)")
        include(FetchContent)
        set(BUILD_SHARED_LIBS OFF)
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)
        set(BUILD_TESTING OFF)
        FetchContent_Declare(highs
            GIT_REPOSITORY https://github.com/ERGO-Code/HiGHS.git
            GIT_TAG        v1.14.0
            GIT_SHALLOW    TRUE
        )
        FetchContent_MakeAvailable(highs)
    endif()
    target_compile_definitions(lera PUBLIC GOC_LP_BACKEND_HIGHS)
    if(TARGET highs::highs)
        target_link_libraries(lera PUBLIC highs::highs)
    else()
        target_link_libraries(lera PUBLIC highs)
    endif()
else()
    if("$ENV{CPLEX_INCLUDE}" STREQUAL "" OR "$ENV{CPLEX_BIN}" STREQUAL "")
        message(FATAL_ERROR "LERA_LP_BACKEND=cplex needs CPLEX_INCLUDE and CPLEX_BIN in the environment. Enter the nix dev shell (nix develop ./nix-dev), which exports them, or build with -DLERA_LP_BACKEND=highs.")
    endif()
    target_include_directories(lera SYSTEM PUBLIC $ENV{CPLEX_INCLUDE})
    target_link_libraries(lera PUBLIC "$ENV{CPLEX_BIN}")
endif()

# kayros-owned bridge (bridge/ is not vendored code): MAMUT instance payload
# -> Lera preprocessing -> VRPInstance -> duration-objective BCP. Links the
# kayros checker-exact engine (cpp/pwlf + cpp/core route eval) for M5.6
# checker-exact column costs.
pybind11_add_module(_lera
    bridge/lera_module.cpp
    bridge/lera_bridge.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../pwlf/pwlf.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/route_eval.cpp
)
target_include_directories(_lera PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/bridge
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_compile_options(_lera PRIVATE -ffp-contract=off)
target_link_libraries(_lera PRIVATE lera)

install(TARGETS _lera DESTINATION kayros)
