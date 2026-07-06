# Build for the vendored Lera-Romero BPC component (see NOTICE.md).
# Only entered when KAYROS_WITH_LERA=ON. Requires a local CPLEX install and
# Boost, both provided by the nix dev shell (never by shipped wheels).

if("$ENV{CPLEX_INCLUDE}" STREQUAL "" OR "$ENV{CPLEX_BIN}" STREQUAL "")
    message(FATAL_ERROR "KAYROS_WITH_LERA needs CPLEX_INCLUDE and CPLEX_BIN in the environment. Enter the nix dev shell (nix develop ./nix-dev), which exports them.")
endif()
if("$ENV{BOOST_INCLUDE}" STREQUAL "" OR "$ENV{BOOST_BIN}" STREQUAL "")
    message(FATAL_ERROR "KAYROS_WITH_LERA needs BOOST_INCLUDE and BOOST_BIN in the environment. Enter the nix dev shell (nix develop ./nix-dev), which exports them.")
endif()

set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost REQUIRED COMPONENTS graph)

file(GLOB_RECURSE LERA_GOC_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/goc/src/*.cpp)
file(GLOB_RECURSE LERA_NYR_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/nyr/src/*.cpp)
file(GLOB_RECURSE LERA_MAIN_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/main/src/*.cpp)

add_library(lera STATIC ${LERA_GOC_SOURCES} ${LERA_NYR_SOURCES} ${LERA_MAIN_SOURCES})

# Upstream compiles as C++26; the vendored code predates kayros' C++23 baseline.
target_compile_features(lera PUBLIC cxx_std_26)
set_target_properties(lera PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Same bit-identical arithmetic policy as kayros' _core.
target_compile_options(lera PRIVATE -ffp-contract=off)

target_include_directories(lera PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/goc/include
    ${CMAKE_CURRENT_SOURCE_DIR}/nyr/include
    ${CMAKE_CURRENT_SOURCE_DIR}/main/include
    ${CMAKE_CURRENT_SOURCE_DIR}/magic_enum/include
)
target_include_directories(lera SYSTEM PUBLIC
    $ENV{CPLEX_INCLUDE}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(lera PUBLIC "$ENV{CPLEX_BIN}" ${Boost_LIBRARIES} dl m)

# kayros-owned bridge (bridge/ is not vendored code): MAMUT instance payload
# -> Lera preprocessing -> VRPInstance -> duration-objective BCP.
pybind11_add_module(_lera
    bridge/lera_module.cpp
    bridge/lera_bridge.cpp
)
target_include_directories(_lera PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/bridge)
target_compile_options(_lera PRIVATE -ffp-contract=off)
target_link_libraries(_lera PRIVATE lera)

install(TARGETS _lera DESTINATION kayros)
