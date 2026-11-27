# Exact LP multipliers of the restricted master (Kayros exact layer, M2 step 4).
# The library depends only on GMP, taken from the exact header target of
# MAMUT-routing-lib as in cpp/exact. Nothing links it yet, so every target is
# excluded from the default build and the wheel is unchanged.
#
# kayros_exact_lp_tests builds and runs the tests: tests/cpp/test_exact_lp.cpp
# (library alone, hand-built masters and fixtures of real root masters) and,
# when the Lera BPC is configured, tests/cpp/test_lp_basis.cpp (basis export of
# the configured LP backend, solved exactly by the library).
# kayros_exact_lp_master is the driver on master files (master_tool.cpp).
cmake_minimum_required(VERSION 3.26)
project(kayros_exact_lp LANGUAGES CXX)

if(NOT TARGET MAMUTRouting::td_exact)
    find_package(MAMUTRouting CONFIG REQUIRED)
endif()

set(KAYROS_EXACT_LP_TEST_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/cpp)
set(KAYROS_EXACT_LP_DATA ${KAYROS_EXACT_LP_TEST_DIR}/data/exact_lp)

add_library(kayros_exact_lp STATIC EXCLUDE_FROM_ALL ${CMAKE_CURRENT_SOURCE_DIR}/exact_lp.cpp)
target_include_directories(kayros_exact_lp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(kayros_exact_lp PUBLIC MAMUTRouting::td_exact)
target_compile_features(kayros_exact_lp PUBLIC cxx_std_17)
target_compile_options(kayros_exact_lp PRIVATE -Wall -Wextra)
set_target_properties(kayros_exact_lp PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(kayros_exact_lp_test EXCLUDE_FROM_ALL ${KAYROS_EXACT_LP_TEST_DIR}/test_exact_lp.cpp)
target_link_libraries(kayros_exact_lp_test PRIVATE kayros_exact_lp)
target_compile_definitions(kayros_exact_lp_test PRIVATE KAYROS_EXACT_LP_DATA="${KAYROS_EXACT_LP_DATA}")
target_compile_options(kayros_exact_lp_test PRIVATE -Wall -Wextra)
set(KAYROS_EXACT_LP_TEST_RUNS COMMAND kayros_exact_lp_test)

if(TARGET lera)
    add_executable(kayros_exact_lp_master EXCLUDE_FROM_ALL ${CMAKE_CURRENT_SOURCE_DIR}/master_tool.cpp)
    target_link_libraries(kayros_exact_lp_master PRIVATE kayros_exact_lp lera)
    target_compile_options(kayros_exact_lp_master PRIVATE -ffp-contract=off)

    add_executable(kayros_exact_lp_backend_test EXCLUDE_FROM_ALL ${KAYROS_EXACT_LP_TEST_DIR}/test_lp_basis.cpp)
    target_link_libraries(kayros_exact_lp_backend_test PRIVATE kayros_exact_lp lera)
    target_compile_definitions(kayros_exact_lp_backend_test PRIVATE KAYROS_EXACT_LP_DATA="${KAYROS_EXACT_LP_DATA}")
    target_compile_options(kayros_exact_lp_backend_test PRIVATE -ffp-contract=off)
    list(APPEND KAYROS_EXACT_LP_TEST_RUNS COMMAND kayros_exact_lp_backend_test)
endif()

add_custom_target(kayros_exact_lp_tests ${KAYROS_EXACT_LP_TEST_RUNS}
    COMMENT "Running the tests of the exact LP component"
    VERBATIM)
add_dependencies(kayros_exact_lp_tests kayros_exact_lp_test)
if(TARGET lera)
    add_dependencies(kayros_exact_lp_tests kayros_exact_lp_backend_test)
endif()
