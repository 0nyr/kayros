# Exact pricing pass of the Kayros exact layer (M2). It depends only on the
# exact header of MAMUT-routing-lib (target MAMUTRouting::td_exact, GMP) and on
# the standard library. Nothing links it yet, so every target here is excluded
# from the default build and the wheel is unchanged.
#
# Used from the top-level CMakeLists.txt, or alone:
#   cmake -S cpp/exact -B build-exact -DMAMUTRouting_DIR="$(python -m mamut_routing_lib.cmake)"
#   cmake --build build-exact --target kayros_exact_tests
# kayros_exact_tests builds and runs the gates (tests/cpp/test_exact_pricing.cpp)
# on the pass and the mutation check on 3 mutated copies of the pass.
cmake_minimum_required(VERSION 3.26)
project(kayros_exact LANGUAGES CXX)

if(NOT TARGET MAMUTRouting::td_exact)
    find_package(MAMUTRouting CONFIG REQUIRED)
endif()

set(KAYROS_EXACT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(KAYROS_EXACT_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(KAYROS_EXACT_TEST_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/cpp/test_exact_pricing.cpp)

function(kayros_exact_library name source)
    add_library(${name} STATIC EXCLUDE_FROM_ALL ${source})
    target_include_directories(${name} PUBLIC ${KAYROS_EXACT_CPP_DIR})
    target_link_libraries(${name} PUBLIC MAMUTRouting::td_exact)
    target_compile_features(${name} PUBLIC cxx_std_17)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
endfunction()

kayros_exact_library(kayros_exact ${KAYROS_EXACT_DIR}/exact_pricing.cpp)

# Command-line driver for gates and measurements on exported instances.
add_executable(kayros_exact_price EXCLUDE_FROM_ALL ${KAYROS_EXACT_DIR}/price_tool.cpp)
target_link_libraries(kayros_exact_price PRIVATE kayros_exact)

add_executable(kayros_exact_test EXCLUDE_FROM_ALL ${KAYROS_EXACT_TEST_SOURCE})
target_link_libraries(kayros_exact_test PRIVATE kayros_exact)
target_compile_options(kayros_exact_test PRIVATE -Wall -Wextra)

# Mutation check (criterion 2 of the M2 note). Each mutant is a copy of
# exact_pricing.cpp with 1 piece of text replaced, written at configure time,
# so the trusted source carries no switch. Configuration fails when the text
# does not occur exactly once. The gates built on a mutant must report a wrong
# minimum (exit 0 of the test built with KAYROS_EXACT_MUTANT_TEST).
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${KAYROS_EXACT_DIR}/exact_pricing.cpp)
file(READ ${KAYROS_EXACT_DIR}/exact_pricing.cpp KAYROS_EXACT_SOURCE)
set(KAYROS_EXACT_TEST_RUNS COMMAND kayros_exact_test)
function(kayros_exact_mutant name from to)
    string(FIND "${KAYROS_EXACT_SOURCE}" "${from}" first)
    string(FIND "${KAYROS_EXACT_SOURCE}" "${from}" last REVERSE)
    if(first EQUAL -1 OR NOT first EQUAL last)
        message(FATAL_ERROR "mutant ${name}: the text to replace must occur exactly once in exact_pricing.cpp")
    endif()
    string(REPLACE "${from}" "${to}" mutated "${KAYROS_EXACT_SOURCE}")
    set(source ${CMAKE_CURRENT_BINARY_DIR}/mutant_${name}.cpp)
    file(WRITE ${source} "${mutated}")
    kayros_exact_library(kayros_exact_mutant_${name} ${source})
    add_executable(kayros_exact_mutant_test_${name} EXCLUDE_FROM_ALL ${KAYROS_EXACT_TEST_SOURCE})
    target_compile_definitions(kayros_exact_mutant_test_${name} PRIVATE KAYROS_EXACT_MUTANT_TEST)
    target_link_libraries(kayros_exact_mutant_test_${name} PRIVATE kayros_exact_mutant_${name})
    add_executable(kayros_exact_price_mutant_${name} EXCLUDE_FROM_ALL ${KAYROS_EXACT_DIR}/price_tool.cpp)
    target_link_libraries(kayros_exact_price_mutant_${name} PRIVATE kayros_exact_mutant_${name})
    set(KAYROS_EXACT_TEST_RUNS ${KAYROS_EXACT_TEST_RUNS} COMMAND kayros_exact_mutant_test_${name} PARENT_SCOPE)
    set(KAYROS_EXACT_MUTANTS ${KAYROS_EXACT_MUTANTS} ${name} PARENT_SCOPE)
endfunction()
kayros_exact_mutant(cut_penalty "pen += w[c];" "{}")
kayros_exact_mutant(set_inclusion "(M.U & ~L.U).any() || " "")
kayros_exact_mutant(right_limit "if (k + 1 < bp.size() && G(bp[k], true) > eval(f, bp[k], true)) return false;" "")

add_custom_target(kayros_exact_tests ${KAYROS_EXACT_TEST_RUNS}
    COMMENT "Running the gates of the exact pricing pass and the mutation check"
    VERBATIM)
add_dependencies(kayros_exact_tests kayros_exact_test)
foreach(name IN LISTS KAYROS_EXACT_MUTANTS)
    add_dependencies(kayros_exact_tests kayros_exact_mutant_test_${name})
endforeach()
