# Exact pricing pass of the Kayros exact layer (M2). It depends only on the
# exact header of MAMUT-routing-lib (target MAMUTRouting::td_exact, GMP) and on
# the standard library. Nothing links it yet, so every target here is excluded
# from the default build and the wheel is unchanged.
#
# Used from the top-level CMakeLists.txt, or alone:
#   cmake -S cpp/exact -B build-exact -DMAMUTRouting_DIR="$(python -m mamut_routing_lib.cmake)"
#   cmake --build build-exact --target kayros_exact_tests
# kayros_exact_tests builds and runs the gates (tests/cpp/test_exact_pricing.cpp)
# on the pass and the mutation check on 3 mutated copies of the pass, then the
# tests of the ledger (tests/cpp/test_ledger.cpp), which run the verifier
# kayros_ledger_verify, and the mutation check on mutated copies of the ledger.
cmake_minimum_required(VERSION 3.26)
project(kayros_exact LANGUAGES CXX)

if(NOT TARGET MAMUTRouting::td_exact)
    find_package(MAMUTRouting CONFIG REQUIRED)
endif()

set(KAYROS_EXACT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(KAYROS_EXACT_CPP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(KAYROS_EXACT_TEST_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/cpp/test_exact_pricing.cpp)

function(kayros_exact_library name source)
    add_library(${name} STATIC EXCLUDE_FROM_ALL ${source} ${ARGN})
    target_include_directories(${name} PUBLIC ${KAYROS_EXACT_CPP_DIR})
    target_link_libraries(${name} PUBLIC MAMUTRouting::td_exact)
    target_compile_features(${name} PUBLIC cxx_std_17)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
endfunction()

kayros_exact_library(kayros_exact ${KAYROS_EXACT_DIR}/exact_pricing.cpp ${KAYROS_EXACT_DIR}/ledger.cpp)

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

# Verifier of a ledger record (criterion 6). It links kayros_exact only, whose
# only dependency is MAMUTRouting::td_exact (the header and GMP). This directory
# configures on its own without cpp/lera or an LP backend, so the verifier cannot
# reach them. The checks below fail the configuration if a link is added.
add_executable(kayros_ledger_verify EXCLUDE_FROM_ALL ${KAYROS_EXACT_DIR}/ledger_verify.cpp)
target_link_libraries(kayros_ledger_verify PRIVATE kayros_exact)
target_compile_options(kayros_ledger_verify PRIVATE -Wall -Wextra)
get_target_property(KAYROS_VERIFY_LINKS kayros_ledger_verify LINK_LIBRARIES)
get_target_property(KAYROS_EXACT_LINKS kayros_exact LINK_LIBRARIES)
if(NOT KAYROS_VERIFY_LINKS STREQUAL "kayros_exact" OR NOT KAYROS_EXACT_LINKS STREQUAL "MAMUTRouting::td_exact")
    message(FATAL_ERROR "kayros_ledger_verify must link kayros_exact only, and kayros_exact MAMUTRouting::td_exact only")
endif()

# Tests of the ledger, and the mutation check of the ledger in the same style as
# that of the pass. Each mutant is a copy of ledger.cpp with 1 piece of text
# replaced, linked with the unmutated pass. The test built with
# KAYROS_LEDGER_MUTANT_TEST exits 0 only when some check fails on the mutant.
set(KAYROS_LEDGER_TEST_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/../../tests/cpp/test_ledger.cpp)
# The test runs the verifier built on the same library, so that a mutant of the
# replay code is exercised by the tampered records.
function(kayros_ledger_test name library verifier)
    add_executable(${name} EXCLUDE_FROM_ALL ${KAYROS_LEDGER_TEST_SOURCE})
    target_link_libraries(${name} PRIVATE ${library})
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    target_compile_definitions(${name} PRIVATE
        KAYROS_LEDGER_VERIFY_PATH="$<TARGET_FILE:${verifier}>"
        KAYROS_LEDGER_TMP="${CMAKE_CURRENT_BINARY_DIR}/${name}" ${ARGN})
    add_dependencies(${name} ${verifier})
endfunction()
kayros_ledger_test(kayros_ledger_test kayros_exact kayros_ledger_verify)
set(KAYROS_EXACT_TEST_RUNS ${KAYROS_EXACT_TEST_RUNS} COMMAND kayros_ledger_test)

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${KAYROS_EXACT_DIR}/ledger.cpp)
file(READ ${KAYROS_EXACT_DIR}/ledger.cpp KAYROS_LEDGER_SOURCE)
function(kayros_ledger_mutant name from to)
    string(FIND "${KAYROS_LEDGER_SOURCE}" "${from}" first)
    string(FIND "${KAYROS_LEDGER_SOURCE}" "${from}" last REVERSE)
    if(first EQUAL -1 OR NOT first EQUAL last)
        message(FATAL_ERROR "ledger mutant ${name}: the text to replace must occur exactly once in ledger.cpp")
    endif()
    string(REPLACE "${from}" "${to}" mutated "${KAYROS_LEDGER_SOURCE}")
    set(source ${CMAKE_CURRENT_BINARY_DIR}/ledger_mutant_${name}.cpp)
    file(WRITE ${source} "${mutated}")
    kayros_exact_library(kayros_ledger_mutant_${name} ${KAYROS_EXACT_DIR}/exact_pricing.cpp ${source})
    add_executable(kayros_ledger_verify_mutant_${name} EXCLUDE_FROM_ALL ${KAYROS_EXACT_DIR}/ledger_verify.cpp)
    target_link_libraries(kayros_ledger_verify_mutant_${name} PRIVATE kayros_ledger_mutant_${name})
    kayros_ledger_test(kayros_ledger_mutant_test_${name} kayros_ledger_mutant_${name} kayros_ledger_verify_mutant_${name}
        KAYROS_LEDGER_MUTANT_TEST)
    set(KAYROS_EXACT_TEST_RUNS ${KAYROS_EXACT_TEST_RUNS} COMMAND kayros_ledger_mutant_test_${name} PARENT_SCOPE)
    set(KAYROS_LEDGER_MUTANTS ${KAYROS_LEDGER_MUTANTS} ${name} PARENT_SCOPE)
endfunction()
kayros_ledger_mutant(kappa_one "const Q kappa(fleet_ ? std::min(n, *fleet_) : n);" "const Q kappa(1);")
kayros_ledger_mutant(bound_minus_one "return L >= *U_;" "return L > *U_ - 1;")
kayros_ledger_mutant(bound_always "return L >= *U_;" "return true;")
kayros_ledger_mutant(right_no_depot "for (int k = 0; k <= n; ++k) {" "for (int k = 1; k <= n; ++k) {")
kayros_ledger_mutant(sigma_sign "if (s > 0) return c.reason = \"positive cut multiplier\", c;" "{}")
kayros_ledger_mutant(no_partition "if (visits[v] != 1) return" "if (false) return")
kayros_ledger_mutant(prune_always "if (!reaches(*it->second.best->bound)) return p.reason = \"stored bound below U\", p;" "")
kayros_ledger_mutant(fleet_below_one "if (fleet_ && *fleet_ < 1) throw std::invalid_argument(\"fleet limit below 1\");" "")
kayros_ledger_mutant(no_capacity "if (instance_.dem[v] > instance_.cap - load) return" "if (false) return")
kayros_ledger_mutant(pi_zero "if (sgn(cert.pi[0]) != 0) return" "if (false) return")
kayros_ledger_mutant(no_strict "if (U_ && !(cost < *U_)) return" "if (U_ && cost > *U_) return")
kayros_ledger_mutant(no_fleet_check "if (fleet_ && routes.size() > static_cast<std::size_t>(*fleet_)) return" "if (false) return")
kayros_ledger_mutant(no_depot_in_route "if (v < 1 || v > n) return o.reason = \"route vertex not a customer\", o;" "if (v < 0 || v > n) return o.reason = \"route vertex not a customer\", o;")
kayros_ledger_mutant(replay_no_bound_check "if (r.delta != c.delta || r.bound != c.bound) return" "if (false) return")
kayros_ledger_mutant(replay_no_U_check "if (!ledger.U() || *ledger.U() != op.U) return" "if (!ledger.U()) return")

add_custom_target(kayros_exact_tests ${KAYROS_EXACT_TEST_RUNS}
    COMMENT "Running the gates of the exact pricing pass, the tests of the ledger and the mutation checks"
    VERBATIM)
add_dependencies(kayros_exact_tests kayros_exact_test)
foreach(name IN LISTS KAYROS_EXACT_MUTANTS)
    add_dependencies(kayros_exact_tests kayros_exact_mutant_test_${name})
endforeach()
add_dependencies(kayros_exact_tests kayros_ledger_test)
foreach(name IN LISTS KAYROS_LEDGER_MUTANTS)
    add_dependencies(kayros_exact_tests kayros_ledger_mutant_test_${name})
endforeach()
