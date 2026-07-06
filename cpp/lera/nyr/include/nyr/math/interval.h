#pragma once

#include <iostream>
#include <vector>

#include <goc/goc.h>

namespace nyr {

/**
 * This file contains functions for working with intervals
 * represented as sorted vectors of doubles.
 */

/// “Is empty” in the same sense as Interval::Empty().
inline bool interval_vector_empty(const std::vector<double>& v) {
    // we treat empty vector or front()>back() as “empty interval”
    return v.empty() || goc::epsilon_bigger(v.front(), v.back());
}

/// Single‐point interval?
inline bool interval_vector_is_point(const std::vector<double>& v) {
    return !v.empty() && goc::epsilon_equal(v.front(), v.back());
}

/// Returns: true if values is within the domain.
/// Precondition: the vector is sorted and has at least 1 element.
inline bool interval_vector_includes(
    const std::vector<double>& v,
    const double value
) {
    return !v.empty() &&
            goc::epsilon_smaller_equal(v.front(), value) &&
               goc::epsilon_smaller_equal(value, v.back());
}

/// Does [v] sit inside [w]?
inline bool interval_vector_is_included_in(
    const std::vector<double>& v,
    const std::vector<double>& w
) {
    if (v.empty()) return true; // empty is subset of anything
    if (w.empty()) return false; // non-empty is not subset of empty
    return goc::epsilon_bigger_equal(v.front(), w.front())
        && goc::epsilon_smaller_equal(v.back(),  w.back());
}

/// Do [v] and [w] overlap at all?
inline bool interval_vector_intersects(
    const std::vector<double>& v,
    const std::vector<double>& w
) {
    if (v.empty() || w.empty()) return false; // empty intervals do not intersect
    return !(goc::epsilon_smaller(v.back(), w.front()) ||
             goc::epsilon_bigger(v.front(), w.back()));
}

/// Do [v] and [w] not overlap at all?
inline bool interval_vector_disjoints(
    const std::vector<double>& v,
    const std::vector<double>& w
) {
 if (v.empty() || w.empty()) return true; // empty intervals do not intersect
 return goc::epsilon_smaller(v.back(), w.front()) ||
    goc::epsilon_bigger(v.front(), w.back());
}

// Tests for the interval_vector functions.
void test_interval_vector_intersects();
void test_interval_vector_includes();


} // namespace nyr
