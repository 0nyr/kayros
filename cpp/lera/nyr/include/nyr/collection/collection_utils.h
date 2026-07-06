#pragma once

#include <goc/goc.h>

#include <vector>

namespace nyr {

/// Return: The vector of indices sorted with respect to an associated
/// vector of values, in ascending order.
template<typename T>
std::vector<size_t> sort_indices_by_ascending_values(const std::vector<T>& values) {
    std::vector<size_t> indices(values.size());
    for (size_t i = 0; i < values.size(); ++i) {
        indices[i] = i;
    }
    std::sort(indices.begin(), indices.end(),
        [&values](size_t i1, size_t i2) {
            return values[i1] < values[i2];
        });
    return indices;
}

} // namespace nyr
