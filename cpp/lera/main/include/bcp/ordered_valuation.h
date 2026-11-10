#pragma once

#include <algorithm>
#include <utility>
#include <vector>

#include "goc/linear_programming/model/valuation.h"

namespace solver
{
// Variable identity hashes contain allocation addresses. Numerical reductions
// instead visit the current formulation columns in a reproducible order.
inline std::vector<std::pair<goc::Variable, double>> ordered_valuation(const goc::Valuation& valuation)
{
    std::vector<std::pair<goc::Variable, double>> values(valuation.begin(), valuation.end());
    std::sort(values.begin(), values.end(), [](const auto& left, const auto& right) {
        return left.first.Index() < right.first.Index();
    });
    return values;
}
} // namespace solver
