#pragma once

#include <goc/goc.h>

namespace solver
{
// Check that instance is valid (i.e. feasible,
// has possible solutions, etc.).
// If not, corrects the instance to make it valid.
// Ordering: Comes after preprocess_triangle_depot.
void preprocess_validity(nlohmann::json& instance);
} // namespace