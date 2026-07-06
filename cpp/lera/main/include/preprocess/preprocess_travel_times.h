#pragma once

#include <goc/goc.h>

namespace solver
{
void preprocess_constant_travel_times(nlohmann::json& instance);

// Adds the travel_times attribute to a JSON instance with a matrix of piecewise linear functions.
void preprocess_igp_travel_times(nlohmann::json& instance);

void preprocess_piecewise_constant_travel_times(nlohmann::json& instance);
} // namespace
