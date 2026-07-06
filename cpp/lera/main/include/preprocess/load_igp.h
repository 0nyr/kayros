#pragma once

#include <goc/goc.h>
#include <nyr/nyr.h>

namespace solver
{

// Takes a JSON instance of a TDVRPTW and preprocesses it.
void preprocess_instance_from_json(nlohmann::json& instance);

/// Loads a TDVRPTW instance from a JSON file.
// nyr::VRPInstance load_instance_from_json(

/// Read a VRPInstance from a pre-parsed JSON object, *with* all the GOC
/// preprocessing steps applied.
nyr::VRPInstance load_instance_from_json(nlohmann::json const& instance);

} // namespace