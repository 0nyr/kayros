#pragma once
#include "goc/lib/json.hpp"
namespace goc {
inline nlohmann::json& ablation_backend_settings() {
    static thread_local nlohmann::json settings = nlohmann::json::object();
    return settings;
}
}
