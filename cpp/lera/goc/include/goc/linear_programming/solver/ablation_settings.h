#pragma once
#include "goc/lib/json.hpp"
namespace goc {
inline nlohmann::json& ablation_backend_settings() {
    static thread_local nlohmann::json settings = nlohmann::json::object();
    return settings;
}

// A callback can start another solve on this same thread.
class BackendSettingsScope {
public:
    BackendSettingsScope() { ablation_backend_settings().swap(previous_); }
    ~BackendSettingsScope() { ablation_backend_settings().swap(previous_); }
    BackendSettingsScope(const BackendSettingsScope&) = delete;
    BackendSettingsScope& operator=(const BackendSettingsScope&) = delete;
private:
    nlohmann::json previous_ = nlohmann::json::object();
};
}
