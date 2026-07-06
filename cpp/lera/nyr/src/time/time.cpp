#include "nyr/time/time.h"

#include <iostream>

/**
 * Generate a timestamp string.
 */
std::string generate_timestamp() {
    // Get the current time
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);

    // Format it to YYYY-MM-DD_HH-MM-SS
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}