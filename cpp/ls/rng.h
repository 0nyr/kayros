#pragma once

#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace kayros {

// Platform-stable integer draw in [0, n): modulo over the full 64-bit stream
// (std::uniform_int_distribution is implementation-defined; bias at these
// ranges is negligible and determinism wins). Hoisted verbatim from
// perturb.cpp (M3, plan 12) so the fleet-descent phase draws identically.
inline std::uint64_t draw(std::mt19937_64& rng, std::uint64_t n) {
    return rng() % n;
}

inline void fisher_yates(std::vector<std::int32_t>& v, std::mt19937_64& rng) {
    for (std::size_t i = v.size(); i > 1; --i) {
        std::swap(v[i - 1], v[static_cast<std::size_t>(draw(rng, i))]);
    }
}

}  // namespace kayros
