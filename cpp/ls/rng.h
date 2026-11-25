#pragma once

#include <cstdint>
#include <random>
#include <utility>
#include <vector>

namespace kayros {

// Draw in [0, n) by reducing the caller's 64-bit RNG output modulo n. Perturbation and fleet descent use the same rule across standard-library implementations.
inline std::uint64_t draw(std::mt19937_64& rng, std::uint64_t n) {
    return rng() % n;
}

inline void fisher_yates(std::vector<std::int32_t>& v, std::mt19937_64& rng) {
    for (std::size_t i = v.size(); i > 1; --i) {
        std::swap(v[i - 1], v[static_cast<std::size_t>(draw(rng, i))]);
    }
}

}  // namespace kayros
