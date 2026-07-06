#pragma once

#include <random>
#include <thread>
#include <boost/functional/hash.hpp>

namespace nyr {

// Global seed for reproducibility
inline constexpr uint32_t GLOBAL_SEED = 1;

// Uses Boost's robust hash_combine to derive a per-thread seed from a global seed
inline uint32_t thread_seed(uint32_t global_seed, std::size_t tid) {
    std::size_t combined = global_seed;
    boost::hash_combine(combined, tid);
    return static_cast<uint32_t>(combined); // truncate to 32-bit
}

// Thread-local RNG engine
inline std::mt19937& thread_engine() {
    static thread_local std::mt19937 engine([] {
        std::size_t tid_hash = std::hash<std::thread::id>{}(std::this_thread::get_id());
        return std::mt19937(thread_seed(GLOBAL_SEED, tid_hash));
    }());
    return engine;
}

// Reproducible thread-local RNG returning a double in [0.0, 1.0] (inclusive)
inline double rand01() {
    static thread_local std::uniform_real_distribution<double> dist(0.0, 1.0);
    return dist(thread_engine());
}

// Reproducible thread-local RNG returning an integer in [a, b] (inclusive)
inline uint32_t rand_int(uint32_t a, uint32_t b) {
    std::uniform_int_distribution<uint32_t> dist(a, b);
    return dist(thread_engine());
}

// Reproducible thread-local RNG returning a random index 
// for a container of the provided size
// Returns an index in integer range [0, size[
inline std::size_t rand_index(std::size_t size) {
    if (size == 0) return 0;
    std::uniform_int_distribution<std::size_t> dist(0, size - 1);
    return dist(thread_engine());
}


} // namespace nyr
