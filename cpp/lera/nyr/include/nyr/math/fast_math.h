#pragma once

#include <cstdint>   // for uint32_t
#include <climits>   // for CHAR_BIT
#include <type_traits>

namespace nyr
{
// Returns the number of bits required to represent the number n.
// which approximates the logarithm in base 2 of n when 
// cast to an integer.
// Uses constexpr if to select the correct builtin function at compile time.
// NOTE: Returns 0 for n = 0 (log2(0) is undefined, -inf)
template<typename T>
inline T fast_log2(T n) {
    static_assert(std::is_unsigned<T>::value, "fast_log2 requires unsigned type");
    if (n == 0) return 0;

    if constexpr (sizeof(T) == 4) {  // 32-bit
        return 31 - __builtin_clz(n);
    } else if constexpr (sizeof(T) == 8) {  // 64-bit
        return 63 - __builtin_clzll(n);
    } else {
        static_assert(sizeof(T) == 4 || sizeof(T) == 8, "Unsupported input size in fast_log2");
        return 0;
    }
}

/**
 * Fast exponentiation function.
 * Based on exponentiation by squaring.
 * Complexity: O(log n) multiplications.
 */
template <typename Base, typename Exponent>
requires std::is_floating_point_v<Base> && std::is_integral_v<Exponent>
constexpr Base fast_pow(Base base, Exponent exp)
{
    if (exp == 0) return static_cast<Base>(1.0);
    if (exp < 0) return static_cast<Base>(1.0) / fast_pow(base, -exp);

    Base result = static_cast<Base>(1.0);
    while (exp > 0)
    {
        if (exp % 2 == 1)
            result *= base;
        base *= base;
        exp /= 2;
    }
    return result;
}

} // namespace