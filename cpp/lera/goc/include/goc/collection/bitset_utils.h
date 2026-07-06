//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_COLLECTION_BITSET_UTILS_H
#define GOC_COLLECTION_BITSET_UTILS_H

#include <vector>
#include <bitset>
#include "goc/lib/json.hpp"

namespace goc
{
// Returns: A bitset with the numbers given.
template<unsigned long N>
std::bitset<N> create_bitset(
	const std::vector<int>& numbers
) {
	std::bitset<N> b;
	for (size_t n: numbers) b.set(n);
	return b;
}

// Returns: The intersection of bitsets b1 and b2.
template<unsigned long N>
inline std::bitset<N> intersection(const std::bitset<N>& b1, const std::bitset<N>& b2)
{
	return b1 & b2;
}

// Returns: The intersection of bitset b1 and the numbers in the second parameter.
template<unsigned long N>
inline std::bitset<N> intersection(const std::bitset<N>& b1, const std::initializer_list<int>& numbers)
{
	return b1 & create_bitset<N>(numbers);
}

// Returns: The union of bitsets b1 and b2.
template<unsigned long N>
inline std::bitset<N> unite(const std::bitset<N>& b1, const std::bitset<N>& b2)
{
	return b1 | b2;
}

// Returns: The union of bitset b1 and the numbers in the second parameter.
template<unsigned long N>
inline std::bitset<N> unite(const std::bitset<N>& b1, const std::initializer_list<int>& numbers)
{
	return b1 | create_bitset<N>(numbers);
}

// Returns: The union of bitset b1 and the numbers in the second parameter.
template<unsigned long N>
inline std::bitset<N> unite(const std::bitset<N>& b1, const std::vector<int>& numbers)
{
	return b1 | create_bitset<N>(numbers);
}

// Returns: if b1 is a subset of b2
template<unsigned long N>
inline bool is_subset(const std::bitset<N>& b1, const std::bitset<N>& b2)
{
	return (b1 & b2) == b1;
}

// Returns: if number is in the bitset.
template<unsigned long N>
inline bool contains(const std::bitset<N>& b, int number)
{
	return b.test(number);
}

// Returns: the difference of bitsets b1 and b2.
template<unsigned long N>
inline std::bitset<N> difference(const std::bitset<N>& b1, const std::bitset<N>& b2)
{
	return b1 & ~b2;
}

template <std::size_t N>
std::bitset<N> set_first(int n)
{
    std::bitset<N> result;
    if (n <= 0) return result;
    if (n >= int(N)) return result.set();

    int i = 0;
    constexpr int Chunk = 64;
    while (n - i >= Chunk) // Fill full 64-bit chunks
    {
        result |= (std::bitset<N>(~0ULL) << i);
        i += Chunk;
    }
	// Fill remaining bits
    if (i < n)
    {
        uint64_t mask = (1ULL << (n - i)) - 1;
        result |= (std::bitset<N>(mask) << i);
    }

    return result;
}

// Returns: the number of bits set to 1 in the bitset.
template <std::size_t N>
inline std::size_t nb_bits_set(const std::bitset<N>& b)
{
	return b.count();
}

}// namespace goc


namespace std
{
template<unsigned long N>
void to_json(nlohmann::json& j, const std::bitset<N>& b)
{
	j = std::vector<int>();
	for (int i = 0; i < b.size(); ++i) if (b.test(i)) j.push_back(i);
}

template<unsigned long N>
void from_json(const nlohmann::json& j, std::bitset<N>& b)
{
	for (int i: j) b.set(i);
}
} // namespace std

#endif //GOC_COLLECTION_BITSET_UTILS_H
