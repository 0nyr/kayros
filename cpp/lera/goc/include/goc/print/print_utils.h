//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

// In this file we include functions for printing objects using the << operator, so they do not have to be reimplemented
// over and over again.

#include <bitset>
#include <deque>
#include <iostream>
#include <list>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#ifndef GOC_PRINT_PRINT_UTILS_H
#define GOC_PRINT_PRINT_UTILS_H

namespace goc
{
// Prints a sequence of elements in the ostream os.
// The output format is "prefix element1, element2, ..., elementn suffix".
// Precondition: The iterated elements must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename Collection>
std::ostream& print_iterable(std::ostream& os, const Collection& collection, char prefix='{', char suffix='}')
{
	os << prefix;
	for (auto it = collection.begin(); it != collection.end(); ++it)
	{
		if (it != collection.begin()) os << ", ";
		os << *it;
	}
	os << suffix;
	return os;
}

template<typename T>
void print_padded_vectors(
    std::ostream& os,
    const std::vector<T>& vec1,
    const std::vector<T>& vec2
) {
    static_assert(std::is_floating_point<T>::value, "print_padded_vectors requires floating point types");

    // Both vectors must have the same size.
    assert(vec1.size() == vec2.size() && "Both vectors must have the same size");

    if (vec1.empty()) {
        os << "[]" << std::endl;
        os << "[]" << std::endl;
        return;
    }
    
    // Convert each element of both vectors to its string representation.
    // std::showpoint forces a decimal point.
    std::vector<std::string> str_vec1, str_vec2;
    str_vec1.reserve(vec1.size());
    str_vec2.reserve(vec2.size());
    
    for (const auto& val : vec1) {
        std::ostringstream oss;
        oss << std::showpoint << val;
        str_vec1.push_back(oss.str());
    }
    for (const auto& val : vec2) {
        std::ostringstream oss;
        oss << std::showpoint << val;
        str_vec2.push_back(oss.str());
    }
    
    // Compute the maximum integer part length and maximum fraction length
    // across both vectors.
    size_t max_int_len = 0;
    size_t max_frac_len = 0;
    
    auto update_lengths = [&](const std::string& s) {
        size_t pos = s.find('.');
        size_t int_len = (pos == std::string::npos) ? s.size() : pos;
        size_t frac_len = (pos == std::string::npos) ? 0 : s.size() - pos - 1;
        max_int_len = std::max(max_int_len, int_len);
        max_frac_len = std::max(max_frac_len, frac_len);
    };

    for (const auto& s : str_vec1) {
        update_lengths(s);
    }
    for (const auto& s : str_vec2) {
        update_lengths(s);
    }
    
    // Helper lambda to print a vector using the computed field widths.
    auto print_vector = [&](const std::vector<std::string>& sv) {
        os << "[";
        bool first = true;
        for (const auto& s : sv) {
            if (!first) {
                os << ", ";
            } else {
                first = false;
            }
            size_t pos = s.find('.');
            size_t int_len = (pos == std::string::npos) ? s.size() : pos;
            size_t frac_len = (pos == std::string::npos) ? 0 : s.size() - pos - 1;
            size_t left_padding = (max_int_len > int_len) ? (max_int_len - int_len) : 0;
            size_t right_padding = (max_frac_len > frac_len) ? (max_frac_len - frac_len) : 0;
            os << std::string(left_padding, ' ') << s << std::string(right_padding, ' ');
        }
        os << "]";
    };

    print_vector(str_vec1);
    os << std::endl;
    print_vector(str_vec2);
    os << std::endl;
}

// Prints the pair in the ostream os.
// The output format is "(first, second)".
// Precondition: The types T1, T2 must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename T1, typename T2>
std::ostream& operator<<(std::ostream& os, const std::pair<T1, T2>& p)
{
	return os << "(" << p.first << ", " << p.second << ")";
}

// Prints the set in the ostream os.
// The output format is "{ element1, element2, ..., elementn }".
// Precondition: The type T must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename T>
std::ostream& operator<<(std::ostream& os, const std::set<T>& s)
{
	return print_iterable(os, s);
}

// Prints the set in the ostream os.
// The output format is "{ element1, element2, ..., elementn }".
// Precondition: The type T must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename T>
std::ostream& operator<<(std::ostream& os, const std::unordered_set<T>& s)
{
	return print_iterable(os, s);
}

// Prints the map in the ostream os.
// The output format is "{ (key1, value1), (key2, value2), ..., (keyn, valuen) }".
// Precondition: The types K, V must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::map<K, V>& m)
{
	return print_iterable(os, m);
}

// Prints the map in the ostream os.
// The output format is "{ (key1, value1), (key2, value2), ..., (keyn, valuen) }".
// Precondition: The types K, V must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename K, typename V>
std::ostream& operator<<(std::ostream& os, const std::unordered_map<K, V>& m)
{
	return print_iterable(os, m);
}

// Prints the vector in the ostream os.
// The output format is "[ element1, element2, ..., elementn ]".
// Precondition: The type T must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
	os << '[';
	for (auto it = v.begin(); it != v.end(); ++it)
	{
		if (it != v.begin()) os << ", ";
		os << *it;
	}
	os << ']';
	return os;
}

// Prints the deque in the ostream os.
// The output format is "[ element1, element2, ..., elementn ]".
// Precondition: The type T must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename T>
std::ostream& operator<<(std::ostream& os, const std::deque<T>& d)
{
	return print_iterable(os, d, '[', ']');
}

// Prints the list in the ostream os.
// The output format is "[ element1, element2, ..., elementn ]".
// Precondition: The type T must have the <<(ostream&) operator implemented.
// Returns: the modified ostream.
template<typename T>
std::ostream& operator<<(std::ostream& os, const std::list<T>& l)
{
	return print_iterable(os, l, '[', ']');
}

// Prints the bitset in the ostream os.
// The output format is "( n1, n2, ..., nk )" where n1, ..., nk are set in the bitset.
// Returns: the modified ostream.
template<unsigned long N>
std::ostream& operator<<(std::ostream& os, const std::bitset<N>& b)
{
	os << "(";
	bool first_added = true;
	for (unsigned long i = 0; i < N; ++i)
	{
		if (b.test(i))
		{
			if (!first_added) os << ", ";
			first_added = false;
			os << i;
		}
	}
	return os << ")";
}

} // namespace goc

#endif //GOC_PRINT_PRINT_UTILS_H
