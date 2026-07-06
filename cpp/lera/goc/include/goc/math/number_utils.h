//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#ifndef GOC_MATH_NUMBER_UTILS_H
#define GOC_MATH_NUMBER_UTILS_H

#include <functional>
#include <math.h>
#include <vector>

namespace goc
{
// Constant that represents the minimum granularity to be taken into account to compare numbers.
constexpr double EPS = 10e-7;
constexpr double EPS_SLOPE_ZERO = EPS*100; // Must be greater that classic EPS. Used with slope tests for special case where 2 slopes are close to 0.

// Constant representing a number that is bigger than the numbers that will be used.
const double INFTY = 10e50;

// Returns: if x = y with respect to the tolerance EPS.
inline bool epsilon_equal(double x, double y)
{
    return fabs(x - y) < EPS;
};

// Returns: if x != y with respect to the tolerance EPS.
inline bool epsilon_different(double x, double y)
{
    return fabs(x - y) >= EPS;
};

// Returns: if x < y with respect to the tolerance EPS.
inline bool epsilon_smaller(double x, double y)
{
    return x + EPS < y;
}

// Returns: if x <= y with respect to the tolerance EPS.
inline bool epsilon_smaller_equal(double x, double y)
{
    return x - EPS < y;
}

// Returns: if x > y with respect to the tolerance EPS.
inline bool epsilon_bigger(double x, double y)
{
    return x - EPS > y;
}

// Returns: if x >= y with respect to the tolerance EPS.
// WARN: Should NOT be used to compare with INFTY.
inline bool epsilon_bigger_equal(double x, double y)
{
    return x + EPS > y;
}

// Returns: if x is a number that is bigger or equal to INFTY.
// Remember that:
// >>> 10e50 + 0.00001 > 10e50
// False
// When comparing with INFTY, use direct comparison
inline bool is_plus_infty(double x)
{
    return x >= INFTY;
}

// Returns: the sum of all numbers in 'numbers'.
// Observation: the sum of an empty collection is 0.
inline double sum(const std::vector<double>& numbers)
{
    double a = 0.0;
    for (double n: numbers) a += n;
    return a;
}

// Returns: the sum of the value(e) for e \in v.
// Observation: the sum of an empty collection is 0.
template<typename T>
double sum(const std::vector<T>& v, const std::function<double(const T& e)>& value)
{
    double a = 0.0;
    for (auto& e: v) a += value(e);
    return a;
}
} // namespace goc

#endif // GOC_MATH_NUMBER_UTILS_H