//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#pragma once

#include <iostream>

#include "goc/math/interval.h"
#include "goc/math/number_utils.h"
#include "goc/math/point_2d.h"
#include "goc/lib/json.hpp"
#include "goc/print/printable.h"

namespace goc
{
// This class represents a segment or bounded linear function.
// An instance can be represented by two points (p1, p2) and the function
// connecting those points.
// Invariant: p1.x <= p2.x.
class LinearFunction : public Printable
{
public:
    Interval domain, image;
    double slope, intercept;
    
    LinearFunction() = default;

    // Creates a linear function from p1 to p2.
    // Precondition: p1.x <= p2.x.
    LinearFunction(const Point2D& p1, const Point2D& p2);

    // Verticals (zero-width pieces with distinct image endpoints) come in two
    // kinds with different composition semantics (M5.9, design memo 13.2):
    //  - JUMP (slope = +INFTY): a genuine discontinuity of a pointwise function
    //    (e.g. a stepwise travel-time up-step in arr). Interior span values are
    //    unattained; composition discards them.
    //  - CHOICE (slope = -INFTY): the inverse of a plateau (dep at a waiting
    //    plateau or a slope-0 arr stretch). Every interior value is a genuinely
    //    attainable choice; composition SWEEPS the outer function over the span.
    // Both store the attained/representative value in `intercept` (13.1): for a
    // choice vertical that is the duration-optimal representative (the latest
    // departure, goc's historical max{x : f(x) = y} inverse semantics).
    // A plateau (slope 0) and a single point are NOT verticals.
    bool is_vertical() const { return slope == INFTY || slope == -INFTY; }
    bool is_jump_vertical() const { return slope == INFTY; }
    bool is_choice_vertical() const { return slope == -INFTY; }

    // Returns: a copy of this vertical re-tagged as a CHOICE vertical.
    // Precondition: is_vertical().
    LinearFunction as_choice_vertical() const
    {
        LinearFunction c = *this;
        c.slope = -INFTY;
        return c;
    }

    // Returns: the (incoming, outgoing) endpoint values in sweep (left-to-right)
    // order. For an ordinary piece these are Value(domain.left), Value(domain.right).
    // For a vertical (value jump) Value returns only the incoming endpoint, so we
    // return both image endpoints, incoming first (intercept holds the
    // left-continuous incoming value). M5.9: preserves jumps under pointwise
    // arithmetic (operator+/operator*), which otherwise collapse them.
    std::pair<double, double> sweep_endpoints() const;
    
    // Returns: the value of the function f(x).
    // Precondition: x \in domain.
    double Value(double x) const;
    
    // Returns: the value of the function f(x).
    // Precondition: x \in domain.
    double operator()(double x) const;
    
    // Returns: the last x such that f(x) = y.
    // Observation: if slope == 0 the function is not inversible therefore many f(x) = y might exist.
    // Precondition: y \in image.
    double PreValue(double y) const;
    
    // Returns: if this function intersects the function f inside both functions domains.
    bool Intersects(const LinearFunction& f) const;
    
    // Returns: the coordinate x such that this(x) == f(x).
    // Precondition: Intersects(*this, f).
    double Intersection(const LinearFunction& f) const;
    
    // Returns: the inverse function of this function (f) if exists, otherwise returns g(y) = max{x : f(x)=y }.
    LinearFunction Inverse() const;
    
    // Restricts the domain.
    // Returns: the restricted function.
    LinearFunction RestrictDomain(const Interval& domain) const;
    
    // Restricts the image.
    // Returns: the restricted function.
    LinearFunction RestrictImage(const Interval& image) const;
    
    // Prints the function.
    // Format: {(x1, y1)->(x2,y2)}.
    void Print(std::ostream& os) const;
    
    // Returns: if two functions have the same p1 and p2.
    bool operator==(const LinearFunction& f) const;
    
    // Returns: if two functions have different p1 and p2.
    bool operator!=(const LinearFunction& f) const;

    // Returns the total memory footprint of *this* object, in bytes,
    // including both the fixed‐size portion (sizeof(*this)) and any
    // heap allocations (e.g. std::vector buffers).
    std::size_t memory_footprint_bytes() const;    
};

// JSON format: [p1, p2].
void from_json(const nlohmann::json& j, LinearFunction& f);

void to_json(nlohmann::json& j, const LinearFunction& f);

// Returns: f.domain
inline Interval dom(const LinearFunction& f) { return f.domain; }

// Returns: f.image
inline Interval img(const LinearFunction& f) { return f.image; }

// Returns: h(x) = f(x)+g(x).
// Obs: dom(h) = dom(f) \cap dom(g).
LinearFunction operator+(const LinearFunction& f, const LinearFunction& g);

// Returns: h(x) = f(x)*g(x).
// Obs: dom(h) = dom(f) \cap dom(g).
LinearFunction operator*(const LinearFunction& f, const LinearFunction& g);

// Returns: h(x) = f(x)+a.
LinearFunction operator+(const LinearFunction& f, double a);

// Returns: h(x) = f(x)*a.
LinearFunction operator*(const LinearFunction& f, double a);
} // namespace goc
