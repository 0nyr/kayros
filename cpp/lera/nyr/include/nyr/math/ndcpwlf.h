#pragma once

#include <vector>
#include <goc/goc.h>

namespace nyr {

/**
 * ### Non-Decreasing Continuous Piecewise Linear Function (CPWLF).
 * 
 * This class represents a non-decreasing continuous piecewise linear function.
 * It is defined by a set of breakpoints and corresponding values.
 * The function is continuous and linear between each pair of 
 * consecutive breakpoints. Note that its slope is always non-negative
 * and can be infinite between two breakpoints (step function).
 * 
 * Note that the function is stored normalized. A function is 
 * normalized iif no two consecutive pieces have the same
 * slope, intercept, and share the end and beginning of their domains.
 * 
 * Contains `p` pieces for `b = p + 1` breakpoints.
 */
class NDCPWLF : public goc::Printable {
private:
    std::vector<double> xs; // Sorted x-values
    std::vector<double> ys; // Corresponding (sorted) y-values
public:
    // NOTE: No need to store domain_ and image_ since they are obtained in O(1) from xs_ and ys_.

    // Returns: f(x)=x with the specific domain.
    static NDCPWLF make_identity(goc::Interval domain);

    // Constructor for 2D continuous piecewise linear function
    // Precondition: The list of breakpoints must be sorted and unique.
    // Precondition: The values must be non-decreasing, but not necessarily unique (step function case).
    NDCPWLF(
        const std::vector<double> xs,
        const std::vector<double> ys
    );
    NDCPWLF(std::pair<std::vector<double>, std::vector<double>> breakpoints_and_values)
        : NDCPWLF(breakpoints_and_values.first, breakpoints_and_values.second) {}
    NDCPWLF(const goc::PWLFunction& f)
        : NDCPWLF(f.copy_breakpoints_and_values()) {}

    // Return empty
    NDCPWLF() = default;

    // Returns true if the function is empty (no pieces).
    bool empty() const;

    // Returns the number of pieces in the function.
    size_t nb_pieces() const;

    // O(log n) evaluation using binary search and linear interpolation
    // Throws std::out_of_range if x is outside the domain.
    // Precondition: x \in dom(f)
    // Note that if x is the domain value of a step, it returns the smallest value of the step.
    double evaluate(double x) const;
    
    // Returns: the evaluation of the piece that includes x in its domain.
    // Exception: if no piece includes x in its domain, it throws an exception.
    // Note that if x is the domain value of a step, it returns the smallest value of the step.
    double operator()(double x) const;

    // Checks if the function is well-formed, i.e. that
    // its xs and ys are non-decreasing.
    // NOTE: There can be duplicate xs and ys.
    bool check_invariant() const;

    // Checks if the function is normalized.
    bool check_normalization() const;

    // Returns: the composition of this function (f) and g, i.e. fog(x) == f(g(x)).
    // Observation: the domain of the new function are those x such that g(x) \in dom(f).
    // Note that the composition of 2 NDCPWLFs is necessarily another NDCPWLF (theorem 3, Visser et al. 2020)
    NDCPWLF compose_alternative(const NDCPWLF& g) const;

    // Composition with Visser's method, no normalization.
    NDCPWLF compose_visser(const NDCPWLF& g) const;

    // Returns: the composition of this function (f) and g, i.e. fog(x) == f(g(x)).
    // Observation: the domain of the new function are those x such that g(x) \in dom(f).
    // Note that the composition of 2 NDCPWLFs is necessarily another NDCPWLF (theorem 3, Visser et al. 2020)
    // Composition with Visser's method
    // Improved with post-normalization.
    NDCPWLF compose(const NDCPWLF& g) const;

    goc::PWLFunction to_goc_pwl_function() const;

    void Print(std::ostream& os) const;

    // Getters
    inline double get_min_domain() const {
        return xs.empty() ? goc::INFTY : xs.front();
    }
    inline double get_max_domain() const {
        return xs.empty() ? -goc::INFTY : xs.back();
    }
    inline double get_min_image() const {
        return ys.empty() ? goc::INFTY : ys.front();
    }
    inline double get_max_image() const {
        return ys.empty() ? -goc::INFTY : ys.back();
    }
    inline goc::Interval get_domain() const {
        return goc::Interval(get_min_domain(), get_max_domain());
    }
    inline goc::Interval get_image() const {
        return goc::Interval(get_min_image(), get_max_image());
    }
    const std::vector<double>& get_xs() const { return xs; }
    const std::vector<double>& get_ys() const { return ys; }    
    const std::pair<std::vector<double>, std::vector<double>> copy_breakpoints_and_values() const {
        return {xs, ys};
    }

    // Returns: if the function equals another function.
    bool operator==(const NDCPWLF& other) const;

    // Returns the total memory footprint of *this* object, in bytes,
    // including both the fixed‐size portion (sizeof(*this)) and any
    // heap allocations (e.g. std::vector buffers).
    std::size_t memory_footprint_bytes() const;

    /**
     * @brief Computes the area under the non-decreasing continuous piecewise linear function.
     * * This function calculates the definite integral of the function from its
     * minimum domain value to its maximum domain value. It achieves this by
     * summing the areas of the trapezoids formed by each linear piece and the x-axis.
     * * For each segment from (x_i, y_i) to (x_{i+1}, y_{i+1}), the area of the
     * trapezoid is calculated as: (y_i + y_{i+1}) * (x_{i+1} - x_i) / 2.0.
     * * Vertical segments (where x_{i+1} - x_i == 0) correctly contribute 0 area.
     * * @return The total area under the function. Returns 0.0 if the function is empty.
     */
    double compute_area() const;
};

// Computes the slope of the segment between two points (x1, y1) and (x2, y2).
// NOTE: If the segment is vertical, it returns goc::INFTY.
// NOTE: If the segment is horizontal, it returns 0.0.
inline double compute_slope(
    const double x1, const double y1,
    const double x2, const double y2
) {
    if (goc::epsilon_equal(x2, x1)) {
        return goc::INFTY; // Handle vertical segments
    }
    double slope = (y2 - y1) / (x2 - x1);
    if (slope < goc::EPS_SLOPE_ZERO) {
        return 0.0; // Handle horizontal segments
    }
    return slope;
}

// Computes the intercept of the segment between a point (x1, y1) and a slope.
// NOTE: If the slope is 0, it returns y1 (horizontal segment).
// NOTE: If the slope is goc::INFTY, it returns goc::INFTY (vertical segment).
inline double compute_intercept(
    const double x1, const double y1,
    const double slope
) {
    if (slope < goc::EPS_SLOPE_ZERO) {
        return y1; // Handle horizontal segments
    } else if (goc::epsilon_bigger_equal(slope, goc::INFTY)) {
        return goc::INFTY; // Handle vertical segments
    }
     return y1 - slope * x1;
}

// Utility function for checking normalization.
// Before adding a point to a NDCPWLF, it checks if the last added point
// is not redundant with the new point, and removed it if it is.
// Then add the new point to the xs and ys vectors.
inline void normalized_add(
    std::vector<double>& xs,
    std::vector<double>& ys,
    double x_next, double y_next
) {
    // Guard case
    if (xs.size() < 2) {
        xs.push_back(x_next);
        ys.push_back(y_next);
        return;
    }

    size_t i = xs.size() - 1; // index of last added point
    double x_prev = xs[i - 1];
    double y_prev = ys[i - 1];
    double x_curr = xs[i];
    double y_curr = ys[i];

    double slope1 = compute_slope(x_prev, y_prev, x_curr, y_curr);
    double slope2 = compute_slope(x_curr, y_curr, x_next, y_next);

    // If the last point is redundant with the new point, remove it
    // by replacing it with the new one.
    // If both slopes are 0, reuse last y value
    if (goc::epsilon_equal(slope1, slope2)) {
        xs[i] = x_next;
        if(goc::epsilon_different(slope1, 0.0)) {
            ys[i] = y_next; // Only update y if the slope is not 0
        }
        return;
    }
    
    // Add the new additional point
    xs.push_back(x_next);
    ys.push_back(y_next);
}

} // namespace nyr