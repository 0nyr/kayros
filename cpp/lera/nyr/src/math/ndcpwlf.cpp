#include "nyr/math/ndcpwlf.h"
#include "nyr/math/interval.h"

#include <stdexcept>
#include <limits>
#include <algorithm>
#include <vector>

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace nyr {

nyr::NDCPWLF nyr::NDCPWLF::make_identity(goc::Interval domain) {
    if (domain.Empty()) {
        return NDCPWLF(); // Return empty function
    }
    
    std::vector<double> xs = {domain.left, domain.right};
    std::vector<double> ys = {domain.left, domain.right};
    return NDCPWLF(xs, ys);
}

nyr::NDCPWLF::NDCPWLF(
    const std::vector<double> xs,
    const std::vector<double> ys
) : xs(std::move(xs)), ys(std::move(ys)) {
    #ifndef NDEBUG
    // Check both check_invariant and check_normalization
    if (!check_normalization()) {
        throw std::runtime_error("Normalization check failed after construction.");
    }
    #endif
}

// Check if function is empty
bool nyr::NDCPWLF::empty() const {
    return xs.empty();
}

// Number of pieces = xs - 1
size_t nyr::NDCPWLF::nb_pieces() const {
    return empty() ? 0 : xs.size() - 1;
}

double nyr::NDCPWLF::evaluate(double x) const {
    if (empty()) {
        throw std::out_of_range("Cannot evaluate empty NDCPWLF");
    }
    if (!interval_vector_includes(xs, x)) {
        throw std::out_of_range("x is outside NDCPWLF domain");
    }

    // Left-continuous at value jumps: f(x) = lim_{t->x^-} f(t). At the abscissa
    // of an up-step (duplicate x) the function takes the LOWER (pre-jump) value,
    // i.e. the FIRST y stored at that x. This matches the canonical checker's
    // convention (its lower_bound returns the first y on an exact hit) and is
    // the semantics every downstream primitive (Inverse, Max, domination)
    // relies on. lower_bound returns the first breakpoint >= x, so an exact hit
    // lands on the first occurrence of the abscissa = the lower step value.
    auto it = std::lower_bound(xs.begin(), xs.end(), x);
    if (it == xs.begin()) {
        return ys.front();  // x at (or dust below) the domain start
    }
    if (it == xs.end()) {
        return ys.back();  // x at (or dust above) the domain end
    }
    size_t idx = static_cast<size_t>(std::distance(xs.begin(), it));
    if (goc::epsilon_equal(*it, x)) {
        return ys[idx];  // exact breakpoint hit -> first occurrence -> lower value
    }

    // Interior of the piece [idx-1, idx]. Since *it > x (not epsilon-equal) and
    // everything before is < x, we have xs[idx-1] < x < xs[idx]: a genuine
    // non-degenerate sloped/flat piece, never a vertical step.
    double x_left = xs[idx - 1], x_right = xs[idx];
    double y_left = ys[idx - 1], y_right = ys[idx];
    double t = (x - x_left) / (x_right - x_left);
    return y_left + t * (y_right - y_left);
}

double nyr::NDCPWLF::operator()(double x) const {
    return evaluate(x);
}

nyr::NDCPWLF nyr::NDCPWLF::inverse() const {
    if (empty()) {
        return NDCPWLF();
    }
    // A non-decreasing PWL stores both xs and ys non-decreasing, so the graph
    // reflected across y = x is again a valid non-decreasing PWL: f^{-1} is the
    // exact coordinate swap. Sloped pieces invert to sloped pieces; a value jump
    // (step: duplicate x) inverts to a plateau (duplicate y) and a plateau
    // inverts to a step. Collinearity is preserved by the reflection, so a
    // normalized f yields a normalized f^{-1} with no extra work and, crucially,
    // no numerical error: this is the operation the 1e-3 mollifier existed to
    // avoid on goc::PWLFunction.
    return NDCPWLF(ys, xs);
}

bool nyr::NDCPWLF::check_invariant() const {
    if (xs.size() != ys.size()) return false;
    if (xs.empty()) return true; // An empty function is valid

    // Check xs and ys are non-decreasing
    for (size_t i = 1; i < xs.size(); ++i) {
        if (goc::epsilon_smaller(xs[i], xs[i-1]) || goc::epsilon_smaller(ys[i], ys[i-1])) {
            std::cerr << "Invariant check failed: point ordering is violated at index " << i
                      << " with xs[" << i-1 << "] = " << xs[i-1] << ", xs[" << i << "] = " << xs[i]
                      << ", ys[" << i-1 << "] = " << ys[i-1] << ", ys[" << i << "] = " << ys[i] << std::endl;
            Print(std::cerr);
            return false; // xs or ys are not non-decreasing
        }
    }

    return true;
}

// Checks if the function is normalized.
// In this context, a normalized NDCPWLF has no intermediate points.
// Handles steps or flat segments which are allowed.
bool nyr::NDCPWLF::check_normalization() const {
    if (xs.size() < 2) {
        return true; // A single point or empty function is normalized
    }

    // Check that xs and ys are non-decreasing.
    if (check_invariant() == false) {
        return false;
    }

    // Check that no piece is the continuation of the previous piece.
    for (size_t i = 1; i < xs.size() - 1; ++i) {
        double x_prev = xs[i-1];
        double y_prev = ys[i-1];
        double x_curr = xs[i];
        double y_curr = ys[i];
        double x_next = xs[i+1];
        double y_next = ys[i+1];

        // Denominators can definitly be zero
        double slope1 =(y_curr - y_prev) / (x_curr - x_prev);
        double slope2 = (y_next - y_curr) / (x_next - x_curr);
        
        // Edge case: comparing two slopes close to 0 is tricky.
        if (slope1 < goc::EPS_SLOPE_ZERO) {
            slope1 = 0.0; // Handle horizontal segments
        }
        if (slope2 < goc::EPS_SLOPE_ZERO) {
            slope2 = 0.0; // Handle horizontal segments
        }

        // If the slopes are not different, the current breakpoint is not necessary.
        if (
            goc::epsilon_equal(slope1, slope2)
        ) {
            std::cerr << "Breakpoints are not normalized: "
                      << "slope1 = " << slope1 << ", slope2 = " << slope2
                      << " slope1 ?= slope2: " << (goc::epsilon_equal(slope1, slope2) ? "true" : "false")
                      << " slope diff: " << std::abs(slope1 - slope2)
                      << " at index " << i
                      << " (x_prev, y_prev) = (" << x_prev << ", " << y_prev << "), "
                      << "(x_curr, y_curr) = (" << x_curr << ", " << y_curr << "), "
                      << "(x_next, y_next) = (" << x_next << ", " << y_next << ")"
                      << std::endl;
            Print(std::cerr);
            return false; // Found a redundant point
        }
    }
    return true; // All checks passed, the function is normalized
}

// NOTE: See my notes, Lab Notebook X104272, p98-99.
// Time complexity: O(log_2(#pieces_of_g)*#pieces_of_f + log_2(#pieces_of_f)*#pieces_of_g) = O(2*p*log_2(p))
nyr::NDCPWLF nyr::NDCPWLF::compose_alternative(const nyr::NDCPWLF& g) const {
    const auto& f = *this;

    // 1. Pre-check
    // If either function is empty, or if the image of g does not intersect the domain
    // of f, the composition is an empty function.
    if (nyr::interval_vector_disjoints(f.xs, g.ys)) {
        #ifndef NDEBUG
        std::cerr << "Composition pre-check failed: "
                  << "f.empty() = " << (f.empty() ? "true" : "false") << ", "
                  << "g.empty() = " << (g.empty() ? "true" : "false") << ", "
                  << "does g.image intersect f.domain: "
                  << (nyr::interval_vector_intersects(f.xs, g.ys) ? "true" : "false")
                  << std::endl;
        #endif
        return nyr::NDCPWLF(); // Return empty function
    }

    // 2. Prepare fog
    std::vector<double> fog_xs;
    std::vector<double> fog_ys;
    // By theorem, fog has at most f.xs_.size() * g.ys_.size() - 1 points.
    // So reserve space for the worst case, and shrink later.
    fog_xs.reserve(f.xs.size() * g.ys.size());
    fog_ys.reserve(f.xs.size() * g.ys.size());

    // Steps 3 and 4: Find all points P'(x', y') in fog.
    // NOTE: Since fog is a NDCPWLF, we can add x' and y' in any order
    // then sort fog_xs and fog_ys at the end.

    // 3. Iterate over f points: (backward: y' given, need to compute x')
    // NOTE: f has generally more points than g, so we iterate over f first.
    // Each point P(x, y) in f such that x \in g.image adds (a maximum of) a point to fog
    // Edge case 1: If x corresponds to a step (vertical segment) on g, then this point has already been added in the previous step (continue)
    // Edge case 2: If x corresponds precisely to a point on g (i.e. x is in g.ys_), then these points have already been added in the previous step (continue)
    double slope_g_piece =  0.0; 
    double intercept_g_piece = 0.0;
    size_t cached_j = f.xs.size(); // Avoid start at 0
    
    for (size_t i = 0; i < f.xs.size(); ++i)
    {
        double y_prime = f.ys[i];
        double x = f.xs[i];
        if (!interval_vector_includes(g.ys, x)) continue; // If x is not in g's image, skip

        // Find the piece of g that contains x, using binary search.
        // Here, it consists in finding the first index j such that g.ys_[j] <= x <= g.ys_[j+1]
        auto it = std::lower_bound(g.ys.begin(), g.ys.end(), x);
        #ifndef NDEBUG
        if (it == g.ys.begin() || it == g.ys.end()) // Should never happen due to image check
            throw std::out_of_range("NDCPWLF::evaluate: x is out of range for g.ys_. This should not happen due to pre-check.");
        #endif
        size_t j = (it - g.ys.begin()) - 1;

        // If x \in g.xs_, then this point has already been added in the previous step, skip
        // NOTE: Also prevent the case where the g piece is horizontal, i.e. g.xs_[j] == g.xs_[j + 1]
        double g_x0 = g.xs[j];
        double g_y0 = g.ys[j];
        double g_x1 = g.xs[j + 1];
        double g_y1 = g.ys[j + 1];
        if (j != cached_j) {
            slope_g_piece = compute_slope(g_x0, g_y0, g_x1, g_y1);
            intercept_g_piece = compute_intercept(g_x0, g_y0, slope_g_piece);
            cached_j = j; // Cache the index for the next iteration
        }

        // #ifndef NDEBUG
        // std::clog << "NDCPWLF::compose: f o g (BACKWARD)"
        //     << " for f point (" << x << ", " << y_prime << "),"
        //     << " Evaluating piece " << j << " of g: (" 
        //    << g_x0 << ", " << g_y0 << ") to (" << g_x1 << ", " << g_y1 
        //    << "), slope: " << slope_g_piece 
        //    << std::endl;
        // #endif

        // If x exactly matches a point on g (i.e. x is in g.ys_)
        // or if the matching g piece is horizontal, skip
        if (goc::epsilon_equal(g_y0, x)
            || goc::epsilon_equal(g_y1, x)
            || goc::epsilon_equal(slope_g_piece, 0.0)
        ) {
            continue;
        }

        // If the g piece is vertical, it has already been added in the previous step, skip
        if (goc::epsilon_bigger_equal(slope_g_piece, goc::INFTY)
        ) {
            continue;
        }

        #ifndef NDEBUG
        if (goc::epsilon_equal(slope_g_piece, 0.0))
                throw std::runtime_error("NDCPWLF::compose: Slope of piece is zero, which should have already been handled by the previous cases.");
        #endif

        // If the piece is not vertical, compute the x_prime value corresponding to y
        // compute x_prime such that fog(x_prime) = y_prime
        double x_prime = (x - intercept_g_piece) / slope_g_piece;
        // Add the point to fog
        fog_xs.push_back(x_prime);
        fog_ys.push_back(y_prime);
    }

    // 4. FORWARD STEP: Iterate over g points (forward: x' given, need to compute y')
    // Each point P(x, y) in g such that y \in f.domain adds (at least) a point to fog
    // Edge case 1: If f(y) corresponds to a step (vertical segment) on f, add 2 points to fog corresponding to the segment endpoints.
    // Edge case 2: If f(y) corresponds to a flat (horizontal segment) on f, might need to remove the last added point if the one before also has same y value
    double slope_f_piece =  0.0; 
    double intercept_f_piece = 0.0;
    cached_j = f.xs.size(); // Avoid start at 0
    
    for (size_t i = 0; i < g.ys.size(); ++i)
    {
        double x_prime = g.xs[i];
        double y = g.ys[i];
        if (!interval_vector_includes(f.xs, y)) continue; // If y is not in f's domain, skip

        // Find the piece of f that contains y, using binary search.
        // Here, it consists in finding the first index j such that f.xs_[j] <= y <= f.xs_[j+1
        auto it = std::lower_bound(f.xs.begin(), f.xs.end(), y);
        #ifndef NDEBUG
        if (it == f.xs.begin() || it == f.xs.end()) // Should never happen due to domain check
            throw std::out_of_range("NDCPWLF::evaluate: y is out of range for f.xs_. This should not happen due to pre-check.");
        #endif
        size_t j = (it - f.xs.begin()) - 1;

        double f_x0 = f.xs[j];
        double f_x1 = f.xs[j + 1];
        double f_y0 = f.ys[j];
        double f_y1 = f.ys[j + 1];
        if (j != cached_j) {
            // If the piece has changed, compute the slope and intercept
            slope_f_piece = (f_y1 - f_y0) / (f_x1 - f_x0);
            intercept_f_piece = f_y0 - slope_f_piece * f_x0;
            cached_j = j; // Update cached index
        }

        #ifndef NDEBUG
        std::clog << "NDCPWLF::compose: f o g (FORWARD)"
            << " for g point (" << x_prime << ", " << y << "),"
            << " Evaluating piece " << j << " of f: ("
            << f_x0 << ", " << f_y0 << ") -> ("
            << f_x1 << ", " << f_y1 << ") with slope "
            << slope_f_piece
            << std::endl;
        #endif

        if (// Is the matching f piece vertical?
            goc::epsilon_bigger_equal(slope_f_piece, goc::INFTY)
        ) {
            // If the piece is vertical, add both endpoints
            // if it is the first time we encounter this piece,
            // else only the second endpoint.
            if (fog_xs.size() > 0 && 
                goc::epsilon_equal(fog_ys[fog_ys.size() - 1], f_y1)
            ) {
                fog_xs.push_back(x_prime);
                fog_ys.push_back(f_y1);
            } else {
                fog_xs.push_back(x_prime);
                fog_ys.push_back(f_y0);
                fog_xs.push_back(x_prime);
                fog_ys.push_back(f_y1);
            }
        } else if (
            // Is the matching f piece horizontal?
            goc::epsilon_equal(slope_f_piece, 0.0)
        ) {
            // If the piece is horizontal, remove the last added point if it is an intermediate (duplicate) point
            double y_prime = f_y0; 
            if (fog_ys.size() > 1 &&
                goc::epsilon_equal(fog_ys[fog_ys.size() - 1], fog_ys[fog_ys.size() - 2]) && 
                goc::epsilon_equal(fog_ys[fog_ys.size() - 1], y_prime)
            ) {
                // Remove the last added point
                fog_xs.pop_back();
                fog_ys.pop_back();
                #ifndef NDEBUG
                std::clog << "NDCPWLF::compose: Removing duplicate point (" 
                      << x_prime << ", " << y_prime << ") from fog." << std::endl;
                #endif
            }
            fog_xs.push_back(x_prime);
            fog_ys.push_back(y_prime);
        } else {
            // If the piece is not vertical or horizontal
            // compute the y value corresponding to x
            // fog(x) = f(y) = f_y0 + (f_y1 - f_y0) * (y - f_x0) / (f_x1 - f_x0)

            #ifndef NDEBUG
            if (goc::epsilon_equal(slope_f_piece, 0.0))
                throw std::runtime_error("NDCPWLF::compose: Slope of piece is zero, which should have already been handled by the previous cases.");
            #endif

            double y_prime = slope_f_piece * y + intercept_f_piece;
            fog_xs.push_back(x_prime);
            fog_ys.push_back(y_prime);
        }
    }

    // 4. Shrink vectors to fit the actual number of points 
    // At this step, we should have added points to fog in the wrong order
    // but since fog is non-decreasing, we can sort both vectors
    fog_xs.shrink_to_fit();
    fog_ys.shrink_to_fit();
    if (fog_xs.empty()) {
        return nyr::NDCPWLF(); 
    }

    std::sort(fog_xs.begin(), fog_xs.end());
    std::sort(fog_ys.begin(), fog_ys.end());

    std::vector<double> fog_xs_normalized;
    std::vector<double> fog_ys_normalized;
    fog_xs_normalized.reserve(fog_xs.size());
    fog_ys_normalized.reserve(fog_ys.size());
    
    fog_xs_normalized.push_back(fog_xs[0]);
    fog_ys_normalized.push_back(fog_ys[0]);

    // std::clog << "NDCPWLF::compose: Nb points in fog before normalization: "
    //        << fog_xs.size() << std::endl;

    // 5. Remove intermediate points
    // Check that no piece is the continuation of the previous piece.
    for (size_t i = 1; i < fog_xs.size() - 1; ++i) {
        double x_prev = fog_xs_normalized.back();
        double y_prev = fog_ys_normalized.back();
        double x_curr = fog_xs[i];
        double y_curr = fog_ys[i];
        double x_next = fog_xs[i+1];
        double y_next = fog_ys[i+1];

        // Denominators can definitly be zero
        double slope1 = compute_slope(x_prev, y_prev, x_curr, y_curr);
        double slope2 = compute_slope(x_curr, y_curr, x_next, y_next);

        // Only add the current point if the slopes are different
        if (goc::epsilon_different(slope1, slope2)) {
            fog_xs_normalized.push_back(x_curr);
            if (slope1 != 0.0) {
                fog_ys_normalized.push_back(y_curr);
            } else {
                fog_ys_normalized.push_back(y_prev); // If horizontal, keep the previous y value
            }
        } else {
            #ifndef NDEBUG
            std::clog << "NDCPWLF::compose: Removing intermediate point at index " << i
            << " with coordinates (" << x_curr << ", " << y_curr << ")"
            << std::endl;
            #endif
        }
    }
    fog_xs_normalized.push_back(fog_xs.back());
    fog_ys_normalized.push_back(fog_ys.back());
    fog_xs_normalized.shrink_to_fit();
    fog_ys_normalized.shrink_to_fit();

    // std::clog << "NDCPWLF::compose: Nb points in fog after normalization: "
    //  << fog_xs_normalized.size() << std::endl;

    return nyr::NDCPWLF(fog_xs_normalized, fog_ys_normalized);
}

nyr::NDCPWLF nyr::NDCPWLF::compose_visser(const nyr::NDCPWLF& g) const {
    const auto& f = *this;

    // 1. Pre-check
    // If either function is empty, or if the image of g does not intersect the domain
    // of f, the composition is an empty function.
    if (nyr::interval_vector_disjoints(f.xs, g.ys)) {
        #ifndef NDEBUG
        std::cerr << "Composition pre-check failed: "
                  << "f.empty() = " << (f.empty() ? "true" : "false") << ", "
                  << "g.empty() = " << (g.empty() ? "true" : "false") << ", "
                  << "does g.image intersect f.domain: "
                  << (nyr::interval_vector_intersects(f.xs, g.ys) ? "true" : "false")
                  << std::endl;
        #endif
        return nyr::NDCPWLF(); // Return empty function
    }

    // 2. Prepare fog
    std::vector<double> fog_xs;
    std::vector<double> fog_ys;
    // By theorem, fog has at most f.xs_.size() * g.ys_.size() - 1 points.
    // So reserve space for the worst case, and shrink later.
    const size_t nb_sum_points =  f.xs.size() + g.ys.size(); 
    fog_xs.reserve(nb_sum_points - 1);
    fog_ys.reserve(nb_sum_points - 1);

    // Steps 3 and 4: Find all points P'(x', y') in fog.
    // Following Visser et al 2020: Efficient Move Evaluations for 
    // Time-Dependent Vehicle Routing Problems, Theorem 3, p4-5.
    size_t i = 0, j = 0;
    double slope_f_piece = 0.0;
    double intercept_f_piece = 0.0;
    double slope_g_piece = 0.0;
    double intercept_g_piece = 0.0;
    if (f.xs.size() > 1) {
        // Compute slope and intercept for the first piece of f
        slope_f_piece = compute_slope(f.xs[0], f.ys[0], f.xs[1], f.ys[1]);
        intercept_f_piece = compute_intercept(f.xs[0], f.ys[0], slope_f_piece);
    }
    if (g.ys.size() > 1) {
        // Compute slope and intercept for the first piece of g
        slope_g_piece = compute_slope(g.xs[0], g.ys[0], g.xs[1], g.ys[1]);
        intercept_g_piece = compute_intercept(g.xs[0], g.ys[0], slope_g_piece);
    }

    while (i < f.xs.size() && j < g.ys.size()) {
        double g_y = g.ys[j];
        double f_x = f.xs[i];
        if (goc::epsilon_bigger(g_y, f_x)) {
            // BACKWARD STEP: From f to g
            double y_prime = f.ys[i];
            if (interval_vector_includes(g.ys, f_x)) {
                double x_prime = (f_x - intercept_g_piece) / slope_g_piece;
                // Add the point (x', y') to fog
                fog_xs.push_back(x_prime);
                fog_ys.push_back(y_prime);

                #ifndef NDEBUG
                std::clog << "NDCPWLF::compose_visser: Backward step from f to g."
                    << " i = "  << i
                    << ", j = " << j
                    << ", g_y = " << g_y
                    << ", f_x = " << f_x
                    << ", f_y = y_prime = " << y_prime
                    << ", piece of g at j = " << j 
                    << " of domain: [" << g.xs[j - 1] << ", " << g.xs[j] << "]"
                    << " and image: [" << g.ys[j - 1] << ", " << g.ys[j] << "]"
                    << ", slope_g_piece = " << slope_g_piece
                    << ", intercept_g_piece = " << intercept_g_piece
                    << ", obtained x_prime = " << x_prime
                    << ". Adding point (" << x_prime << ", " << y_prime << ")"
                    << std::endl;
                #endif
            }
            // i has changed, so we need to recompute the slope and intercept
            if (i < f.xs.size() - 1) {
                slope_f_piece = compute_slope(f.xs[i], f.ys[i], f.xs[i + 1], f.ys[i + 1]);
                intercept_f_piece = compute_intercept(f.xs[i], f.ys[i], slope_f_piece);
            }
            i += 1; // Move to next point of f
        } else if (goc::epsilon_equal(g_y, f_x)) {
            // Matching breakpoint from g to f. 
            double x_prime = g.xs[j];
            double y_prime = f.ys[i];
            // Add the point (x', y') to fog
            fog_xs.push_back(x_prime);
            fog_ys.push_back(y_prime);

            if (i < f.xs.size() - 1) {
                // Recompute the slope and intercept for the next piece of f
                slope_f_piece = compute_slope(f.xs[i], f.ys[i], f.xs[i + 1], f.ys[i + 1]);
                intercept_f_piece = compute_intercept(f.xs[i], f.ys[i], slope_f_piece);
            }
            i += 1; // Move to next point of f
            if (j < g.ys.size() - 1) {
                // Recompute the slope and intercept for the next piece of g
                slope_g_piece = compute_slope(g.xs[j], g.ys[j], g.xs[j + 1], g.ys[j + 1]);
                intercept_g_piece = compute_intercept(g.xs[j], g.ys[j], slope_g_piece);
            }
            j += 1; // Move to next point of g

            #ifndef NDEBUG
            std::clog << "NDCPWLF::compose_visser: Matching breakpoint found."
                << " Adding point (" << x_prime << ", " << y_prime << ")" 
                << std::endl;
            #endif
        } else {
            // FORWARD STEP: From g to f
            if (interval_vector_includes(f.xs, g_y)) {
                double x_prime = g.xs[j];
                // Get the piece of f that contains g_y
                // This is the last piece i
                double y_prime = slope_f_piece * g_y + intercept_f_piece;
                // Add the point (x', y') to fog
                fog_xs.push_back(x_prime);
                fog_ys.push_back(y_prime);

                #ifndef NDEBUG
                std::clog << "NDCPWLF::compose_visser: Forward step from g to f."
                    << " i = "  << i 
                    << ", j = " << j 
                    << ", g_y = " << g_y 
                    << ", f_x = " << f_x
                    << ", piece of f at i = " << i << " (f.xs[i] = " << f.ys[i] <<")"
                    << ", slope_f_piece = " << slope_f_piece
                    << ", intercept_f_piece = " << intercept_f_piece    
                    << ". Adding point (" << x_prime << ", " << y_prime << ")"
                    << std::endl;
                #endif
            }
            // j has changed, so we need to recompute the slope and intercept
            if (j < g.ys.size()) {
                slope_g_piece = compute_slope(g.xs[j], g.ys[j], g.xs[j + 1], g.ys[j + 1]);
                intercept_g_piece = compute_intercept(g.xs[j], g.ys[j], slope_g_piece);
            }
            j += 1; // Move to next point of g
        }
    }

    // Tail flush — exact boundary ties at the exhausted operand's last breakpoint.
    // The merge exits as soon as one operand is exhausted, but trailing points of
    // the other operand still belong to fog when they tie with the exhausted
    // operand's last breakpoint: g flat at max(dom f) (deadline-riding routes:
    // upstream waiting clamps make g flat exactly at a TW deadline), or f with a
    // vertical step at max(img g). Dropping them collapses the composed domain
    // and loses the optimal (latest) departure time.
    if (!fog_xs.empty()) {
        if (i == f.xs.size()) {
            for (; j < g.ys.size(); ++j) {
                if (goc::epsilon_equal(g.ys[j], f.xs.back())) {
                    fog_xs.push_back(g.xs[j]);
                    fog_ys.push_back(f.ys.back());
                }
            }
        } else if (j == g.ys.size()) {
            for (; i < f.xs.size(); ++i) {
                if (goc::epsilon_equal(f.xs[i], g.ys.back())) {
                    fog_xs.push_back(g.xs.back());
                    fog_ys.push_back(f.ys[i]);
                }
            }
        }
    }

    fog_xs.shrink_to_fit();
    fog_ys.shrink_to_fit();

    return nyr::NDCPWLF(fog_xs, fog_ys);
}

// visser+normalization version
nyr::NDCPWLF nyr::NDCPWLF::compose(const nyr::NDCPWLF& g) const {
    const auto& f = *this;

    // 1. Pre-check
    // If either function is empty, or if the image of g does not intersect the domain
    // of f, the composition is an empty function.
    if (nyr::interval_vector_disjoints(f.xs, g.ys)) {
        #ifndef NDEBUG
        std::cerr << "Composition pre-check failed: "
                  << "f.empty() = " << (f.empty() ? "true" : "false") << ", "
                  << "g.empty() = " << (g.empty() ? "true" : "false") << ", "
                  << "does g.image intersect f.domain: "
                  << (nyr::interval_vector_intersects(f.xs, g.ys) ? "true" : "false")
                  << std::endl;
        #endif
        return nyr::NDCPWLF(); // Return empty function
    }

    // 2. Prepare fog
    std::vector<double> fog_xs;
    std::vector<double> fog_ys;
    // By theorem, fog has at most f.xs_.size() * g.ys_.size() - 1 points.
    // So reserve space for the worst case, and shrink later.
    const size_t nb_sum_points =  f.xs.size() + g.ys.size(); 
    fog_xs.reserve(nb_sum_points - 1);
    fog_ys.reserve(nb_sum_points - 1);

    // 3. Find all points P'(x', y') in fog.
    // Following Visser et al 2020: Efficient Move Evaluations for 
    // Time-Dependent Vehicle Routing Problems, Theorem 3, p4-5.
    size_t i = 0, j = 0;
    double slope_f_piece = 0.0;
    double intercept_f_piece = 0.0;
    double slope_g_piece = 0.0;
    double intercept_g_piece = 0.0;
    if (f.xs.size() > 1) {
        // Compute slope and intercept for the first piece of f
        slope_f_piece = compute_slope(f.xs[0], f.ys[0], f.xs[1], f.ys[1]);
        intercept_f_piece = compute_intercept(f.xs[0], f.ys[0], slope_f_piece);
    }
    if (g.ys.size() > 1) {
        // Compute slope and intercept for the first piece of g
        slope_g_piece = compute_slope(g.xs[0], g.ys[0], g.xs[1], g.ys[1]);
        intercept_g_piece = compute_intercept(g.xs[0], g.ys[0], slope_g_piece);
    }

    while (i < f.xs.size() && j < g.ys.size()) {
        double g_y = g.ys[j];
        double f_x = f.xs[i];
        if (goc::epsilon_bigger(g_y, f_x)) {
            // BACKWARD STEP: From f to g
            double y_prime = f.ys[i];
            if (interval_vector_includes(g.ys, f_x)) {
                double x_prime = (f_x - intercept_g_piece) / slope_g_piece;
                normalized_add(fog_xs, fog_ys, x_prime, y_prime);

                #ifndef NDEBUG
                std::clog << "NDCPWLF::compose_visser: Backward step from f to g."
                    << " i = "  << i
                    << ", j = " << j
                    << ", g_y = " << g_y
                    << ", f_x = " << f_x
                    << ", f_y = y_prime = " << y_prime
                    << ", piece of g at j = " << j 
                    << " of domain: [" << g.xs[j - 1] << ", " << g.xs[j] << "]"
                    << " and image: [" << g.ys[j - 1] << ", " << g.ys[j] << "]"
                    << ", slope_g_piece = " << slope_g_piece
                    << ", intercept_g_piece = " << intercept_g_piece
                    << ", obtained x_prime = " << x_prime
                    << ". Adding point (" << x_prime << ", " << y_prime << ")"
                    << std::endl;
                #endif
            }
            // i has changed, so we need to recompute the slope and intercept
            if (i < f.xs.size() - 1) {
                slope_f_piece = compute_slope(f.xs[i], f.ys[i], f.xs[i + 1], f.ys[i + 1]);
                intercept_f_piece = compute_intercept(f.xs[i], f.ys[i], slope_f_piece);
            }
            i += 1; // Move to next point of f
        } else if (goc::epsilon_equal(g_y, f_x)) {
            // Matching breakpoint from g to f. 
            double x_prime = g.xs[j];
            double y_prime = f.ys[i];
            normalized_add(fog_xs, fog_ys, x_prime, y_prime);

            if (i < f.xs.size() - 1) {
                // Recompute the slope and intercept for the next piece of f
                slope_f_piece = compute_slope(f.xs[i], f.ys[i], f.xs[i + 1], f.ys[i + 1]);
                intercept_f_piece = compute_intercept(f.xs[i], f.ys[i], slope_f_piece);
            }
            i += 1; // Move to next point of f
            if (j < g.ys.size() - 1) {
                // Recompute the slope and intercept for the next piece of g
                slope_g_piece = compute_slope(g.xs[j], g.ys[j], g.xs[j + 1], g.ys[j + 1]);
                intercept_g_piece = compute_intercept(g.xs[j], g.ys[j], slope_g_piece);
            }
            j += 1; // Move to next point of g

            #ifndef NDEBUG
            std::clog << "NDCPWLF::compose_visser: Matching breakpoint found."
                << " Adding point (" << x_prime << ", " << y_prime << ")" 
                << std::endl;
            #endif
        } else {
            // FORWARD STEP: From g to f
            if (interval_vector_includes(f.xs, g_y)) {
                double x_prime = g.xs[j];
                // Get the piece of f that contains g_y
                // This is the last piece i
                double y_prime = slope_f_piece * g_y + intercept_f_piece;
                // Add the point (x', y') to fog
                normalized_add(fog_xs, fog_ys, x_prime, y_prime);

                #ifndef NDEBUG
                std::clog << "NDCPWLF::compose_visser: Forward step from g to f."
                    << " i = "  << i 
                    << ", j = " << j 
                    << ", g_y = " << g_y 
                    << ", f_x = " << f_x
                    << ", piece of f at i = " << i << " (f.xs[i] = " << f.ys[i] <<")"
                    << ", slope_f_piece = " << slope_f_piece
                    << ", intercept_f_piece = " << intercept_f_piece    
                    << ". Adding point (" << x_prime << ", " << y_prime << ")"
                    << std::endl;
                #endif
            }
            // j has changed, so we need to recompute the slope and intercept
            if (j < g.ys.size()) {
                slope_g_piece = compute_slope(g.xs[j], g.ys[j], g.xs[j + 1], g.ys[j + 1]);
                intercept_g_piece = compute_intercept(g.xs[j], g.ys[j], slope_g_piece);
            }
            j += 1; // Move to next point of g
        }
    }

    // Tail flush — exact boundary ties at the exhausted operand's last breakpoint
    // (same rationale as in compose_visser above: g flat at max(dom f), or f with
    // a vertical step at max(img g); without this the composed domain collapses).
    if (!fog_xs.empty()) {
        if (i == f.xs.size()) {
            for (; j < g.ys.size(); ++j) {
                if (goc::epsilon_equal(g.ys[j], f.xs.back())) {
                    normalized_add(fog_xs, fog_ys, g.xs[j], f.ys.back());
                }
            }
        } else if (j == g.ys.size()) {
            for (; i < f.xs.size(); ++i) {
                if (goc::epsilon_equal(f.xs[i], g.ys.back())) {
                    normalized_add(fog_xs, fog_ys, g.xs.back(), f.ys[i]);
                }
            }
        }
    }

    return nyr::NDCPWLF(fog_xs, fog_ys);
}

goc::PWLFunction nyr::NDCPWLF::to_goc_pwl_function() const {
    goc::PWLFunction pwlf;
    if (empty()) {
        return pwlf; // Return empty PWLFunction
    }

    // Convert xs and ys to goc::PWLFunction
    for (size_t i = 0; i < xs.size() - 1; ++i) {
        pwlf.AddPiece(goc::LinearFunction(
            goc::Point2D(xs[i], ys[i]),
            goc::Point2D(xs[i + 1], ys[i + 1])
        ));
    }

    return pwlf;
}

void nyr::NDCPWLF::Print(std::ostream& os) const {
    os << "NDCPWLF Domain: " << get_domain() 
       << ", Image: " << get_image() << std::endl;
    goc::print_padded_vectors(os, xs, ys);
}

bool nyr::NDCPWLF::operator==(const NDCPWLF& other) const {
    // Check if xs and ys are the same
    return (
        (xs == other.xs) && 
        (ys == other.ys)
    );
}

std::size_t nyr::NDCPWLF::memory_footprint_bytes() const {
    // sizeof(*this) covers: 
    //   - domain_, image_, the two vector headers (3 pointers each on most platforms), etc.
    // the following adds the heap‐allocated storage for both vectors:
    return sizeof(*this)
         + xs.capacity() * sizeof(double)
         + ys.capacity() * sizeof(double);
}

double NDCPWLF::compute_area() const {
    // If the function has no pieces (i.e., it's empty), the area under it is 0.
    if (empty()) {
        return 0.0;
    }

    double total_area = 0.0; // Initialize the accumulated area.

    // Iterate through each linear piece (segment) of the function.
    // A function with 'N' breakpoints has 'N-1' pieces.
    // The 'nb_pieces()' method returns xs.size() - 1 when not empty.
    for (size_t i = 0; i < nb_pieces(); ++i) {
        // Get the coordinates of the start point of the current piece.
        const double x1 = xs[i];
        const double y1 = ys[i];

        // Get the coordinates of the end point of the current piece.
        const double x2 = xs[i+1];
        const double y2 = ys[i+1];

        const double slope = compute_slope(x1, y1, x2, y2);
        if (// Is the matching f piece vertical?
            goc::epsilon_bigger_equal(slope, goc::INFTY)
        ) {
            continue; // A vertical segment has 0 area under it.
        }

        // Calculate the area of the trapezoid formed by the current piece and the x-axis.
        // The formula for the area of a trapezoid is ((base1 + base2) / 2) * height.
        // In this context:
        // - base1 is y1 (the y-value at the start of the segment)
        // - base2 is y2 (the y-value at the end of the segment)
        // - height is (x2 - x1) (the width of the segment along the x-axis)
        total_area += ((y1 + y2) * (x2 - x1)) / 2.0;
    }

    return total_area; // Return the accumulated total area.
}

} // namespace nyr