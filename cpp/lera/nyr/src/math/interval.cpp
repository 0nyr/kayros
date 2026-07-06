#include "nyr/math/interval.h"

namespace nyr {

void test_interval_vector_intersects() {
    // Test cases for interval_vector_intersects
    std::vector<double> vec1 = {1.0, 2.0, 3.0};
    std::vector<double> vec2 = {2.5, 3.5, 4.5};
    std::vector<double> vec3 = {0.5, 1.5, 2.5};

    // Should return true: intervals [1.0, 3.0] and [2.5, 4.5] intersect
    assert(interval_vector_intersects(vec1, vec2) == true);
    // Should return false: intervals [1.0, 3.0] and [4.5, 6.5] do not intersect
    assert(interval_vector_intersects(vec1, {4.5, 6.5}) == false);
    // Should return true: intervals [0.5, 2.5] and [1.0, 3.0] intersect
    assert(interval_vector_intersects(vec3, vec1) == true);
    // Should return false: empty vector should not intersect with any other vector
    assert(interval_vector_intersects({}, vec1) == false);
    // Point case: [1.0, 3.0] and [3.0, 4.0] intersect at 3.0
    assert(interval_vector_intersects(vec1, {3.0, 4.0}) == true);

    #ifndef NDEBUG
    std::clog << "All interval_vector_intersects tests passed!" << std::endl;
    #endif
}

void test_interval_vector_includes() {
    // Test cases for interval_vector_includes
    std::vector<double> vec1 = {1.0, 2.0, 3.0};
    std::vector<double> vec2 = {2.5, 3.5, 4.5};
    std::vector<double> vec3 = {0.5, 1.5, 2.5};

    // Should return true: 2.0 is in [1.0, 3.0]
    assert(interval_vector_includes(vec1, 2.5) == true);
    // Should return false: 4.0 is not in [1.0, 3.0]
    assert(interval_vector_includes(vec1, 4.0) == false);
    // Should return true: 1.5 is in [0.5, 2.5]
    assert(interval_vector_includes(vec3, 1.5) == true);
    // Should return false: empty vector should not include any value
    assert(interval_vector_includes({}, 1.0) == false);
    // Point case: should return true since 3.0 is in [1.0, 3.0]
    assert(interval_vector_includes(vec1, 3.0) == true);

    #ifndef NDEBUG
    std::clog << "All interval_vector_includes tests passed!" << std::endl;
    #endif
}


} // namespace nyr
