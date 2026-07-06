#pragma once

#include <vector>
#include <iostream>

#include "goc/lib/json.hpp"
#include "goc/math/interval.h"
#include "goc/print/printable.h"

namespace goc 
{
typedef unsigned int IntervalIndex; // Index of an interval in a partitioned interval.

// This class represent a closed partitioned interval [left, ..., right].
// The interval is subdivided by a sorted vector of breakpoints that represent
// a sorted union of successive intervals.
// Invariant: left = breakpoints[0] < breakpoints[1] < … < breakpoints.back() = right.
// Note that if the interval is a point, left = right = breakpoints[0].
class PartitionedInterval : public Printable 
{
public:
    // Default constructor: creates an empty partitioned interval.
    PartitionedInterval();

    // Constructs a PartitionedInterval with a sorted vector of breakpoints.
    PartitionedInterval(const std::vector<double>& breakpoints);

    // Constructs a PartitionedInterval with a single interval [left, right].
    PartitionedInterval(const Interval& interval);

    // Constructs a PartitionedInterval with a sorted contiguous vector of intervals.
    PartitionedInterval(const std::vector<Interval>& intervals);

    // Adds a breakpoint to the partitioned interval.
    // Keeps the sorted breakpoints invariant.
    void add(double breakpoint);

    // Adds an interval to the partitioned interval.
    // Keeps the sorted breakpoints invariant.
    void add(const Interval& interval);

    bool empty() const;

    // Returns: if the domain is [a]
    bool is_point() const;

    // Returns the overall interval as an Interval.
    Interval bound() const;

    /**
     * @brief Returns the index of the interval in which 'value' belongs.
     * @throws std::out_of_range if 'value' is not inside the domain.
     */
    [[nodiscard]]
    size_t interval_index_or_throw(double value) const;

    /**
     * @brief Tries to find the index of the interval in which 'value' belongs.
     * @return std::nullopt if 'value' is not in the domain.
     */
    [[nodiscard]]
    std::optional<size_t> find_interval_index(double value) const;

    // Returns the interval segment in which 'value' belongs.
    // If the value is not in the domain, the empty interval is returned.
    // Note: if 'value' equals a partition breakpoint, the next interval is returned,
    // or [breakpoint, breakpoint] if it is the last one.
    Interval find_interval(double value) const;

    // Returns the interval segment at specified index.
    // If the index is out of bounds, the empty interval is returned.
    Interval get_interval(int index) const;

    // Prints the PartitionedInterval.
    // Format: [left, breakpoints[1], ..., right].
    virtual void Print(std::ostream& os) const;

    // Getters for the overall bounds and the breakpoints.
    double left() const;
    double right() const;
    const std::vector<double>& get_breakpoints() const;
    
    // Returns the number of intervals in the partitioned interval.
    // If point interval, returns 0. If empty, returns 0.
    inline size_t nb_intervals() const { return empty() ? 0 : breakpoints_.size() - 1; }

private:
    // Stores the breakpoints in strictly increasing order.
    // They subdivide the interval [left, right] into segments:
    // left = breakpoints[0] < breakpoints[1] < … < breakpoints.back() = right.
    std::vector<double> breakpoints_;
};

// JSON format: [left, breakpoints[1], ..., right].
void from_json(const nlohmann::json& j, PartitionedInterval& i);

void to_json(nlohmann::json& j, const PartitionedInterval& i);
} // namespace goc

// Adding to namespace std to not conflict with overload.
namespace std
{
// Returns: pi.left
inline double min(const goc::PartitionedInterval& p) { return p.left(); }

// Returns: pi.right
inline double max(const goc::PartitionedInterval& p) { return p.right(); }
} // namespace std