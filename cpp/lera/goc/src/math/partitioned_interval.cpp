#include <algorithm>
#include <cassert>
#include <limits>

#include "goc/math/partitioned_interval.h"
#include "goc/math/number_utils.h"  // For epsilon_equal, epsilon_smaller, etc.

namespace goc {

PartitionedInterval::PartitionedInterval(): breakpoints_()
{
    // Default constructor creates an empty partitioned interval.
}

PartitionedInterval::PartitionedInterval(
    const std::vector<double>& breakpoints
): breakpoints_(breakpoints) 
{
    // Ensure the breakpoint vector is not empty.
    assert(!breakpoints_.empty() && "Breakpoint vector must not be empty.");

    // Check strictly increasing order.
    for (size_t i = 1; i < breakpoints_.size(); ++i) 
    {
        assert(epsilon_smaller(breakpoints_[i - 1], breakpoints_[i]) && "Breakpoints must be strictly increasing.");
    }
}

PartitionedInterval::PartitionedInterval(const Interval& interval): breakpoints_()
{
    // If the interval is empty, do nothing.
    if (interval.Empty())
        return;

    // Add the left and right bounds of the interval.
    // NOTE: Since it's an interval, the bounds are included.
    breakpoints_.push_back(interval.left);
    breakpoints_.push_back(interval.right);
}

PartitionedInterval::PartitionedInterval(const std::vector<Interval>& intervals): breakpoints_()
{
    if (intervals.empty())
        return;

    // Add the left bound of first interval.
    breakpoints_.push_back(intervals.front().left);

    // Then add the right bound of each interval.
    for (const Interval& interval: intervals)
        add(interval.right);
}

void PartitionedInterval::add(double breakpoint) 
{
    // If the partitioned interval is empty, add the breakpoint.
    if (empty()) 
    {
        breakpoints_.push_back(breakpoint);
        return;
    }

    // If the breakpoint is outside the current bounds, update the bounds
    // and shift the breakpoints accordingly.
    if (epsilon_smaller(breakpoint, breakpoints_.front())) 
    {
        breakpoints_.insert(breakpoints_.begin(), breakpoint);
        return;
    }
    if (epsilon_bigger(breakpoint, breakpoints_.back())) 
    {
        breakpoints_.push_back(breakpoint);
        return;
    }

    // Insert the breakpoint while keeping the vector sorted.
    auto it = std::lower_bound(breakpoints_.begin(), breakpoints_.end(), breakpoint,
        [](double bp, double value) { return epsilon_smaller(bp, value); });
    
    // Do not insert if it already exists.
    if (it != breakpoints_.end() && epsilon_equal(*it, breakpoint))
        return;
    
    breakpoints_.insert(it, breakpoint);
}

void PartitionedInterval::add(const Interval& interval)
{
    // If the interval is empty, do nothing.
    if (interval.Empty())
        return;

    // Add the left and right bounds of the interval.
    add(interval.left);
    add(interval.right);
}

bool PartitionedInterval::empty() const 
{
    return breakpoints_.empty();
}

bool PartitionedInterval::is_point() const 
{
    // According to the invariant, a point interval has a single breakpoint.
    return !empty() && breakpoints_.size() == 1;
}

Interval PartitionedInterval::bound() const 
{
    if (empty())
        return Interval();
    return Interval(breakpoints_.front(), breakpoints_.back());
}

// Assumes breakpoints_ is sorted: [b0, b1, ..., bn]
size_t PartitionedInterval::interval_index_or_throw(double value) const
{
    if (empty())
        throw std::out_of_range("PartitionedInterval is empty.");

    if (value < breakpoints_.front() || value > breakpoints_.back())
        throw std::out_of_range("Value out of bounds of the partitioned interval.");

    auto it = std::upper_bound(breakpoints_.begin(), breakpoints_.end(), value);

    // Special case: value == breakpoints_.back()
    if (it == breakpoints_.end())
        return breakpoints_.size() - 2;

    size_t idx = std::distance(breakpoints_.begin(), it) - 1;
    return idx;
}

std::optional<size_t> PartitionedInterval::find_interval_index(double value) const
{
    if (empty() || value < breakpoints_.front() || value > breakpoints_.back())
        return std::nullopt;

    auto it = std::upper_bound(breakpoints_.begin(), breakpoints_.end(), value);

    if (it == breakpoints_.end())
        return breakpoints_.size() >= 2 ? std::optional<size_t>{breakpoints_.size() - 2} : std::nullopt;

    size_t idx = std::distance(breakpoints_.begin(), it) - 1;
    return idx;
}

Interval PartitionedInterval::find_interval(double value) const {
    // If the partitioned interval is empty, return an empty interval.
    if (empty())
        return Interval();

    Interval overall = bound();
    if (!overall.Includes(value))
        return Interval(); // Return an empty interval if value is out-of-bound.

    // Use std::upper_bound to find the first breakpoint strictly greater than value.
    auto it = std::upper_bound(breakpoints_.begin(), breakpoints_.end(), value,
        [](double val, double bp) { return epsilon_smaller(val, bp); });

    // If value is less than the first breakpoint (should not happen because of overall.Includes),
    // or if for some reason it == begin(), return an empty interval.
    if (it == breakpoints_.begin())
        return Interval();

    // If value is equal to the last breakpoint, return the point interval.
    if (it == breakpoints_.end()) 
    {
        double bp = breakpoints_.back();
        return Interval(bp, bp);
    }

    // Otherwise, the interval is from the previous breakpoint to the found breakpoint.
    auto lower = it - 1;
    return Interval(*lower, *it);
}

Interval PartitionedInterval::get_interval(int index) const 
{
    // If the partitioned interval is empty, return an empty interval.
    if (empty())
        return Interval();

    // If the index is out of bounds, return an empty interval.
    if (index < 0 || index >= (int)breakpoints_.size() - 1)
        return Interval();

    // Return the interval segment at the specified index.
    return Interval(breakpoints_[index], breakpoints_[index + 1]);
}

void PartitionedInterval::Print(std::ostream& os) const 
{
    if (empty()) 
    {
        os << "[]";
        return;
    }
    os << "[";
    for (size_t i = 0; i < breakpoints_.size(); ++i) 
    {
        os << breakpoints_[i];
        if (i + 1 < breakpoints_.size()) os << ", ";
    }
    os << "]";
}

double PartitionedInterval::left() const 
{
    // If empty, return positive infinity.
    return empty() ? std::numeric_limits<double>::infinity() : breakpoints_.front();
}

double PartitionedInterval::right() const 
{
    // If empty, return negative infinity.
    return empty() ? -std::numeric_limits<double>::infinity() : breakpoints_.back();
}

const std::vector<double>& PartitionedInterval::get_breakpoints() const 
{
    return breakpoints_;
}

// JSON serialization: expects a JSON array [left, ..., right]
void from_json(const nlohmann::json& j, PartitionedInterval& i) {
    // Parse the JSON array as a vector of doubles.
    std::vector<double> b = j.get<std::vector<double>>();
    if (!b.empty()) {
        // Construct a new PartitionedInterval ensuring the invariant holds.
        i = PartitionedInterval(b);
    } else {
        i = PartitionedInterval();
    }
}

void to_json(nlohmann::json& j, const PartitionedInterval& i) {
    // Serialize as a JSON array of breakpoints.
    j = i.get_breakpoints();
}

} // namespace goc
