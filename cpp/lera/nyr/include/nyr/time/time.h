#pragma once

#include <chrono>
#include <goc/goc.h>

namespace nyr
{
using Clock = std::chrono::high_resolution_clock;

// Program DURation EXecution: Pseudo-type alias for a duration in seconds.
class Durex : public goc::Printable
{
public:
    explicit Durex(double seconds = 0.0) : duration_(seconds) {}

    double count() const { return duration_.count(); }

    /**
     * Prints duration in seconds.
     * The output is formatted to 9 decimal places (nano-seconds).
     */
    void Print(std::ostream& os) const override
    {
        std::streamsize old_precision = os.precision();
        std::ios_base::fmtflags old_flags = os.flags();

        os << std::fixed << std::setprecision(9) << count();

        os.precision(old_precision);
        os.flags(old_flags);
    }

    // Conversion to std::chrono::duration<double>
    operator std::chrono::duration<double>() const { return duration_; }

    // Comparison operators for Durex
    bool operator==(const Durex& rhs) const
    {
        return duration_ == rhs.duration_;
    }

    bool operator!=(const Durex& rhs) const
    {
        return !(*this == rhs);
    }

    bool operator<(const Durex& rhs) const
    {
        return duration_ < rhs.duration_;
    }

    bool operator<=(const Durex& rhs) const
    {
        return duration_ <= rhs.duration_;
    }

    bool operator>(const Durex& rhs) const
    {
        return duration_ > rhs.duration_;
    }

    bool operator>=(const Durex& rhs) const
    {
        return duration_ >= rhs.duration_;
    }

private:
    std::chrono::duration<double> duration_; // duration in seconds
};

using TimePoint = Clock::time_point;

// Serializes a Durex to JSON.
inline void to_json(nlohmann::json& j, const Durex& d)
{
    j = d.count();
}

// Parses a Durex from JSON.
inline void from_json(const nlohmann::json& j, Durex& d)
{
    d = Durex(j.get<double>());
}

// Returns the time elapsed in seconds (as a double) from 'start' to now.
inline Durex seconds_since(const Clock::time_point& start)
{
    return Durex(std::chrono::duration<double>(Clock::now() - start).count());
}

// Converts a Durex to seconds as a double.
inline double seconds(const Durex& duration)
{
    return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::duration<double>(duration)).count();
}

/**
 * Overloads the << operator to print a Durex in seconds.
 * The output is formatted to 9 decimal places (nano-seconds).
 */
inline std::ostream& operator<<(std::ostream& os, const Durex& d)
{
    std::streamsize old_precision = os.precision();
    std::ios_base::fmtflags old_flags = os.flags();

    os << std::fixed << std::setprecision(9) << d.count() << "s";

    os.precision(old_precision);
    os.flags(old_flags);

    return os;
}

std::string generate_timestamp();

/**
 * @brief A class that represents the general clock of 
 * a program. Used to measure time elapsed in the program.
 */
class ProgramClock
{
public:
    // Constructor.
    ProgramClock() : program_start_time_(Clock::now()) {}

    // Returns the time since the program started in seconds.
    Durex elapsed() const
    {
        return seconds_since(program_start_time_);
    }
    // Returns the start time of the program.
    const TimePoint& start_time() const
    {
        return program_start_time_;
    }

private:
    // The start time of the program.
    const TimePoint program_start_time_;
};



} // namespace
