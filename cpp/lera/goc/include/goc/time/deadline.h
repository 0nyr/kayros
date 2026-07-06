//
// kayros addition (M5.2, not part of the original GOC library).
//
// An absolute wall-clock deadline shared across the BCP stack. One Deadline is
// derived from the caller's time limit at BCP::Run and every component
// (column generation, pricing, cut separation, strong branching, freeze
// heuristic, labeling merges) consults it via Reached()/Remaining() instead of
// re-deriving nested per-level budgets from fresh stopwatches, so residual
// budgets cannot drift and no component can outlive the caller's limit.
//

#ifndef GOC_TIME_DEADLINE_H
#define GOC_TIME_DEADLINE_H

#include <chrono>

#include "goc/time/duration.h"

namespace goc
{
class Deadline
{
public:
	// No deadline: Reached() is always false, Remaining() is Duration::Max().
	Deadline() : set_(false) {}

	// Returns: the deadline 'd' from now; Duration::Max() (and above) means no deadline.
	static Deadline In(Duration d)
	{
		Deadline dl;
		if (d >= Duration::Max()) return dl;
		dl.set_ = true;
		dl.tp_ = std::chrono::steady_clock::now() +
			std::chrono::microseconds((long long)(d.Amount(DurationUnit::Milliseconds) * 1000.0));
		return dl;
	}

	bool IsSet() const { return set_; }

	bool Reached() const { return set_ && std::chrono::steady_clock::now() >= tp_; }

	// Returns: the time left (clamped at 0); Duration::Max() if no deadline is set.
	Duration Remaining() const
	{
		if (!set_) return Duration::Max();
		auto left_us = std::chrono::duration_cast<std::chrono::microseconds>(
			tp_ - std::chrono::steady_clock::now()).count();
		if (left_us <= 0) return Duration::None();
		return Duration(left_us / 1000.0, DurationUnit::Milliseconds);
	}

private:
	bool set_;
	std::chrono::steady_clock::time_point tp_;
};
} // namespace goc

#endif //GOC_TIME_DEADLINE_H
