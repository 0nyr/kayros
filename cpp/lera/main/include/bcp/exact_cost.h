// kayros-added (not vendored code). Exact column costs of the master.
#pragma once

#include <gmpxx.h>

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

namespace solver
{
using ExactCost = mpq_class;

// The binary64 value nearest to q, ties to even. mpq_get_d truncates toward
// zero, so the other neighbour is compared with it in rationals.
inline double nearest_binary64(const ExactCost& q)
{
	const double d = q.get_d();
	const ExactCost dq(d);
	if (dq == q) return d;
	const double other = std::nextafter(d, q > dq ? std::numeric_limits<double>::infinity()
	                                              : -std::numeric_limits<double>::infinity());
	if (!std::isfinite(other)) return d;
	const ExactCost below = abs(q - dq), above = abs(q - ExactCost(other));
	if (below != above) return below < above ? d : other;
	return (std::bit_cast<std::uint64_t>(d) & 1) == 0 ? d : other;
}

// The least binary64 value that is at least q.
inline double binary64_at_least(const ExactCost& q)
{
	const double d = nearest_binary64(q);
	return ExactCost(d) < q ? std::nextafter(d, std::numeric_limits<double>::infinity()) : d;
}
} // namespace solver
