#pragma once

#include <bitset>

// MAX_N is the maximum number of vertices an instance may have, we need this at compilation time for bitset purposes.
#ifndef MAX_N
#define MAX_N 102
#endif

namespace nyr
{

typedef double TimeUnit; // Represents time (time point or duration).
typedef double CapacityUnit; // Represents the capacity.
typedef double ProfitUnit; // Represents the profit of vertices.
typedef std::bitset<MAX_N> VertexSet; // Set of vertices.

} // namespace nyr