#pragma once

/**
 * Nyr is a C++ library for solving vehicle routing problems (VRP),
 * in a Time Dependent (TD) context.
 * 
 * It builds on GOC library for graph and combinatorial optimization.
 */

#include "nyr/solutions/route.h"

#include "nyr/math/fast_math.h"
#include "nyr/math/interval.h"
#include "nyr/math/ndcpwlf.h"

#include "nyr/vrp/types.h"
#include "nyr/vrp/instance.h"
