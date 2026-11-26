// The kernel keeps every slope it computes and merges adjacent pieces only
// when their slopes are exactly equal (rows P6 of the M2 inventory and the
// AddPiece tolerance of the s139 residual experiment).
#include "goc/math/pwl_function.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

static void require(bool value, const char* what)
{
    if (!value) { std::cerr << "exact slopes regression: " << what << "\n"; std::exit(1); }
}

int main()
{
    // A nearly flat piece keeps its slope and both of its end values. With
    // the former snap of slopes below 1e-6 to 0, the left value became 10.0005.
    goc::LinearFunction flat({0.0, 10.0}, {1000.0, 10.0005});
    require(flat.slope > 0.0, "nearly flat slope kept");
    require(std::fabs(flat.Value(0.0) - 10.0) <= 1e-12 && std::fabs(flat.Value(1000.0) - 10.0005) <= 1e-12,
            "nearly flat piece keeps its end values");

    // Slopes 1e-3 and 1.0005e-3 differ by 5e-7, within the former merge
    // tolerance. The merged piece carried the incoming line, so the value at 0
    // was -0.0005 instead of 0.
    goc::PWLFunction f({0.0, 1000.0, 2000.0}, {0.0, 1.0, 2.0005});
    require(f.PieceCount() == 2, "pieces of different slopes stay apart");
    require(std::fabs(f.Value(0.0)) <= 1e-12 && std::fabs(f.Value(1000.0) - 1.0) <= 1e-12
                && std::fabs(f.Value(2000.0) - 2.0005) <= 1e-12,
            "breakpoint values kept");

    // Exactly collinear pieces still merge.
    goc::PWLFunction g({0.0, 1000.0, 2000.0}, {0.0, 1.0, 2.0});
    require(g.PieceCount() == 1, "collinear pieces merge");

    std::cout << "exact slopes: ok\n";
    return 0;
}
