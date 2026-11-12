// Regression test for PWLDominationFunction::DominatePieces, case D, on a decreasing piece (NOTICE.md item 15).
//
// The candidate function f1 is one decreasing piece on [1122.2, 1266.199] with values from 642.2 down to 546.2, and the dominated interval ends at 1175.501. These numbers come from a path trace of a label in a KAYROS release before 1.7.0. The function f2 is a synthetic constant piece on [1150, 1200] that dominates f1 exactly on [1150, 1175.501]. Partial dominance must remove that interval and keep f1 unchanged elsewhere, so the right part must run from about 606.665 at 1175.501 down to 546.2 at 1266.199. Removing dominated parts may shrink the domain of f1 but must never change its values. Before 1.7.0 the right part was rebuilt with the maximum of the piece (p1.image.right) as its value at the right end, which this test rejects.
//
// Adapted from code/tests/test_partial_dominance.cpp of https://github.com/0nyr/lera-tdvrptw-bpc-repaired. CMake target kayros_test_partial_dominance (not part of the default build). Exit code 0 on success, 1 on failure.

#include <cmath>
#include <iostream>

#include "goc/goc.h"
#include "labeling/pwl_domination_function.h"

using namespace std;
using namespace goc;
using namespace solver;

int main()
{
	const double tl = 1122.2, tr = 1266.199, vl = 642.2, vr = 546.2, split = 1175.501;
	PWLFunction f1;
	f1.AddPiece(LinearFunction({tl, vl}, {tr, vr}));
	const double v_split = f1.Value(split);
	PWLFunction f2;
	f2.AddPiece(LinearFunction({1150.0, v_split}, {1200.0, v_split}));

	PWLDominationFunction g(f1);
	bool fully_dominated = g.DominatePieces(f2);
	PWLFunction result = static_cast<PWLFunction>(g);

	int failures = 0;
	if (fully_dominated) { cout << "FAIL: f1 reported as fully dominated" << endl; ++failures; }
	cout << "Pieces after DominatePieces:" << endl;
	for (auto& p: result.Pieces())
	{
		double xl = p.domain.left, xr = p.domain.right;
		double yl = p.Value(xl), yr = p.Value(xr);
		cout << "  [" << xl << ", " << xr << "] values " << yl << " -> " << yr << endl;
		for (double x: {xl, xr})
		{
			double expected = f1.Value(x);
			if (fabs(p.Value(x) - expected) > 1e-6)
			{
				cout << "FAIL: value " << p.Value(x) << " at t = " << x << ", expected " << expected << endl;
				++failures;
			}
		}
	}
	if (result.PieceCount() == 0 || fabs(result.Domain().right - tr) > 1e-6)
	{
		cout << "FAIL: the right part [" << split << ", " << tr << "] was not kept" << endl;
		++failures;
	}
	cout << (failures == 0 ? "PASS" : "FAIL") << endl;
	return failures == 0 ? 0 : 1;
}
