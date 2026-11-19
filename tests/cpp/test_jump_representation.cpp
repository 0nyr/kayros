// Unit tests of the representation of value jumps (NOTICE.md item 18, 1.8.0).
//
// A value jump of an arrival function is carried either by a JUMP vertical piece or by a bare jump, i.e. two consecutive pieces that share an abscissa and differ in value there. Both denote the same left-continuous function. Before 1.8.0, PWLFunction::Inverse joined the lower point of a bare jump to the right end of the next piece by a chord, the JSON form of a vertical piece was a single point, and PWLFunction::Compose gave an image to a JUMP vertical of the inner function that lies outside the domain of the outer one.
//
// CMake target kayros_test_jump_representation, part of kayros_cpp_tests.

#include <cmath>
#include <iostream>
#include <string>

#include "goc/goc.h"

using namespace std;
using namespace goc;

namespace
{
int failures = 0;

void require(bool condition, const string& what)
{
	if (condition) return;
	cout << "FAIL: " << what << endl;
	++failures;
}

bool close_to(double a, double b) { return fabs(a - b) <= 1e-9; }

// Arrival function with a jump of 5 at abscissa 10: slope 1 from (0, 100) to (10, 110), then slope 1 from (10, 115) to (20, 125).
PWLFunction arrival(bool with_vertical)
{
	PWLFunction f;
	f.AddPiece(LinearFunction({0.0, 100.0}, {10.0, 110.0}));
	if (with_vertical) f.AddPiece(LinearFunction({10.0, 110.0}, {10.0, 115.0}));
	f.AddPiece(LinearFunction({10.0, 115.0}, {20.0, 125.0}));
	return f;
}

template <class F> bool throws_bare_jump(F&& call, double abscissa)
{
	try { call(); }
	catch (const BareJumpError& e) { return close_to(e.abscissa, abscissa); }
	return false;
}

void test_inverse_of_a_bare_upward_jump()
{
	StepExactArithmeticScope step(true);
	for (bool with_vertical : {false, true})
	{
		const string form = with_vertical ? "vertical form" : "bare form";
		PWLFunction dep = arrival(with_vertical).Inverse();
		require(close_to(dep.Domain().left, 100.0) && close_to(dep.Domain().right, 125.0), form + ": domain of the inverse is [100, 125]");
		// Latest departure that arrives by t: t - 100 up to 110, then 10 on the whole jump [110, 115], then t - 105.
		const double expected[][2] = {{100.0, 0.0}, {105.0, 5.0}, {110.0, 10.0}, {111.0, 10.0}, {112.5, 10.0}, {115.0, 10.0}, {116.0, 11.0}, {120.0, 15.0}, {125.0, 20.0}};
		for (auto& e : expected)
			require(close_to(dep.Value(e[0]), e[1]), form + ": inverse at " + to_string(e[0]) + " is " + to_string(dep.Value(e[0])) + ", expected " + to_string(e[1]));
		for (auto& p : dep.Pieces())
			if (!p.is_vertical()) require(close_to(p.slope, 0.0) || close_to(p.slope, 1.0), form + ": a piece of the inverse has slope " + to_string(p.slope));
	}
}

void test_inverse_refuses_what_it_cannot_invert()
{
	{
		StepExactArithmeticScope step(true);
		PWLFunction down;
		down.AddPiece(LinearFunction({0.0, 100.0}, {10.0, 110.0}));
		down.AddPiece(LinearFunction({10.0, 105.0}, {20.0, 115.0}));
		require(throws_bare_jump([&] { down.Inverse(); }, 10.0), "a downward bare jump is refused on a step-carrying solve");
		PWLFunction hole;
		hole.AddPiece(LinearFunction({0.0, 100.0}, {10.0, 110.0}));
		hole.AddPiece(LinearFunction({12.0, 112.0}, {20.0, 120.0}));
		require(throws_bare_jump([&] { hole.Inverse(); }, 12.0), "a domain gap is refused on a step-carrying solve");
	}
	{
		StepExactArithmeticScope jump_free(false);
		require(throws_bare_jump([&] { arrival(false).Inverse(); }, 10.0), "a bare jump is refused on a jump-free solve");
		// A gap below BARE_JUMP_TOLERANCE is rounding on a jump-free solve: no error, historical arithmetic.
		PWLFunction dust;
		dust.AddPiece(LinearFunction({0.0, 100.0}, {10.0, 110.0}));
		dust.AddPiece(LinearFunction({10.0, 110.0 + 1e-5}, {20.0, 120.0}));
		PWLFunction dep = dust.Inverse();
		require(close_to(dep.Value(100.0), 0.0) && close_to(dep.Value(120.0), 20.0), "a rounding gap is inverted as before on a jump-free solve");
		PWLFunction continuous;
		continuous.AddPiece(LinearFunction({0.0, 100.0}, {10.0, 110.0}));
		continuous.AddPiece(LinearFunction({10.0, 110.0}, {20.0, 130.0}));
		PWLFunction inv = continuous.Inverse();
		require(inv.PieceCount() == 2 && close_to(inv.Value(120.0), 15.0), "a continuous function is inverted as before");
	}
}

void test_json_round_trip_keeps_verticals()
{
	const LinearFunction jump({10.0, 110.0}, {10.0, 115.0});
	nlohmann::json j = jump;
	require(j.size() == 2, "a JUMP vertical is written as 2 points");
	LinearFunction back = j;
	require(back.is_jump_vertical() && close_to(back.Value(10.0), 110.0) && close_to(back.image.left, 110.0) && close_to(back.image.right, 115.0),
		"a JUMP vertical survives the JSON round trip with its attained value and its span");

	const LinearFunction choice = LinearFunction({10.0, 115.0}, {10.0, 110.0}).as_choice_vertical();
	nlohmann::json jc = choice;
	LinearFunction back_choice = jc;
	require(jc.size() == 3 && back_choice.is_choice_vertical() && close_to(back_choice.Value(10.0), choice.Value(10.0))
		&& close_to(back_choice.image.left, 110.0) && close_to(back_choice.image.right, 115.0), "a CHOICE vertical survives the JSON round trip with its kind");

	const LinearFunction plain({0.0, 100.0}, {10.0, 110.0});
	nlohmann::json jp = plain;
	LinearFunction back_plain = jp;
	require(jp.size() == 2 && !back_plain.is_vertical() && back_plain.slope == plain.slope && back_plain.intercept == plain.intercept, "an ordinary piece is written and read as before");

	StepExactArithmeticScope step(true);
	nlohmann::json jf = arrival(true);
	PWLFunction f = jf;
	require(f.PieceCount() == 3 && f.Piece(1).is_jump_vertical(), "a function keeps its JUMP vertical through JSON");
	require(close_to(f.Inverse().Value(112.5), 10.0), "the inverse of the round-tripped function has the plateau");
}

void test_compose_drops_a_jump_outside_the_outer_domain()
{
	StepExactArithmeticScope step(true);
	PWLFunction outer; // defined on [0, 100]
	outer.AddPiece(LinearFunction({0.0, 1.0}, {100.0, 101.0}));
	PWLFunction inner; // reaches 100 at abscissa 5, jumps to 120 there, then jumps again at 8 from 123 to 130
	inner.AddPiece(LinearFunction({0.0, 90.0}, {5.0, 100.0}));
	inner.AddPiece(LinearFunction({5.0, 100.0}, {5.0, 120.0}));
	inner.AddPiece(LinearFunction({5.0, 120.0}, {8.0, 123.0}));
	inner.AddPiece(LinearFunction({8.0, 123.0}, {8.0, 130.0}));
	inner.AddPiece(LinearFunction({8.0, 130.0}, {10.0, 132.0}));
	PWLFunction fog = outer.Compose(inner);
	require(!fog.Empty() && close_to(fog.Domain().left, 0.0) && close_to(fog.Domain().right, 5.0), "the composition is defined on [0, 5] only");
	require(close_to(fog.Value(5.0), 101.0), "the composition attains outer(100) at abscissa 5");
}
} // namespace

int main()
{
	test_inverse_of_a_bare_upward_jump();
	test_inverse_refuses_what_it_cannot_invert();
	test_json_round_trip_keeps_verticals();
	test_compose_drops_a_jump_outside_the_outer_domain();
	cout << (failures == 0 ? "PASS" : "FAIL") << endl;
	return failures == 0 ? 0 : 1;
}
