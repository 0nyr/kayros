#include "goc/math/pwl_function.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

goc::PWLFunction choices()
{
    goc::PWLFunction f;
    f.AddPiece(goc::LinearFunction({770.0, 62.6125}, {770.0, 50.425}).as_choice_vertical());
    f.AddPiece(goc::LinearFunction({770.0, 50.425}, {770.0, 50.0}).as_choice_vertical());
    f.AddPiece(goc::LinearFunction({770.0, 50.0}, {770.0, 41.0272189349112}).as_choice_vertical());
    f.AddPiece(goc::LinearFunction({770.0, 41.0272189349112}, {771.0, 41.1195266272189}));
    return f;
}

void check(bool step_exact)
{
    goc::StepExactArithmeticScope scope(step_exact);
    const auto f = choices();
    const auto travel = goc::PWLFunction::ConstantFunction(19.0, {770.0, 770.0});
    const auto departure = goc::PWLFunction::ConstantFunction(770.0, {789.0, 789.0});
    const double expected = f.MinValueAt(770.0) + 19.0;
    for (const auto& sum : {f + travel, travel + f})
    {
        require(sum.Domain().left == 770.0 && sum.Domain().right == 770.0,
                "A scalar point must restrict the sum to its abscissa");
        require(std::abs(sum.MinValueAt(770.0) - expected) < 1e-10,
                "Adding a scalar point lost an attainable departure choice");
        const auto extended = sum.Compose(departure);
        require(std::abs(extended.MinValueAt(789.0) - expected) < 1e-10,
                "Tight-arc extension must retain the best prefix duration");
    }
    const auto outside = goc::PWLFunction::ConstantFunction(19.0, {772.0, 772.0});
    require((f + outside).Empty() && (outside + f).Empty(),
            "A disjoint point must not create a new feasible time");
    const auto ordinary = goc::PWLFunction::ConstantFunction(2.0, {0.0, 4.0});
    const auto point = goc::PWLFunction::ConstantFunction(3.0, {4.0, 4.0});
    require((ordinary + point).MinValueAt(4.0) == 5.0,
            "Ordinary scalar-point addition must remain unchanged");

    goc::PWLFunction boundary;
    boundary.AddPiece(goc::LinearFunction({760.0, 43.0}, {770.0, 43.0}));
    boundary.AddPiece(goc::LinearFunction({770.0, 43.0}, {770.0, 41.0}).as_choice_vertical());
    for (const auto& sum : {boundary + travel, travel + boundary})
        require(sum.MinValueAt(770.0) == 60.0,
                "An ordinary segment followed by a boundary choice must retain the choice");

    goc::PWLFunction jump;
    jump.AddPiece(goc::LinearFunction({0.0, 2.0}, {1.0, 2.0}));
    jump.AddPiece(goc::LinearFunction({1.0, 2.0}, {1.0, 8.0}));
    jump.AddPiece(goc::LinearFunction({1.0, 8.0}, {2.0, 8.0}));
    const auto at_jump = goc::PWLFunction::ConstantFunction(3.0, {1.0, 1.0});
    for (const auto& sum : {jump + at_jump, at_jump + jump})
    {
        require(sum.PieceCount() == 2 && sum.Piece(1).is_jump_vertical(),
                "The point-choice repair must leave the unrelated JUMP sweep unchanged");
        require(sum.Value(1.0) == 5.0,
                "The attained side of an unrelated JUMP must remain unchanged");
    }
}
}

int main()
{
    try
    {
        check(false);
        check(true);
        std::cout << "Point-addition and tight-extension invariants pass in legacy and step-exact modes\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
