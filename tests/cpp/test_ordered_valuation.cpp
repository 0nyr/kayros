#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <random>
#include <set>

#include "bcp/ordered_valuation.h"
#include "goc/linear_programming/highs/matrix_formulation.h"

static void require(bool value)
{
    if (!value) { std::cerr << "ordered valuation regression\n"; std::exit(1); }
}

int main()
{
    // Fractional values from the captured C201 root LP, including its dust.
    const std::array<double, 23> values{
        .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
        .05000000000000007, .05, .05, .05, .049999999999999795,
        .05, .050000000000000114, .05, .05, .049999999999999906,
        .049999999999999836, 1., 1.};
    std::mt19937 rng(58);
    std::set<uint64_t> raw_results;
    std::array<uint64_t, 3> expected{};
    for (int trial = 0; trial < 128; ++trial)
    {
        goc::MatrixFormulation formulation;
        std::vector<goc::Variable> variables;
        for (int index = 0; index < (int) values.size(); ++index)
            variables.push_back(formulation.AddVariable("y_" + std::to_string(index)));
        std::vector<int> order(values.size());
        std::iota(order.begin(), order.end(), 0);
        std::shuffle(order.begin(), order.end(), rng);
        goc::Valuation valuation;
        for (int index : order) valuation.SetValue(variables[index], values[index]);
        double raw = 0.;
        for (const auto& item : valuation) raw += item.second;
        raw_results.insert(std::bit_cast<uint64_t>(raw));
        auto sorted = solver::ordered_valuation(valuation);
        require(sorted.size() == values.size());
        std::array<double, 3> sums{};
        for (int index = 0; index < (int) sorted.size(); ++index)
        {
            require(sorted[index].first.Index() == index);
            require(std::bit_cast<uint64_t>(sorted[index].second) == std::bit_cast<uint64_t>(values[index]));
            sums[0] += sorted[index].second;
            if (index % 3) sums[1] += sorted[index].second;
            if (index % 2) sums[2] += sorted[index].second;
        }
        for (int index = 0; index < 3; ++index)
        {
            uint64_t bits = std::bit_cast<uint64_t>(sums[index]);
            if (trial == 0) expected[index] = bits;
            else require(bits == expected[index]);
        }
    }
    require(raw_results.size() > 1);
    std::cout << "PASS 128 valuation permutations, invariant canonical reductions, "
              << raw_results.size() << " distinct raw sum bit patterns\n";
}
