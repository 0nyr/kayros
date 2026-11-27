// Exact LP multipliers of the restricted master (Kayros exact layer, M2 design
// note of session 139, section 3 element 3 and Claim 10).
//
// The binary64 LP backend proposes an optimal basis. This component solves that
// basis in GMP rationals, tests it for exact primal and dual feasibility, and
// repairs it by exact primal simplex pivots within a budget. It returns either an
// exact optimum of the master (multipliers, primal solution, value) that it has
// verified directly on the data, or the status unresolved with its reason. It
// never returns a multiplier vector that failed the verification.
//
// The component is not trusted by the proof (Claim 10), since the leaf bound
// holds for any multipliers. It decides only whether the bound is good. It has
// no phase 1. A basis that is singular or not exactly primal feasible is unresolved, as is a
// basis that needs more pivots than the budget. The master keeps artificial
// columns (Claim 11) and the backend provides a basis at every round, so neither
// case is expected in the search.
#pragma once

#include <gmpxx.h>

#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

namespace kayros::exact_lp {

// Every variable has lower bound 0. Equality rows have a logical fixed at 0, the
// logical of a "<= " row is its nonnegative slack, so its multiplier is <= 0 in
// an optimum (minimisation).
enum class Sense { Equal, LessEqual };

struct Entry {
    int row = 0;
    long coefficient = 0;  // nonzero
};

struct Column {
    mpq_class cost;
    std::vector<Entry> entries;  // rows strictly increasing
    bool fixed_at_zero = false;  // upper bound 0 (column of a forbidden arc at a node)
};

struct Master {
    std::vector<Sense> sense;
    std::vector<mpq_class> rhs;
    std::vector<Column> columns;
    int rows() const { return (int)sense.size(); }
};

// A basis lists the rows() basic variables. Id j >= 0 is column j, id
// logical(i) is the logical of row i. Every other variable is nonbasic at 0.
using Basis = std::vector<int>;
constexpr int logical(int row) { return -1 - row; }

enum class Status { Optimal, Unresolved };

enum class Reason {
    None,
    InvalidBasis,      // wrong size, unknown or repeated id, or a fixed column entering
    Singular,          // the basis matrix is singular
    PrimalInfeasible,  // the basic solution of the start basis is not exactly primal feasible
    Unbounded,         // a column with negative reduced cost and no leaving variable
    PivotBudget,       // the budget of exact pivots was used up
    VerificationFailed // the final answer failed the direct check (would be a defect of this component)
};

struct Solution {
    Status status = Status::Unresolved;
    Reason reason = Reason::None;
    mpq_class z;                                     // exact value, set when optimal
    std::vector<mpq_class> duals;                    // 1 per row when optimal: pi on "=" rows, sigma <= 0 on "<=" rows
    std::vector<std::pair<int, mpq_class>> primal;   // columns with a nonzero value when optimal, by increasing index
    Basis basis;                                     // last basis (optimal basis when optimal)
    int pivots = 0;                                  // exact pivots done
    int degenerate_pivots = 0;
};

// Default budget of exact pivots per call. See the step 4 report of session
// 139 for the measurements it is chosen from.
constexpr int default_pivot_budget = 100;

// Solves the master exactly from the start basis. The entering variable has
// the most negative reduced cost. The leaving variable has the minimum ratio,
// and a basic fixed variable (fixed column or logical of an "=" row) leaves at
// ratio 0. Ties go
// to the first variable in the order columns by index, then logicals by row.
// The budget bounds the number of pivots, so the call always ends, cycling
// included. The answer is verified by verify() before it is returned.
Solution solve_from_basis(const Master &master, const Basis &start, int pivot_budget = default_pivot_budget);

// Direct check of an optimal solution, without the basis: x >= 0, fixed columns
// at 0, rows satisfied, sigma <= 0, every reduced cost of a column that is not
// fixed is >= 0, and the primal value equals the dual value equals z. Returns
// an empty string when every condition holds, else the first condition that fails.
std::string verify(const Master &master, const Solution &solution);

const char *to_string(Reason reason);

// Master file, one item per line:
//   kayros-exact-lp-master 1
//   rows M
//   E|L RHS                                  (M lines)
//   columns N
//   COST BINARY64 UPPER K r1:a1 ... rK:aK    (N lines; UPPER is inf or 0)
// COST and RHS are rationals p/q or integers. BINARY64 is a hex float, the
// value handed to a binary64 backend. Throws std::runtime_error on bad input.
struct MasterFile {
    Master master;
    std::vector<double> binary64_cost;
};
MasterFile read_master(std::istream &in);

}  // namespace kayros::exact_lp
