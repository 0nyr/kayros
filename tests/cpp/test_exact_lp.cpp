// Tests of the exact LP component (kayros::exact_lp) without an LP backend:
// hand-built masters, degenerate masters, an infeasible master closed by
// artificial columns of large rational cost, bases that need pivots, the
// unresolved outcomes, a brute-force cross-check over all bases of random small
// masters, and fixtures of real root masters with the basis of each backend.
#include "exact_lp/exact_lp.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

using namespace kayros::exact_lp;

namespace {
int checks = 0;

void require(bool value, const std::string &what) {
    ++checks;
    if (!value) {
        std::cerr << "exact lp regression: " << what << "\n";
        std::exit(1);
    }
}

mpq_class Q(const char *s) {
    mpq_class q(s, 10);
    q.canonicalize();
    return q;
}

Column col(mpq_class cost, std::vector<int> rows, bool fixed = false) {
    Column c;
    c.cost = cost;
    c.fixed_at_zero = fixed;
    for (int r : rows) c.entries.push_back({r, 1});
    return c;
}

// Set-partitioning master in the SPF layout: row 0 start depot (= 0, empty),
// rows 1..n customers (= 1), row n+1 end depot (= 0, empty), then cut rows (<= 1).
Master spf(int n, int cuts) {
    Master M;
    M.sense.assign(n + 2, Sense::Equal);
    M.rhs.assign(n + 2, mpq_class(1));
    M.rhs[0] = M.rhs[n + 1] = 0;
    for (int k = 0; k < cuts; ++k) M.sense.push_back(Sense::LessEqual), M.rhs.push_back(1);
    return M;
}

// Basis made of the given columns and the logicals of every row not listed.
Basis with_logicals(const Master &M, std::vector<int> columns, std::vector<int> skip_rows) {
    Basis B = columns;
    for (int i = 0; i < M.rows(); ++i)
        if (std::find(skip_rows.begin(), skip_rows.end(), i) == skip_rows.end()) B.push_back(logical(i));
    return B;
}

void require_optimal(const Master &M, const Solution &s, const mpq_class &z, const std::string &what) {
    require(s.status == Status::Optimal, what + ": optimal (reason " + to_string(s.reason) + ")");
    require(s.z == z, what + ": z = " + z.get_str() + ", got " + s.z.get_str());
    require(verify(M, s).empty(), what + ": verify " + verify(M, s));
}

// 3 customers, costs with non-dyadic denominators. Optimum {1,2} + {3} = 10/3 + 7/3 = 17/3.
Master three_customers() {
    Master M = spf(3, 0);
    M.columns = {col(Q("8/3"), {1}), col(Q("8/3"), {2}), col(Q("7/3"), {3}), col(Q("10/3"), {1, 2}),
                 col(Q("4"), {2, 3}), col(Q("7"), {1, 2, 3})};
    return M;
}

void hand_built() {
    Master M = three_customers();
    // Optimal basis: {1,2} and {3}, the logicals of the 2 depot rows, and {1} at value 0 (primal degenerate).
    Basis B = {3, 2, 0, logical(0), logical(4)};
    Solution s = solve_from_basis(M, B);
    require_optimal(M, s, Q("17/3"), "hand-built optimal basis");
    require(s.primal.size() == 2 && s.primal[0].first == 2 && s.primal[1].first == 3, "hand-built primal support");
    mpq_class dual_value = 0;
    for (int i = 0; i < M.rows(); ++i) dual_value += M.rhs[i] * s.duals[i];
    require(dual_value == Q("17/3"), "hand-built dual value");
    // Start from a basis that is primal feasible and not optimal: singletons {1}, {2}, {3}, value 23/3.
    Solution p = solve_from_basis(M, {0, 1, 2, logical(0), logical(4)});
    require_optimal(M, p, Q("17/3"), "hand-built from singletons");
    require(p.pivots >= 1, "hand-built from singletons needs a pivot");
    Solution none = solve_from_basis(M, {0, 1, 2, logical(0), logical(4)}, 0);
    require(none.status == Status::Unresolved && none.reason == Reason::PivotBudget && none.duals.empty(),
            "budget 0 on a basis that needs a pivot is unresolved");
}

void unresolved_outcomes() {
    Master M = three_customers();
    require(solve_from_basis(M, {0, 1, 2}).reason == Reason::InvalidBasis, "basis of the wrong size");
    require(solve_from_basis(M, {0, 0, 2, logical(0), logical(4)}).reason == Reason::InvalidBasis, "repeated id");
    require(solve_from_basis(M, {0, 1, 99, logical(0), logical(4)}).reason == Reason::InvalidBasis, "unknown column");
    // {1}, {2} and {1,2} are linearly dependent.
    require(solve_from_basis(M, {0, 1, 3, logical(0), logical(4)}).reason == Reason::Singular, "singular basis");
    // {2}, {1,2}, {2,3}: row 1 gives {1,2} = 1, row 3 gives {2,3} = 1, row 2 then gives {2} = -1.
    require(solve_from_basis(M, {1, 3, 4, logical(0), logical(4)}).reason == Reason::PrimalInfeasible, "primal infeasible basis");
    // An equality logical basic at a nonzero value is primal infeasible: drop column 2 ({3}) for the logical of row 3.
    require(solve_from_basis(M, {0, 1, logical(3), logical(0), logical(4)}).reason == Reason::PrimalInfeasible,
            "equality logical at a nonzero value");
}

// The 2 depot rows and a cut row. With the cut on {1,2,3}, the fractional point 1/2 ({1,2} + {2,3} + {1,3}) is cut.
void cut_row_sign() {
    Master M = spf(3, 1);
    const int cut = 5;
    M.columns = {col(Q("3"), {1}), col(Q("3"), {2}), col(Q("3"), {3}), col(Q("2"), {1, 2, cut}),
                 col(Q("2"), {2, 3, cut}), col(Q("2"), {1, 3, cut})};
    // Without the cut the optimum is 3 (each pair at 1/2). With it: at most 1 pair, optimum 2 + 3 = 5.
    Solution s = solve_from_basis(M, with_logicals(M, {0, 1, 2}, {1, 2, 3}));
    require_optimal(M, s, Q("5"), "cut master from singletons");
    require(sgn(s.duals[cut]) <= 0, "cut multiplier <= 0");
    require(sgn(s.duals[cut]) < 0, "the cut is binding: negative multiplier");
    // With right-hand side 2 the cut is loose: the pairs come back at 1/2 and the slack is basic.
    Master loose = M;
    loose.rhs[cut] = 2;
    Solution l = solve_from_basis(loose, with_logicals(loose, {0, 1, 2}, {1, 2, 3}));
    require_optimal(loose, l, Q("3"), "loose cut master");
    require(sgn(l.duals[cut]) == 0, "loose cut multiplier 0");
}

// An uncovered customer and an artificial column per customer, of cost M = 10^6 U + 1 with a rational U.
void artificial_columns() {
    Master M = spf(3, 0);
    const mpq_class U = Q("17/3"), big = U * 1000000 + 1;
    for (int i = 1; i <= 3; ++i) M.columns.push_back(col(big, {i}));
    Master feasible = M;
    for (const Column &c : three_customers().columns) feasible.columns.push_back(c);
    Basis art = with_logicals(M, {0, 1, 2}, {1, 2, 3});
    Solution f = solve_from_basis(feasible, art);
    require_optimal(feasible, f, U, "artificials, feasible master, from the all-artificial basis");
    // Customer 1 uncovered by the routes: only its artificial covers it.
    Master infeasible = M;
    for (const Column &c : three_customers().columns)
        if (c.entries.front().row != 1) infeasible.columns.push_back(c);
    Solution g = solve_from_basis(infeasible, art);
    // Best: artificial of 1 and the route {2,3} (cost 4).
    require_optimal(infeasible, g, big + 4, "artificials, customer 1 uncovered");
    require(g.z >= big, "z >= M when a customer is uncovered");
}

// Fixed columns (forbidden arcs at a node): they never enter, and a basic fixed column at 0 can leave.
void fixed_columns() {
    Master M = three_customers();
    M.columns[3].fixed_at_zero = true;  // {1,2} forbidden: optimum {1} + {2,3} = 8/3 + 4 = 20/3
    Solution s = solve_from_basis(M, {0, 1, 2, logical(0), logical(4)});
    require_optimal(M, s, Q("20/3"), "fixed column never enters");
    // Start {1}, {2,3}, {1,2} (fixed): row 3 gives {2,3} = 1, row 2 gives {1,2} = 0, row 1 gives {1} = 1. The fixed
    // column is basic at 0, {3} enters and the fixed column leaves at ratio 0.
    Solution t = solve_from_basis(M, {0, 4, 3, logical(0), logical(4)});
    require_optimal(M, t, Q("20/3"), "fixed column basic at 0");
    // A fixed column basic at a nonzero value is primal infeasible: {1,2} (fixed), {3}.
    require(solve_from_basis(M, {3, 2, 0, logical(0), logical(4)}).reason == Reason::PrimalInfeasible,
            "fixed column at a nonzero value");
}

// Independent reference for the brute force: Gauss-Jordan in rationals on B x = b.
bool basic_solution(const Master &M, const Basis &B, std::vector<mpq_class> &x) {
    const int m = M.rows();
    std::vector<std::vector<mpq_class>> a(m, std::vector<mpq_class>(m + 1, mpq_class(0)));
    for (int k = 0; k < m; ++k) {
        if (B[k] >= 0)
            for (const Entry &e : M.columns[B[k]].entries) a[e.row][k] = e.coefficient;
        else
            a[-1 - B[k]][k] = 1;
    }
    for (int i = 0; i < m; ++i) a[i][m] = M.rhs[i];
    for (int c = 0; c < m; ++c) {
        int p = c;
        while (p < m && sgn(a[p][c]) == 0) ++p;
        if (p == m) return false;
        std::swap(a[p], a[c]);
        for (int r = 0; r < m; ++r) {
            if (r == c || sgn(a[r][c]) == 0) continue;
            mpq_class f = a[r][c] / a[c][c];
            for (int k = c; k <= m; ++k) a[r][k] -= f * a[c][k];
        }
    }
    x.assign(m, 0);
    for (int i = 0; i < m; ++i) x[i] = a[i][m] / a[i][i];
    return true;
}

// Random masters with 4 customers, 1 or 2 cuts, a few random routes (some fixed), integer and
// rational costs, 1 artificial per customer of cost 50. Every primal feasible basis is a start, and each run must end at
// the minimum over all primal feasible bases.
void brute_force() {
    std::mt19937 rng(20261126);
    int starts = 0, with_pivots = 0, masters = 0;
    for (int trial = 0; trial < 40; ++trial) {
        const int n = 4, cuts = 1 + trial % 2;
        // Customer i is row i - 1, cut k is row n + k (no depot rows, to keep the enumeration small).
        Master M;
        M.sense.assign(n, Sense::Equal);
        M.rhs.assign(n, mpq_class(1));
        for (int k = 0; k < cuts; ++k) M.sense.push_back(Sense::LessEqual), M.rhs.push_back(1);
        std::vector<std::vector<int>> triples = {{1, 2, 3}, {2, 3, 4}};
        for (int i = 0; i < n; ++i) M.columns.push_back(col(mpq_class(50), {i}));
        const int routes = 5 + (int)(rng() % 3);
        for (int r = 0; r < routes; ++r) {
            std::vector<int> rows;
            for (int i = 1; i <= n; ++i)
                if (rng() % 2) rows.push_back(i - 1);
            if (rows.empty()) rows.push_back((int)(rng() % n));
            for (int k = 0; k < cuts; ++k) {
                int hit = 0;
                for (int v : triples[k]) hit += std::count(rows.begin(), rows.end(), v - 1);
                if (hit >= 2) rows.push_back(n + k);
            }
            mpq_class cost(int(rows.size() * 3 + rng() % 7), int(1 + rng() % 3));
            cost.canonicalize();
            M.columns.push_back(col(cost, rows, rng() % 7 == 0));
        }
        // All variable ids: columns, then logicals.
        std::vector<int> ids;
        for (int j = 0; j < (int)M.columns.size(); ++j) ids.push_back(j);
        for (int i = 0; i < M.rows(); ++i) ids.push_back(logical(i));
        const int m = M.rows(), N = (int)ids.size();
        // Enumerate the subsets of size m, keep the primal feasible bases, take the minimum value.
        std::vector<Basis> feasible;
        mpq_class best;
        bool have = false;
        std::vector<int> pick(m);
        for (int k = 0; k < m; ++k) pick[k] = k;
        for (;;) {
            Basis B;
            for (int k : pick) B.push_back(ids[k]);
            std::vector<mpq_class> x;
            if (basic_solution(M, B, x)) {
                bool ok = true;
                mpq_class v = 0;
                for (int k = 0; k < m && ok; ++k) {
                    const bool fixed = B[k] >= 0 ? M.columns[B[k]].fixed_at_zero : M.sense[-1 - B[k]] == Sense::Equal;
                    ok = fixed ? sgn(x[k]) == 0 : sgn(x[k]) >= 0;
                    if (B[k] >= 0) v += M.columns[B[k]].cost * x[k];
                }
                if (ok) {
                    feasible.push_back(B);
                    if (!have || v < best) best = v, have = true;
                }
            }
            int k = m - 1;
            while (k >= 0 && pick[k] == N - m + k) --k;
            if (k < 0) break;
            ++pick[k];
            for (int q = k + 1; q < m; ++q) pick[q] = pick[q - 1] + 1;
        }
        require(have, "brute force: the artificial basis is feasible");
        ++masters;
        for (const Basis &B : feasible) {
            Solution s = solve_from_basis(M, B, 1000);
            require_optimal(M, s, best, "brute force trial " + std::to_string(trial));
            ++starts;
            if (s.pivots > 0) ++with_pivots;
        }
    }
    std::cout << "brute force: " << masters << " masters, " << starts << " primal feasible start bases, " << with_pivots
              << " needed pivots: ok\n";
    require(with_pivots > 0, "brute force exercised pivots");
}

std::string read_file(const std::string &path) {
    std::ifstream in(path);
    require((bool)in, "cannot read " + path);
    std::stringstream s;
    s << in.rdbuf();
    return s.str();
}

// Real root masters of the s139 exact LP experiment, with the basis of each backend stored by the driver, and the
// exact value recorded by that experiment.
void fixtures() {
    const std::string dir = KAYROS_EXACT_LP_DATA;
    std::istringstream list(read_file(dir + "/fixtures.txt"));
    std::string name, z, line;
    int count = 0;
    while (std::getline(list, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        fields >> name >> z;
        std::istringstream master_text(read_file(dir + "/" + name + ".master"));
        Master M = read_master(master_text).master;
        for (const char *backend : {"highs", "cplex"}) {
            std::istringstream basis_text(read_file(dir + "/" + name + "." + backend + ".basis"));
            Basis B;
            int id;
            while (basis_text >> id) B.push_back(id);
            Solution s = solve_from_basis(M, B);
            require_optimal(M, s, Q(z.c_str()), "fixture " + name + " from the " + backend + " basis");
            std::cout << "fixture " << name << " (" << M.rows() << " rows, " << M.columns.size() << " columns) from the "
                      << backend << " basis: " << s.pivots << " pivots, ok\n";
            ++count;
        }
    }
    require(count >= 2, "fixtures read");
}

void file_format() {
    std::istringstream good("kayros-exact-lp-master 1\nrows 2\nE 1\nL 3/2\ncolumns 2\n1/3 0x1.5555555555555p-2 inf 2 0:1 1:2\n"
                            "2 0x1p+1 0 1 0:-1\n");
    MasterFile f = read_master(good);
    require(f.master.rows() == 2 && f.master.columns.size() == 2, "file: sizes");
    require(f.master.rhs[1] == Q("3/2") && f.master.sense[1] == Sense::LessEqual, "file: row");
    require(f.master.columns[0].cost == Q("1/3") && f.binary64_cost[0] == 1.0 / 3.0, "file: costs");
    require(f.master.columns[0].entries[1].coefficient == 2 && f.master.columns[1].fixed_at_zero, "file: entries");
    for (const char *bad : {"kayros-exact-lp-master 2\n", "kayros-exact-lp-master 1\nrows 1\nG 1\ncolumns 0\n",
                            "kayros-exact-lp-master 1\nrows 1\nE 1/0\ncolumns 0\n",
                            "kayros-exact-lp-master 1\nrows 1\nE 1\ncolumns 1\n1 0x1p+0 inf 1 1:1\n",
                            "kayros-exact-lp-master 1\nrows 2\nE 1\nE 1\ncolumns 1\n1 0x1p+0 inf 2 1:1 0:1\n"}) {
        std::istringstream in(bad);
        bool threw = false;
        try {
            read_master(in);
        } catch (const std::runtime_error &) {
            threw = true;
        }
        require(threw, "file: refused input");
    }
}
}  // namespace

int main() {
    hand_built();
    unresolved_outcomes();
    cut_row_sign();
    artificial_columns();
    fixed_columns();
    file_format();
    brute_force();
    fixtures();
    std::cout << "exact lp: " << checks << " checks ok\n";
    return 0;
}
