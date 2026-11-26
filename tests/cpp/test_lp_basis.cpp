// Basis export of the configured LP backend (HiGHS or CPLEX) and its exact
// solve by kayros::exact_lp. The test is the same for both backends.
#include <goc/goc.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "exact_lp/exact_lp.h"
#include "exact_lp/goc_master.h"

using namespace goc;
namespace xlp = kayros::exact_lp;

namespace {
void require(bool value, const std::string &what) {
    if (!value) {
        std::cerr << "lp basis regression: " << what << "\n";
        std::exit(1);
    }
}

mpq_class Q(const char *s) {
    mpq_class q(s, 10);
    q.canonicalize();
    return q;
}

xlp::Column col(mpq_class cost, std::vector<int> rows) {
    xlp::Column c;
    c.cost = cost;
    for (int r : rows) c.entries.push_back({r, 1});
    return c;
}

// Rows in the SPF layout: start depot (= 0, empty), customers 1..3 (= 1), end depot (= 0, empty), then 1 cut on
// {1,2,3} (<= 1) when `cut` holds. Costs with denominator 3, so the backend sees rounded costs.
xlp::Master three_customers(bool cut) {
    xlp::Master M;
    M.sense.assign(5, xlp::Sense::Equal);
    M.rhs = {0, 1, 1, 1, 0};
    if (cut) M.sense.push_back(xlp::Sense::LessEqual), M.rhs.push_back(1);
    auto c = [&](std::vector<int> rows) {
        if (cut && rows.size() >= 2) rows.push_back(5);
        return rows;
    };
    M.columns = {col(Q("8/3"), c({1})), col(Q("8/3"), c({2})), col(Q("7/3"), c({3})), col(Q("10/3"), c({1, 2})),
                 col(Q("4"), c({2, 3})), col(Q("7"), c({1, 2, 3}))};
    return M;
}

std::vector<double> nearest(const xlp::Master &M) {
    std::vector<double> v;
    for (const auto &c : M.columns) v.push_back(c.cost.get_d());
    return v;
}

// Solves the first `count` columns with the backend, then the full master exactly from the backend basis.
xlp::Solution backend_then_exact(const xlp::Master &M, const std::vector<double> &costs, long count,
                                 const std::string &what) {
    Formulation *f = xlp::to_formulation(M, costs, count);
    LPSolver solver;
    LPExecutionLog log = solver.Solve(f, {LPOption::Duals, LPOption::Basis});
    delete f;
    std::ostringstream status;
    status << *log.status;
    require(status.str() == "Optimum", what + ": backend status " + status.str());
    require(log.basis.IsSet(), what + ": basis set");
    require((int)(log.basis->columns.size() + log.basis->rows.size()) == M.rows(), what + ": 1 basic variable per row");
    xlp::Solution s = xlp::solve_from_basis(M, xlp::to_basis(*log.basis));
    require(s.status == xlp::Status::Optimal, what + ": exact optimum (" + xlp::to_string(s.reason) + ")");
    require(xlp::verify(M, s).empty(), what + ": verify");
    return s;
}

void no_basis_unless_requested() {
    xlp::Master M = three_customers(false);
    Formulation *f = xlp::to_formulation(M, nearest(M), (long)M.columns.size());
    LPSolver solver;
    LPExecutionLog log = solver.Solve(f, {LPOption::Duals, LPOption::Incumbent});
    require(!log.basis.IsSet(), "no basis without LPOption::Basis");
    delete f;
}

void small_masters() {
    xlp::Master M = three_customers(false);
    xlp::Solution s = backend_then_exact(M, nearest(M), (long)M.columns.size(), "3 customers");
    require(s.z == Q("17/3"), "3 customers: z = 17/3");
    xlp::Master C = three_customers(true);
    xlp::Solution t = backend_then_exact(C, nearest(C), (long)C.columns.size(), "3 customers with a cut");
    require(t.z == Q("17/3") && sgn(t.duals[5]) <= 0, "3 customers with a cut: z = 17/3, sigma <= 0");
    // A forbidden arc at a node: {1,2} fixed at 0, optimum {1} + {2,3} = 20/3.
    xlp::Master F = three_customers(false);
    F.columns[3].fixed_at_zero = true;
    xlp::Solution u = backend_then_exact(F, nearest(F), (long)F.columns.size(), "fixed column");
    require(u.z == Q("20/3"), "fixed column: z = 20/3");
    // A basis of the first 3 columns (singletons) needs exact pivots on the full master.
    xlp::Solution p = backend_then_exact(M, nearest(M), 3, "prefix of 3 columns");
    require(p.z == Q("17/3") && p.pivots >= 1, "prefix of 3 columns: z = 17/3 after pivots");
    std::cout << "small masters: ok\n";
}

std::string read_file(const std::string &path) {
    std::ifstream in(path);
    require((bool)in, "cannot read " + path);
    std::stringstream s;
    s << in.rdbuf();
    return s.str();
}

// Real root masters, listed in fixtures.txt with the name, the recorded exact value and an optional column count K.
// The backend basis on the full master and, when K is given, the backend basis of the first K columns.
void fixtures() {
    const std::string dir = KAYROS_EXACT_LP_DATA;
    std::istringstream list(read_file(dir + "/fixtures.txt"));
    std::string line, name, z;
    while (std::getline(list, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream fields(line);
        long prefix = -1;
        fields >> name >> z >> prefix;
        std::istringstream text(read_file(dir + "/" + name + ".master"));
        xlp::MasterFile file = xlp::read_master(text);
        const long N = (long)file.master.columns.size();
        for (long count : {N, prefix}) {
            if (count < 0) continue;
            xlp::Solution s = backend_then_exact(file.master, file.binary64_cost, count, name);
            require(s.z == Q(z.c_str()), name + ": z equals the recorded value");
            std::cout << "fixture " << name << ", backend on " << count << " of " << N << " columns: " << s.pivots
                      << " pivots, ok\n";
        }
    }
}
}  // namespace

int main() {
    no_basis_unless_requested();
    small_masters();
    fixtures();
    std::cout << "lp basis: ok\n";
    return 0;
}
