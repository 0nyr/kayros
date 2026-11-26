// Driver of the exact LP component on a master file. It solves the master with the
// configured LP backend of the Lera BPC (HiGHS or CPLEX) through goc, as SPF
// builds it (to_formulation of goc_master.h),
// takes the optimal basis of the backend and solves it exactly with
// kayros::exact_lp. Prints 1 JSON line.
//
//   kayros_exact_lp_master MASTER [--budget N] [--prefix K] [--ref Z] [--basis-out FILE]
//
// --prefix K   the backend solves the master restricted to its first K columns,
//              and the exact solve starts from that basis on the full master
//              (a start basis that needs exact pivots).
// --ref Z      a rational, the expected exact value (field z_equals_ref).
// --basis-out  writes the basis ids of the backend (exact_lp::Basis) on 1 line.
#include <goc/goc.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "exact_lp/exact_lp.h"
#include "exact_lp/goc_master.h"

using namespace goc;
namespace xlp = kayros::exact_lp;

namespace {
#ifdef GOC_LP_BACKEND_HIGHS
const char *backend_name = "HiGHS";
#else
const char *backend_name = "CPLEX";
#endif
double seconds_since(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}
size_t bits(const mpq_class &q) {
    return std::max(mpz_sizeinbase(q.get_num_mpz_t(), 2), mpz_sizeinbase(q.get_den_mpz_t(), 2));
}
std::string hex(double v) {
    char buffer[64];
    std::snprintf(buffer, sizeof buffer, "%a", v);
    return buffer;
}
}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "usage: kayros_exact_lp_master MASTER [--budget N] [--prefix K] [--ref Z] [--basis-out FILE]\n";
        return 2;
    }
    int budget = xlp::default_pivot_budget;
    long prefix = -1;
    std::string ref, basis_out;
    for (int a = 2; a + 1 < argc; a += 2) {
        std::string k = argv[a];
        if (k == "--budget") budget = std::stoi(argv[a + 1]);
        else if (k == "--prefix") prefix = std::stol(argv[a + 1]);
        else if (k == "--ref") ref = argv[a + 1];
        else if (k == "--basis-out") basis_out = argv[a + 1];
        else { std::cerr << "unknown option " << k << "\n"; return 2; }
    }
    std::ifstream in(argv[1]);
    if (!in) { std::cerr << "cannot read " << argv[1] << "\n"; return 2; }
    auto t = std::chrono::steady_clock::now();
    xlp::MasterFile file = xlp::read_master(in);
    const xlp::Master &M = file.master;
    const double read_s = seconds_since(t);
    const int m = M.rows();
    const long ncols = (long)M.columns.size();
    const long k = prefix < 0 ? ncols : std::min(prefix, ncols);

    Formulation *formulation = xlp::to_formulation(M, file.binary64_cost, k);
    LPSolver solver;
    t = std::chrono::steady_clock::now();
    LPExecutionLog log = solver.Solve(formulation, {LPOption::Duals, LPOption::Basis});
    const double backend_s = seconds_since(t);

    std::ostringstream o;
    o << "{\"master\":\"" << argv[1] << "\",\"backend\":\"" << backend_name << "\",\"rows\":" << m
      << ",\"columns\":" << ncols << ",\"backend_columns\":" << k << ",\"read_s\":" << read_s
      << ",\"backend_s\":" << backend_s << ",\"backend_status\":\"" << *log.status << "\"";
    if (log.incumbent_value.IsSet()) o << ",\"backend_value\":\"" << hex(*log.incumbent_value) << "\"";
    if (log.simplex_iterations.IsSet()) o << ",\"backend_iterations\":" << *log.simplex_iterations;
    o << ",\"basis_set\":" << (log.basis.IsSet() ? "true" : "false");
    if (log.basis.IsSet()) {
        xlp::Basis B = xlp::to_basis(*log.basis);
        o << ",\"basis_columns\":" << log.basis->columns.size() << ",\"basis_logicals\":" << log.basis->rows.size();
        if (!basis_out.empty()) {
            std::ofstream b(basis_out);
            for (size_t q = 0; q < B.size(); ++q) b << (q ? " " : "") << B[q];
            b << "\n";
        }
        t = std::chrono::steady_clock::now();
        xlp::Solution s = xlp::solve_from_basis(M, B, budget);
        const double exact_s = seconds_since(t);
        const bool optimal = s.status == xlp::Status::Optimal;
        o << ",\"budget\":" << budget << ",\"exact_status\":\"" << (optimal ? "optimal" : "unresolved") << "\",\"reason\":\""
          << xlp::to_string(s.reason) << "\",\"pivots\":" << s.pivots << ",\"degenerate_pivots\":" << s.degenerate_pivots
          << ",\"exact_s\":" << exact_s;
        if (optimal) {
            size_t dual_bits = 0;
            int negative_sigma = 0;
            for (int i = 0; i < m; ++i) {
                dual_bits = std::max(dual_bits, bits(s.duals[i]));
                if (M.sense[i] == xlp::Sense::LessEqual && sgn(s.duals[i]) < 0) ++negative_sigma;
            }
            bool integral = std::all_of(s.primal.begin(), s.primal.end(), [](const auto &p) { return p.second == 1; });
            t = std::chrono::steady_clock::now();
            std::string check = xlp::verify(M, s);
            o << ",\"verify_s\":" << seconds_since(t) << ",\"verify\":\"" << (check.empty() ? "ok" : check) << "\""
              << ",\"z\":\"" << s.z.get_str() << "\",\"z_binary64\":\"" << hex(s.z.get_d()) << "\",\"dual_max_bits\":" << dual_bits
              << ",\"negative_sigma\":" << negative_sigma << ",\"support\":" << s.primal.size()
              << ",\"integral\":" << (integral ? "true" : "false");
            if (!ref.empty()) {
                mpq_class r(ref, 10);
                r.canonicalize();
                o << ",\"z_equals_ref\":" << (r == s.z ? "true" : "false");
            }
        }
    }
    o << "}";
    std::cout << o.str() << std::endl;
    delete formulation;
    return 0;
}
