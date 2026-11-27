// Exact LP multipliers of the restricted master. See exact_lp.h.
#include "exact_lp/exact_lp.h"

#include <algorithm>
#include <cstdlib>
#include <istream>
#include <sstream>
#include <stdexcept>

namespace kayros::exact_lp {
namespace {

// Dense LU of the basis matrix with row pivoting, in rationals: P B = L U, L
// with unit diagonal below the diagonal of a, U on and above it. The entries of
// B are small integers and the master is small (rows: customers and cuts), so
// a dense factorisation from scratch at every pivot is enough.
struct LU {
    int m = 0;
    std::vector<mpq_class> a;
    std::vector<int> perm;  // row perm[k] of B is row k of P B
    mpq_class &at(int i, int j) { return a[(size_t)i * m + j]; }
};

bool is_fixed(const Master &M, int id) {
    return id >= 0 ? M.columns[id].fixed_at_zero : M.sense[-1 - id] == Sense::Equal;
}

mpq_class cost_of(const Master &M, int id) { return id >= 0 ? M.columns[id].cost : mpq_class(0); }

// Order of the ties, columns by index and then logicals by row.
long key(const Master &M, int id) { return id >= 0 ? id : (long)M.columns.size() + (-1 - id); }

std::vector<mpq_class> dense_column(const Master &M, int id) {
    std::vector<mpq_class> col(M.rows(), mpq_class(0));
    if (id >= 0)
        for (const Entry &e : M.columns[id].entries) col[e.row] = e.coefficient;
    else
        col[-1 - id] = 1;
    return col;
}

// Returns false when B is singular.
bool factor(LU &F, const Master &M, const Basis &B) {
    const int m = M.rows();
    F.m = m;
    F.a.assign((size_t)m * m, mpq_class(0));
    F.perm.resize(m);
    for (int k = 0; k < m; ++k) {
        if (B[k] >= 0)
            for (const Entry &e : M.columns[B[k]].entries) F.at(e.row, k) = e.coefficient;
        else
            F.at(-1 - B[k], k) = 1;
    }
    for (int i = 0; i < m; ++i) F.perm[i] = i;
    for (int c = 0; c < m; ++c) {
        int p = -1;
        for (int r = c; r < m; ++r)
            if (sgn(F.at(r, c)) != 0) {
                p = r;
                break;
            }
        if (p < 0) return false;
        if (p != c) {
            for (int k = 0; k < m; ++k) std::swap(F.at(p, k), F.at(c, k));
            std::swap(F.perm[p], F.perm[c]);
        }
        const mpq_class pivot = F.at(c, c);
        for (int r = c + 1; r < m; ++r) {
            if (sgn(F.at(r, c)) == 0) continue;
            mpq_class f = F.at(r, c) / pivot;
            F.at(r, c) = f;
            for (int k = c + 1; k < m; ++k)
                if (sgn(F.at(c, k)) != 0) F.at(r, k) -= f * F.at(c, k);
        }
    }
    return true;
}

// B x = b.
std::vector<mpq_class> solve(LU &F, const std::vector<mpq_class> &b) {
    const int m = F.m;
    std::vector<mpq_class> y(m), x(m);
    for (int i = 0; i < m; ++i) {
        mpq_class s = b[F.perm[i]];
        for (int k = 0; k < i; ++k)
            if (sgn(F.at(i, k)) != 0 && sgn(y[k]) != 0) s -= F.at(i, k) * y[k];
        y[i] = s;
    }
    for (int i = m - 1; i >= 0; --i) {
        mpq_class s = y[i];
        for (int k = i + 1; k < m; ++k)
            if (sgn(F.at(i, k)) != 0 && sgn(x[k]) != 0) s -= F.at(i, k) * x[k];
        x[i] = s / F.at(i, i);
    }
    return x;
}

// B^T y = c: U^T w = c, L^T v = w, y[perm[i]] = v[i].
std::vector<mpq_class> solve_transposed(LU &F, const std::vector<mpq_class> &c) {
    const int m = F.m;
    std::vector<mpq_class> w(m), v(m), y(m);
    for (int i = 0; i < m; ++i) {
        mpq_class s = c[i];
        for (int k = 0; k < i; ++k)
            if (sgn(F.at(k, i)) != 0 && sgn(w[k]) != 0) s -= F.at(k, i) * w[k];
        w[i] = s / F.at(i, i);
    }
    for (int i = m - 1; i >= 0; --i) {
        mpq_class s = w[i];
        for (int k = i + 1; k < m; ++k)
            if (sgn(F.at(k, i)) != 0 && sgn(v[k]) != 0) s -= F.at(k, i) * v[k];
        v[i] = s;
    }
    for (int i = 0; i < m; ++i) y[F.perm[i]] = v[i];
    return y;
}

// The multipliers scaled to integers: Y_i = y_i D with D the least common
// multiple of their denominators. The reduced cost of column j is then
// T_j / (den(c_j) D) with T_j = num(c_j) D - den(c_j) sum_i a_ij Y_i, so its sign
// and the order of 2 reduced costs need integer arithmetic only.
struct ScaledDuals {
    mpz_class D = 1;
    std::vector<mpz_class> Y;
    explicit ScaledDuals(const std::vector<mpq_class> &y) : Y(y.size()) {
        for (const mpq_class &v : y) mpz_lcm(D.get_mpz_t(), D.get_mpz_t(), v.get_den_mpz_t());
        for (size_t i = 0; i < y.size(); ++i) Y[i] = y[i].get_num() * (D / y[i].get_den());
    }
    // T_j of the reduced cost of column c.
    mpz_class numerator(const Column &c) const {
        mpz_class s = 0;
        for (const Entry &e : c.entries) s += e.coefficient * Y[e.row];
        return c.cost.get_num() * D - c.cost.get_den() * s;
    }
};

Solution unresolved(Solution s, Reason r) {
    s.status = Status::Unresolved;
    s.reason = r;
    s.duals.clear();
    s.primal.clear();
    s.z = 0;
    return s;
}

}  // namespace

Solution solve_from_basis(const Master &M, const Basis &start, int pivot_budget) {
    const int m = M.rows();
    const int ncols = (int)M.columns.size();
    Solution out;
    out.basis = start;
    Basis &B = out.basis;
    if ((int)B.size() != m) return unresolved(out, Reason::InvalidBasis);
    std::vector<char> in_basis(ncols + m, 0);
    for (int id : B) {
        if (id >= ncols || id < -m || in_basis[key(M, id)]) return unresolved(out, Reason::InvalidBasis);
        in_basis[key(M, id)] = 1;
    }
    LU F;
    std::vector<mpq_class> x, y;
    for (;;) {
        if (!factor(F, M, B)) return unresolved(out, Reason::Singular);
        x = solve(F, M.rhs);
        for (int k = 0; k < m; ++k)
            if (is_fixed(M, B[k]) ? sgn(x[k]) != 0 : sgn(x[k]) < 0) return unresolved(out, Reason::PrimalInfeasible);
        std::vector<mpq_class> cB(m);
        for (int k = 0; k < m; ++k) cB[k] = cost_of(M, B[k]);
        y = solve_transposed(F, cB);

        // The entering variable has the most negative reduced cost, compared as
        // T / den(cost) since D is common to all candidates.
        ScaledDuals S(y);
        int enter = 0;
        bool found = false;
        mpq_class best;
        for (int j = 0; j < ncols; ++j) {
            if (in_basis[j] || M.columns[j].fixed_at_zero) continue;
            mpz_class T = S.numerator(M.columns[j]);
            if (sgn(T) >= 0) continue;
            mpq_class d(T, M.columns[j].cost.get_den());
            d.canonicalize();
            if (!found || d < best) found = true, enter = j, best = d;
        }
        for (int i = 0; i < m; ++i) {
            if (M.sense[i] != Sense::LessEqual || in_basis[key(M, logical(i))] || sgn(S.Y[i]) <= 0) continue;
            mpq_class d(-S.Y[i]);  // reduced cost of the slack: -y_i
            if (!found || d < best) found = true, enter = logical(i), best = d;
        }
        if (!found) break;
        if (out.pivots >= pivot_budget) return unresolved(out, Reason::PivotBudget);

        std::vector<mpq_class> u = solve(F, dense_column(M, enter));
        int leave = -1;
        mpq_class best_ratio;
        for (int k = 0; k < m; ++k) {
            const bool fixed = is_fixed(M, B[k]);
            if (fixed ? sgn(u[k]) == 0 : sgn(u[k]) <= 0) continue;
            mpq_class ratio = fixed ? mpq_class(0) : mpq_class(x[k] / u[k]);
            if (leave < 0 || ratio < best_ratio || (ratio == best_ratio && key(M, B[k]) < key(M, B[leave])))
                leave = k, best_ratio = ratio;
        }
        if (leave < 0) return unresolved(out, Reason::Unbounded);
        if (sgn(best_ratio) == 0) ++out.degenerate_pivots;
        in_basis[key(M, B[leave])] = 0;
        in_basis[key(M, enter)] = 1;
        B[leave] = enter;
        ++out.pivots;
    }

    out.status = Status::Optimal;
    out.reason = Reason::None;
    out.z = 0;
    for (int k = 0; k < m; ++k) {
        out.z += cost_of(M, B[k]) * x[k];
        if (B[k] >= 0 && sgn(x[k]) != 0) out.primal.emplace_back(B[k], x[k]);
    }
    std::sort(out.primal.begin(), out.primal.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
    out.duals = std::move(y);
    if (!verify(M, out).empty()) return unresolved(out, Reason::VerificationFailed);
    return out;
}

std::string verify(const Master &M, const Solution &s) {
    const int m = M.rows();
    const int ncols = (int)M.columns.size();
    if (s.status != Status::Optimal) return "status is not optimal";
    if ((int)s.duals.size() != m) return "one multiplier per row";
    std::vector<mpq_class> activity(m, mpq_class(0));
    mpq_class primal_value = 0;
    int previous = -1;
    for (const auto &[j, v] : s.primal) {
        if (j <= previous || j >= ncols) return "primal indices strictly increasing and in range";
        previous = j;
        if (sgn(v) <= 0) return "primal values > 0";
        if (M.columns[j].fixed_at_zero) return "fixed columns at 0";
        for (const Entry &e : M.columns[j].entries) activity[e.row] += e.coefficient * v;
        primal_value += M.columns[j].cost * v;
    }
    mpq_class dual_value = 0;
    for (int i = 0; i < m; ++i) {
        if (M.sense[i] == Sense::Equal ? activity[i] != M.rhs[i] : activity[i] > M.rhs[i]) return "rows satisfied";
        if (M.sense[i] == Sense::LessEqual && sgn(s.duals[i]) > 0) return "multipliers of \"<=\" rows <= 0";
        dual_value += M.rhs[i] * s.duals[i];
    }
    ScaledDuals S(s.duals);
    for (const Column &c : M.columns)
        if (!c.fixed_at_zero && sgn(S.numerator(c)) < 0) return "reduced costs >= 0";
    if (primal_value != s.z) return "primal value equals z";
    if (dual_value != s.z) return "dual value equals z";
    return "";
}

const char *to_string(Reason r) {
    switch (r) {
        case Reason::None: return "none";
        case Reason::InvalidBasis: return "invalid basis";
        case Reason::Singular: return "singular basis";
        case Reason::PrimalInfeasible: return "basis not primal feasible";
        case Reason::Unbounded: return "unbounded";
        case Reason::PivotBudget: return "pivot budget used up";
        case Reason::VerificationFailed: return "verification failed";
    }
    return "?";
}

namespace {
mpq_class parse_rational(const std::string &s) {
    mpq_class q;
    if (s.empty() || q.set_str(s, 10) != 0 || sgn(q.get_den()) == 0) throw std::runtime_error("bad rational: " + s);
    q.canonicalize();
    return q;
}
std::string next_line(std::istream &in, const char *what) {
    std::string line;
    if (!std::getline(in, line)) throw std::runtime_error(std::string("master file ends before ") + what);
    return line;
}
}  // namespace

MasterFile read_master(std::istream &in) {
    MasterFile f;
    Master &M = f.master;
    if (next_line(in, "the header") != "kayros-exact-lp-master 1") throw std::runtime_error("not a master file of version 1");
    std::string word;
    long count = -1;
    std::istringstream rows_line(next_line(in, "rows"));
    if (!(rows_line >> word >> count) || word != "rows" || count < 0) throw std::runtime_error("bad rows line");
    for (long i = 0; i < count; ++i) {
        std::istringstream line(next_line(in, "the rows"));
        std::string sense, rhs;
        if (!(line >> sense >> rhs) || (sense != "E" && sense != "L")) throw std::runtime_error("bad row line");
        M.sense.push_back(sense == "E" ? Sense::Equal : Sense::LessEqual);
        M.rhs.push_back(parse_rational(rhs));
    }
    std::istringstream cols_line(next_line(in, "columns"));
    if (!(cols_line >> word >> count) || word != "columns" || count < 0) throw std::runtime_error("bad columns line");
    for (long j = 0; j < count; ++j) {
        std::istringstream line(next_line(in, "the columns"));
        std::string cost, binary64, upper;
        long k = -1;
        if (!(line >> cost >> binary64 >> upper >> k) || k < 0 || (upper != "inf" && upper != "0"))
            throw std::runtime_error("bad column line");
        Column c;
        c.cost = parse_rational(cost);
        c.fixed_at_zero = upper == "0";
        char *end = nullptr;
        f.binary64_cost.push_back(std::strtod(binary64.c_str(), &end));
        if (end == binary64.c_str() || *end != '\0') throw std::runtime_error("bad binary64 cost: " + binary64);
        for (long e = 0; e < k; ++e) {
            std::string item;
            if (!(line >> item)) throw std::runtime_error("bad column entries");
            size_t colon = item.find(':');
            if (colon == std::string::npos) throw std::runtime_error("bad column entry: " + item);
            Entry entry;
            try {
                size_t used_row = 0, used_coefficient = 0;
                entry.row = std::stoi(item.substr(0, colon), &used_row);
                entry.coefficient = std::stol(item.substr(colon + 1), &used_coefficient);
                if (used_row != colon || used_coefficient != item.size() - colon - 1) throw std::invalid_argument(item);
            } catch (const std::logic_error &) {
                throw std::runtime_error("bad column entry: " + item);
            }
            if (entry.row < 0 || entry.row >= M.rows() || entry.coefficient == 0 ||
                (!c.entries.empty() && entry.row <= c.entries.back().row))
                throw std::runtime_error("bad column entry: " + item);
            c.entries.push_back(entry);
        }
        M.columns.push_back(std::move(c));
    }
    return f;
}

}  // namespace kayros::exact_lp
