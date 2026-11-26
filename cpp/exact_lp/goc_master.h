// Bridge between a master of kayros::exact_lp and the goc LP layer of the Lera
// BPC, for the driver and the backend test. Only for targets that link lera.
#pragma once

#include <goc/goc.h>

#include <stdexcept>
#include <string>
#include <vector>

#include "exact_lp/exact_lp.h"

namespace kayros::exact_lp {

// The formulation of the first `count` columns of the master, built as SPF
// builds the restricted master: minimisation, rows "=" or "<=", columns Binary
// on [0, inf) or [0, 0] when fixed, with the binary64 costs. The caller owns it.
inline goc::Formulation *to_formulation(const Master &M, const std::vector<double> &binary64_cost, long count) {
    goc::Formulation *f = goc::LPSolver::NewFormulation();
    f->Minimize(goc::Expression());
    for (int i = 0; i < M.rows(); ++i) {
        const double rhs = M.rhs[i].get_d();
        if (mpq_class(rhs) != M.rhs[i]) throw std::invalid_argument("right-hand side not a binary64");
        f->AddConstraint(M.sense[i] == Sense::Equal ? goc::Expression().EQ(rhs) : goc::Expression().LEQ(rhs));
    }
    for (long j = 0; j < count; ++j) {
        const Column &c = M.columns[j];
        goc::Variable y = f->AddVariable("y_" + std::to_string(j), goc::VariableDomain::Binary, 0.0, c.fixed_at_zero ? 0.0 : goc::INFTY);
        for (const Entry &e : c.entries) f->SetConstraintCoefficient(e.row, y, (double)e.coefficient);
        f->SetObjectiveCoefficient(y, binary64_cost[j]);
    }
    return f;
}

// The basis of the backend in the ids of exact_lp.
inline Basis to_basis(const goc::LPBasis &b) {
    Basis B(b.columns.begin(), b.columns.end());
    for (int i : b.rows) B.push_back(logical(i));
    return B;
}

}  // namespace kayros::exact_lp
