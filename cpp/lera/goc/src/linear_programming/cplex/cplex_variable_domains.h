#ifndef GOC_CPLEX_VARIABLE_DOMAINS_H
#define GOC_CPLEX_VARIABLE_DOMAINS_H

#include <algorithm>
#include <numeric>
#include <typeinfo>
#include <vector>

#include "goc/linear_programming/cplex/cplex_formulation.h"

namespace goc::cplex::detail
{
// Keep this snapshot local to one solve_lp call. Restoration is explicit so an exception follows the existing path, without an extra CPLEX mutation on unwind.
class VariableDomains
{
public:
    explicit VariableDomains(CplexFormulation* formulation)
    {
        const int count = formulation->VariableCount();
        // Preserve virtual domain accessors and uncommon problem types verbatim.
        if (count > 1 && typeid(*formulation) == typeid(CplexFormulation) &&
            cplex::getprobtype(formulation->Environment(), formulation->Problem()) == CPXPROB_MILP)
        {
            types_.resize(count);
            cplex::getctype(formulation->Environment(), formulation->Problem(), types_.data(), 0, count - 1);
            if (std::all_of(types_.begin(), types_.end(), [](char type) {
                    return type == 'C' || type == 'I' || type == 'B';
                }))
            {
                indices_.resize(count - 1);
                std::iota(indices_.begin(), indices_.end(), 1);
                first_domain_ = types_[0] == 'B' ? VariableDomain::Binary :
                    types_[0] == 'I' ? VariableDomain::Integer : VariableDomain::Real;
                batched_ = true;
                return;
            }
        }

        domains_.resize(count);
        for (int i = 0; i < formulation->VariableCount(); ++i)
            domains_[i] = formulation->GetVariableDomain(formulation->VariableAtIndex(i));
    }

    void Restore(CplexFormulation* formulation) const
    {
        if (batched_)
        {
            // Keep the original first mutation, including its LP-to-MILP transition. All subsequent updates target that same MILP object.
            formulation->SetVariableDomain(formulation->VariableAtIndex(0), first_domain_);
            cplex::chgctype(formulation->Environment(), formulation->Problem(),
                           static_cast<int>(indices_.size()), indices_.data(), types_.data() + 1);
            return;
        }
        for (int i = 0; i < formulation->VariableCount(); ++i)
            formulation->SetVariableDomain(formulation->VariableAtIndex(i), domains_[i]);
    }

    bool UsesBatch() const { return batched_; }

private:
    bool batched_ = false;
    VariableDomain first_domain_ = VariableDomain::Real;
    std::vector<char> types_;
    std::vector<int> indices_;
    std::vector<VariableDomain> domains_;
};
}

#endif
