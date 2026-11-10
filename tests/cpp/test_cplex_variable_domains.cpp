#include <bit>
#include <cstdint>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "goc/lib/json.hpp"
#include "goc/linear_programming/cplex/cplex_formulation.h"
#include "goc/linear_programming/cplex/cplex_solver.h"
#include "../../cpp/lera/goc/src/linear_programming/cplex/cplex_variable_domains.h"

using goc::CplexFormulation;
using goc::VariableDomain;
using nlohmann::json;
namespace cpx = goc::cplex;

namespace
{
void require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

json bits(const std::vector<double>& values)
{
    json result = json::array();
    for (double value : values) result.push_back(std::bit_cast<uint64_t>(value));
    return result;
}

void assert_same_state(const json& expected, const json& actual, const std::string& context)
{
    if (expected != actual)
        throw std::runtime_error(context + "\nlegacy=" + expected.dump() + "\ncandidate=" + actual.dump());
}

// These are the original snapshot and restoration loops from best327e481.
class LegacyDomains
{
public:
    explicit LegacyDomains(CplexFormulation* formulation) : domains_(formulation->VariableCount())
    {
        for (int i = 0; i < formulation->VariableCount(); ++i)
            domains_[i] = formulation->GetVariableDomain(formulation->VariableAtIndex(i));
    }

    void Restore(CplexFormulation* formulation) const
    {
        for (int i = 0; i < formulation->VariableCount(); ++i)
            formulation->SetVariableDomain(formulation->VariableAtIndex(i), domains_[i]);
    }

private:
    std::vector<VariableDomain> domains_;
};

void configure(CplexFormulation& model)
{
    auto env = model.Environment();
    cpx::setdefaults(env);
    cpx::setintparam(env, CPX_PARAM_THREADS, 1);
    cpx::setdblparam(env, CPX_PARAM_EPRHS, 1e-7);
    cpx::setdblparam(env, CPX_PARAM_EPOPT, 1e-7);
    cpx::setdblparam(env, CPX_PARAM_EPINT, 1e-6);
    cpx::setdblparam(env, CPX_PARAM_EPGAP, 0.0);
    cpx::setdblparam(env, CPX_PARAM_TILIM, 30.0);
    cpx::setintparam(env, CPX_PARAM_REDUCE, CPX_PREREDUCE_NOPRIMALORDUAL);
}

json model_state(CplexFormulation& model)
{
    auto env = model.Environment();
    auto lp = model.Problem();
    const int count = model.VariableCount();
    json result = {{"problem_type", cpx::getprobtype(env, lp)}, {"columns", count}};
    if (count)
    {
        std::vector<char> types(count);
        std::vector<double> lower(count), upper(count);
        int status = CPXgetctype(env, lp, types.data(), 0, count - 1);
        result["ctype_status"] = status;
        if (!status) result["types"] = std::string(types.begin(), types.end());
        cpx::getlb(env, lp, lower.data(), 0, count - 1);
        cpx::getub(env, lp, upper.data(), 0, count - 1);
        result["lower_bits"] = bits(lower);
        result["upper_bits"] = bits(upper);
    }
    return result;
}

// Raw query status is part of the observation, including unavailable bases or solutions after a type change.
json solution_state(CplexFormulation& model)
{
    auto env = model.Environment();
    auto lp = model.Problem();
    const int columns = model.VariableCount(), rows = model.ConstraintCount();
    json result = {{"status", cpx::getstat(env, lp)}};
    double objective = 0.0;
    int status = CPXgetobjval(env, lp, &objective);
    result["objective_status"] = status;
    if (!status) result["objective_bits"] = std::bit_cast<uint64_t>(objective);
    if (columns)
    {
        std::vector<double> primal(columns), reduced(columns);
        status = CPXgetx(env, lp, primal.data(), 0, columns - 1);
        result["primal_status"] = status;
        if (!status) result["primal_bits"] = bits(primal);
        status = CPXgetdj(env, lp, reduced.data(), 0, columns - 1);
        result["reduced_cost_status"] = status;
        if (!status) result["reduced_cost_bits"] = bits(reduced);
    }
    if (rows)
    {
        std::vector<double> duals(rows);
        status = CPXgetpi(env, lp, duals.data(), 0, rows - 1);
        result["dual_status"] = status;
        if (!status) result["dual_bits"] = bits(duals);
    }
    std::vector<int> column_basis(columns), row_basis(rows);
    status = CPXgetbase(env, lp, columns ? column_basis.data() : nullptr, rows ? row_basis.data() : nullptr);
    result["basis_status"] = status;
    if (!status)
    {
        result["column_basis"] = column_basis;
        result["row_basis"] = row_basis;
    }
    return result;
}

struct Case
{
    std::string name;
    std::vector<char> types;
    std::vector<double> lower, upper, objective;
    bool constrained = false;
    bool batch = true;
    int problem_type = CPXPROB_MILP;
};

std::unique_ptr<CplexFormulation> make_model(const Case& test)
{
    auto model = std::make_unique<CplexFormulation>();
    goc::Expression objective, capacity;
    for (int i = 0; i < static_cast<int>(test.types.size()); ++i)
    {
        const char type = test.types[i];
        auto domain = type == 'B' ? VariableDomain::Binary :
            type == 'I' ? VariableDomain::Integer : VariableDomain::Real;
        auto variable = model->AddVariable("x" + std::to_string(i), domain, test.lower[i], test.upper[i]);
        if (type == 'S' || type == 'N') cpx::chgctype(model->Environment(), model->Problem(), 1, &i, &type);
        objective += test.objective[i] * variable;
        if (test.constrained) capacity += (type == 'B' ? 3.0 : type == 'I' ? 1.0 : 2.0) * variable;
    }
    if (test.types.empty()) cpx::chgobjsen(model->Environment(), model->Problem(), CPX_MAX);
    else model->Maximize(objective);
    if (test.constrained) model->AddConstraint(capacity.LEQ(7.5));
    cpx::chgprobtype(model->Environment(), model->Problem(), test.problem_type);
    configure(*model);
    return model;
}

template<class Domains>
json cycle(CplexFormulation& model, bool optimize)
{
    Domains domains(&model);
    cpx::chgprobtype(model.Environment(), model.Problem(), CPXPROB_LP);
    if (optimize) cpx::lpopt(model.Environment(), model.Problem());
    json result = {{"relaxed_model", model_state(model)}};
    if (optimize) result["lp"] = solution_state(model);
    domains.Restore(&model);
    result["restored_model"] = model_state(model);
    if (optimize) result["after_restore"] = solution_state(model);
    return result;
}

void check_case(const Case& test, json& records)
{
    auto legacy = make_model(test), candidate = make_model(test);
    const json original = model_state(*legacy);
    assert_same_state(original, model_state(*candidate), test.name + " initial state");
    cpx::detail::VariableDomains selector(candidate.get());
    require(selector.UsesBatch() == test.batch, test.name + " wrong batching selection");
    json rounds = json::array();
    for (int round = 0; round < 3; ++round)
    {
        auto old_result = cycle<LegacyDomains>(*legacy, !test.types.empty());
        auto new_result = cycle<cpx::detail::VariableDomains>(*candidate, !test.types.empty());
        assert_same_state(old_result, new_result, test.name + " LP cycle " + std::to_string(round));
        if (test.batch) assert_same_state(original, model_state(*candidate), test.name + " changed types or bounds");
        rounds.push_back(new_result);
    }
    json mip = nullptr;
    if (cpx::getprobtype(candidate->Environment(), candidate->Problem()) == CPXPROB_MILP)
    {
        cpx::mipopt(legacy->Environment(), legacy->Problem());
        cpx::mipopt(candidate->Environment(), candidate->Problem());
        mip = solution_state(*candidate);
        assert_same_state(solution_state(*legacy), mip, test.name + " MIP following 3 LP solves");
        assert_same_state(model_state(*legacy), model_state(*candidate), test.name + " MIP model state");
        if (test.name == "mixed-CIB")
        {
            require(rounds[0]["lp"]["status"] == CPX_STAT_OPTIMAL, "mixed LP not optimal");
            require(rounds[0]["lp"]["objective_bits"] == std::bit_cast<uint64_t>(15.0), "wrong mixed LP objective");
            require(mip["status"] == CPXMIP_OPTIMAL, "mixed MIP not optimal");
            require(mip["objective_bits"] == std::bit_cast<uint64_t>(14.75), "wrong mixed MIP objective");
        }
    }
    records.push_back({{"case", test.name}, {"batch", test.batch}, {"original", original}, {"rounds", rounds}, {"mip", mip}});
}

template<class Callback>
std::string exception_text(Callback callback)
{
    try { callback(); }
    catch (const std::runtime_error& error) { return error.what(); }
    return "";
}

void check_errors(const Case& test, json& records)
{
    auto legacy = make_model(test), candidate = make_model(test);
    cpx::chgprobtype(legacy->Environment(), legacy->Problem(), CPXPROB_LP);
    cpx::chgprobtype(candidate->Environment(), candidate->Problem(), CPXPROB_LP);
    auto old_error = exception_text([&] { LegacyDomains snapshot(legacy.get()); });
    auto new_error = exception_text([&] { cpx::detail::VariableDomains snapshot(candidate.get()); });
    require(old_error == new_error, "snapshot exception text changed");
    assert_same_state(model_state(*legacy), model_state(*candidate), "snapshot exception mutated state");
    records.push_back({{"case", "explicit-LP-error"}, {"error", new_error}});

    legacy = make_model(test);
    candidate = make_model(test);
    LegacyDomains old_snapshot(legacy.get());
    cpx::detail::VariableDomains new_snapshot(candidate.get());
    for (auto* model : {legacy.get(), candidate.get()})
    {
        cpx::mipopt(model->Environment(), model->Problem());
        cpx::chgprobtype(model->Environment(), model->Problem(), CPXPROB_FIXEDMILP);
    }
    old_error = exception_text([&] { old_snapshot.Restore(legacy.get()); });
    new_error = exception_text([&] { new_snapshot.Restore(candidate.get()); });
    require(!old_error.empty(), "expected CPXchgctype error on a fixed MILP");
    require(old_error == new_error, "restore exception text changed");
    assert_same_state(model_state(*legacy), model_state(*candidate), "restore exception state differs");
    records.push_back({{"case", "fixed-MILP-restore-error"}, {"error", new_error}});
}

void check_solver_integration(const Case& test, json& records)
{
    auto legacy = make_model(test), candidate = make_model(test);
    auto expected = cycle<LegacyDomains>(*legacy, true);
    auto result = cpx::solve_lp(candidate.get(), nullptr, goc::Duration(30, goc::DurationUnit::Seconds),
                                json::object(), {goc::LPOption::Incumbent, goc::LPOption::Duals});
    require(result.status.IsSet() && result.status.Value() == goc::LPStatus::Optimum, "production LP status");
    require(result.incumbent_value.IsSet(), "production LP value missing");
    require(std::bit_cast<uint64_t>(result.incumbent_value.Value()) ==
                expected["lp"]["objective_bits"].get<uint64_t>(), "production LP value differs");
    require(result.incumbent.IsSet() && result.duals.IsSet(), "production primal or dual missing");
    std::vector<double> primal, expected_primal;
    goc::Valuation old_incumbent;
    for (int i = 0; i < candidate->VariableCount(); ++i)
        old_incumbent.SetValue(candidate->VariableAtIndex(i),
            std::bit_cast<double>(expected["lp"]["primal_bits"][i].get<uint64_t>()));
    for (int i = 0; i < candidate->VariableCount(); ++i)
    {
        primal.push_back(result.incumbent.Value()[candidate->VariableAtIndex(i)]);
        expected_primal.push_back(old_incumbent[candidate->VariableAtIndex(i)]);
    }
    assert_same_state(bits(expected_primal), bits(primal), "production primal differs");
    assert_same_state(expected["lp"]["dual_bits"], bits(result.duals.Value()), "production dual differs");
    assert_same_state(expected["restored_model"], model_state(*candidate), "production restored model differs");
    assert_same_state(expected["after_restore"], solution_state(*candidate), "production post-restore state differs");
    cpx::mipopt(legacy->Environment(), legacy->Problem());
    cpx::mipopt(candidate->Environment(), candidate->Problem());
    assert_same_state(solution_state(*legacy), solution_state(*candidate), "production LP then MIP differs");
    records.push_back({{"case", "production-solve-lp"}, {"passed", true}});
}

class DerivedFormulation : public CplexFormulation
{
public:
    mutable int reads = 0;
    int writes = 0;
    VariableDomain GetVariableDomain(const goc::Variable& variable) const override
    {
        ++reads;
        return CplexFormulation::GetVariableDomain(variable);
    }
    void SetVariableDomain(const goc::Variable& variable, VariableDomain domain) override
    {
        ++writes;
        CplexFormulation::SetVariableDomain(variable, domain);
    }
};

void check_virtual_accessors(json& records)
{
    DerivedFormulation model;
    model.AddVariable("x", VariableDomain::Binary, 0.0, 1.0);
    model.AddVariable("y", VariableDomain::Integer, -2.0, 3.0);
    model.reads = model.writes = 0;
    cpx::detail::VariableDomains snapshot(&model);
    require(!snapshot.UsesBatch() && model.reads == 2, "virtual domain reads bypassed");
    cpx::chgprobtype(model.Environment(), model.Problem(), CPXPROB_LP);
    snapshot.Restore(&model);
    require(model.writes == 2, "virtual domain writes bypassed");
    records.push_back({{"case", "derived-formulation"}, {"reads", model.reads}, {"writes", model.writes}});
}
}

int main()
{
    json records = json::array();
    try
    {
        const Case mixed{"mixed-CIB", {'C', 'I', 'B'}, {0, 0, 0}, {4.25, 5, 1}, {3, 2, 6}, true};
        const std::vector<Case> cases{
            mixed,
            {"mixed-BIC", {'B', 'I', 'C'}, {0, 0, 0}, {1, 5, 4.25}, {6, 2, 3}, true},
            {"mixed-IBC", {'I', 'B', 'C'}, {0, 0, 0}, {5, 1, 4.25}, {2, 6, 3}, true},
            {"continuous-MILP", {'C', 'C', 'C'}, {-1.5, -0.0, 0.125}, {2.25, 4.5, 0.125}, {1, 2, 3}},
            {"fixed-binaries", {'B', 'B', 'I', 'C'}, {0, 1, -3, -1.25}, {0, 1, -1, 2.5}, {4, 3, 2, 1}},
            {"fractional-binary-bounds", {'B', 'C'}, {0.25, -1}, {0.75, 2}, {1, 2}},
            {"wide-binary-bounds", {'B', 'C'}, {-1, -1}, {2, 2}, {1, 2}},
            {"infinite-continuous-bounds", {'C', 'I'}, {-CPX_INFBOUND, -2}, {CPX_INFBOUND, 3}, {0, 1}},
            {"one-integer", {'I'}, {-3}, {2.75}, {1}, false, false},
            {"empty-LP", {}, {}, {}, {}, false, false, CPXPROB_LP},
            {"empty-MILP", {}, {}, {}, {}, false, false},
            {"semitype-fallback", {'S', 'N', 'B'}, {2, 2, 0}, {5, 5, 1}, {1, 2, 3}, false, false},
            {"empty-quadratic-MIQP-fallback", {'I', 'C'}, {0, 0}, {2.5, 3.25}, {1, 2}, false, false, CPXPROB_MIQP},
        };
        for (const auto& test : cases) check_case(test, records);
        check_errors(mixed, records);
        check_solver_integration(mixed, records);
        check_virtual_accessors(records);
        CplexFormulation version_model;
        std::cout << json({{"passed", true}, {"cplex_version", CPXversion(version_model.Environment())},
                           {"cases", records}, {"timing_claim", false}}).dump(2) << '\n';
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cout << json({{"passed", false}, {"error", error.what()}, {"completed_cases", records}}).dump(2) << '\n';
        return 1;
    }
}
