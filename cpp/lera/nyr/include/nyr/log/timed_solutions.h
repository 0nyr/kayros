#pragma once

#include <vector>
#include <iostream>
#include <concepts>
#include <ranges>
#include <string>
#include <memory>

#include <goc/goc.h>
#include "nyr/time/time.h"
#include "nyr/solutions/vrp_solution.h"
#include "nyr/solutions/route.h"
#include "nyr/solutions/objectives.h"

namespace nyr
{

class AbstractSolutionRecord: public goc::Log
{
public:
    virtual ~AbstractSolutionRecord() = default;

    // Add a solution
    virtual void add(nyr::Durex time, std::unique_ptr<AbstractSolution> sol, const std::string& origin) = 0;

    // Try to add (only if better)
    virtual void try_add(std::unique_ptr<AbstractSolution> sol, const std::string& origin) = 0;

    // Last solution value
    virtual double last_solution_value() const = 0;

    // Empty?
    virtual bool empty() const = 0;

    // Get last solution
    virtual const AbstractSolution& last_solution() const = 0;

    // Export all solutions to JSON
    virtual nlohmann::json ToJSON() const = 0;
};

/** A record entry for a solution found by the solver.
 * Contains useful information about the solution:
 * - The time at which the solution was found.
 * - The solution itself.
 * - The origin or source of the solution (e.g. "GMH1", "BPCA", ...).
 */
template<std::derived_from<AbstractSolution> Solution>
struct AnnotatedSolution
{
    Durex time;         // Time since the algorithm started.
    Solution solution;  // The solution found.
    std::string origin; // The origin or source of the solution.
};

// This class stores all the Upper Bound solutions 
// found by a solver at increasing times and quality.
// It is used to log the solutions found by the solver.
// The solution type must be serializable to JSON.
template<std::derived_from<AbstractSolution> Solution>
class SolutionRecord: public AbstractSolutionRecord
{
public:
    const ProgramClock& pclock; // Program clock to measure time.

    SolutionRecord(const nyr::ProgramClock& pclock)
        : pclock(pclock)
    {}

    /**
     * Adds a new solution to the list of solutions.
     */
    void add(nyr::Durex time, std::unique_ptr<AbstractSolution> sol, const std::string& origin) override
    {
        auto* derived_sol = dynamic_cast<Solution*>(sol.get());
        if (!derived_sol)
        {
            throw std::invalid_argument("Invalid solution type.");
        }
        add(time, *derived_sol, origin);
    }

    void add(nyr::Durex time, const Solution& sol, const std::string& origin)
    {
        sol_records_.push_back({time, sol, origin});

        std::clog << "✨[" << origin << "]> Solution: "
            << "nb routes: " << sol.routes.size() << ", "
            << "value: " << sol.value << ", "
            << "routes: " << sol.routes
            << std::endl;
    }

    /**
     * Try to add a new solution to the list of solutions.
     * Only adds the solution if it is not the same as the last
     * solution added, and if its value is smaller than the 
     * last solution added (minimization problem). 
     */
    void try_add(std::unique_ptr<AbstractSolution> sol, const std::string& origin) override
    {
        auto* derived_sol = dynamic_cast<Solution*>(sol.get());
        if (!derived_sol)
        {
            throw std::invalid_argument("Invalid solution type.");
        }
        try_add(*derived_sol, origin);
    }

    void try_add(const Solution& sol, const std::string& origin)
    {
        // Check that the solution to add is not the same as the last 
        // solution added, and if its value is smaller than the last solution added.
        if (
            !sol_records_.empty() &&
            (sol == sol_records_.back().solution ||
             sol.value >= sol_records_.back().solution.value)
        )
            return;

        // Get the time since the program started.
        add(pclock.elapsed(), sol, origin);
    }

    // Check if there are any solutions.
    bool empty() const override
    {
        return sol_records_.empty();
    }

    // Returns the last (best) solution found.
    const AbstractSolution& last_solution() const override
    {
        return sol_records_.back().solution;
    }

    // Returns the last (best) solution value found
    // or INFTY if no solution was found.
    double last_solution_value() const override
    {
        return sol_records_.empty() ? goc::INFTY : sol_records_.back().solution.value;
    }

    // Serializes the object to JSON.
    // Format: [{"time": time, "solution": solution}, ...]
    nlohmann::json ToJSON() const override
    {
        nlohmann::json j;
        std::vector<nlohmann::json> j_sols;
        j_sols.reserve(sol_records_.size());
        for (auto& annotated_sol : sol_records_)
        {
            j_sols.push_back({
                {"time", annotated_sol.time},
                {"origin", annotated_sol.origin},
                {"solution", annotated_sol.solution}
            });
        }
        j = j_sols;
        return j;
    }

private:
    // List of solutions with the time they were found, in increasing order of time.
    std::vector<AnnotatedSolution<Solution>> sol_records_;
};

// std::unique_ptr<AbstractSolutionRecord> create_solution_record(
//     const ProgramClock& pclock,
//     ObjectiveFunction objective
// ) {
//     switch (objective)
//     {
//         case ObjectiveFunction::Duration:
//             return std::make_unique<SolutionRecord<VRPSolutionDuration>>(pclock);
//         case ObjectiveFunction::Makespan:
//             return std::make_unique<SolutionRecord<VRPSolutionMakespan>>(pclock);
//         case ObjectiveFunction::TravelTime:
//             return std::make_unique<SolutionRecord<VRPSolutionTravelTime>>(pclock);
//         default:
//             throw std::invalid_argument("Unknown objective function.");
//     }
// }

template<typename Solution>
void print_last_solution(
    const nyr::SolutionRecord<Solution>& rec,
    const nyr::ObjectiveFunction objective
)
{
    if (rec.empty())
    {
        std::clog << "No solution found." << std::endl;
    }
    else
    {
        const auto& best_solution = rec.last_solution();
        std::clog << "Best solution:" << std::endl;
        std::clog << "\tObjective: " << objective << std::endl;
        std::clog << "\tValue: " << best_solution.value << std::endl;
        try_print_routes<Solution>(best_solution);
    }
}

} // namespace nyr