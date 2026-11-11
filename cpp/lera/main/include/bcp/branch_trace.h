#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace solver
{
// A private first-branch diagnostic. With no environment path this does no IO.
// The file is exclusive and flushed before any strong-branching probe.
class FirstBranchTrace
{
public:
    FirstBranchTrace()
    {
        static bool claimed = false;
        const char* path = std::getenv("KAYROS_FIRST_BRANCH_TRACE");
        if (path == nullptr || *path == '\0' || claimed) return;
        claimed = true;
        stream_ = std::fopen(path, "wx");
        if (stream_ == nullptr) throw std::runtime_error("Cannot exclusively create first-branch trace");
    }

    ~FirstBranchTrace() { if (stream_) std::fclose(stream_); }
    FirstBranchTrace(const FirstBranchTrace&) = delete;
    FirstBranchTrace& operator=(const FirstBranchTrace&) = delete;
    explicit operator bool() const { return stream_ != nullptr; }

    template<class Arcs, class Matrix, class Valuation>
    void Begin(int node, double bound, const Arcs& canonical, const Matrix& x,
               const Arcs& legacy, const Matrix& raw_x, const Valuation& raw)
    {
        std::fprintf(stream_, "{\"event\":\"first_branch_begin\",\"node\":%d,\"bound\":", node);
        Number(bound);
        std::fprintf(stream_, ",\"raw_valuation_order\":[");
        bool comma = false;
        for (const auto& item : raw)
        {
            if (comma) std::fprintf(stream_, ",");
            comma = true;
            std::fprintf(stream_, "[%d,", item.first.Index());
            Number(item.second);
            std::fprintf(stream_, "]");
        }
        std::fprintf(stream_, "],\"canonical_candidates\":[");
        Candidates(canonical, x);
        std::fprintf(stream_, "],\"legacy_order_candidates\":[");
        Candidates(legacy, raw_x);
        std::fprintf(stream_, "]}\n");
        std::fflush(stream_);
    }

    void Finish(int tail, int head, double estimate)
    {
        if (!stream_) return;
        std::fprintf(stream_, "{\"event\":\"first_branch_selected\",\"tail\":%d,\"head\":%d,\"estimate\":", tail, head);
        Number(estimate);
        std::fprintf(stream_, "}\n");
        std::fflush(stream_);
    }

private:
    void Number(double value)
    {
        if (std::isfinite(value)) std::fprintf(stream_, "%.17g", value);
        else std::fprintf(stream_, "null");
    }

    template<class Arcs, class Matrix>
    void Candidates(const Arcs& arcs, const Matrix& x)
    {
        bool comma = false;
        for (const auto& arc : arcs)
        {
            if (comma) std::fprintf(stream_, ",");
            comma = true;
            std::fprintf(stream_, "{\"tail\":%d,\"head\":%d,\"fraction\":", arc.tail, arc.head);
            Number(x[arc.tail][arc.head]);
            std::fprintf(stream_, ",\"distance_from_half\":");
            Number(std::fabs(0.5 - x[arc.tail][arc.head]));
            std::fprintf(stream_, "}");
        }
    }

    std::FILE* stream_ = nullptr;
};
} // namespace solver
