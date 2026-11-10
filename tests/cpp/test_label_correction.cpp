#include <goc/goc.h>
#include <nyr/nyr.h>

#include <iostream>
#include <optional>
#include <stdexcept>

#include "labeling/bidirectional_labeling.h"

using namespace goc;
using namespace nyr;
using namespace solver;

namespace solver {
struct CorrectionInvariantTest {
    static MonodirectionalLabeling& Mono(BidirectionalLabeling& solver) { return solver.lbl_[0]; }
    static int Correct(MonodirectionalLabeling& solver, Label* label) { return solver.CorrectionStep(label); }
    static bool Dominated(MonodirectionalLabeling& solver, Label* label) { return solver.DominationStep(label); }
    static Label* Extend(MonodirectionalLabeling& solver, const LazyLabel& label) { return solver.ExtensionStep(label); }
    static std::size_t Retired(MonodirectionalLabeling& solver) { return solver.retired_labels_.size(); }
    static std::size_t Changes(MonodirectionalLabeling& solver) { return solver.corrected_merge_buckets_.size(); }
    static void Refresh(BidirectionalLabeling& solver) { solver.RefreshCorrectedMergeLabels(0); }
    static auto& Pool(BidirectionalLabeling& solver) {
        if (solver.M[0].empty()) solver.M[0].resize(5);
        return solver.M[0][1].Insert(1, {});
    }
    static std::size_t MergePool(BidirectionalLabeling& solver, Label* label) {
        solver.pp_.P = std::vector<double>(5, 0.0);
        solver.S.clear();
        solver.IterativeMerge(label, solver.M[0]);
        return solver.S.size();
    }
    static void Clean(BidirectionalLabeling& solver) {
        solver.M[0].clear();
        solver.lbl_[0].Clean();
    }
};
}

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

VRPInstance instance() {
    VRPInstance vrp;
    vrp.D = Digraph(5);
    vrp.o = 0;
    vrp.d = 4;
    vrp.T = 10;
    vrp.horizon = {0.0, 10.0};
    vrp.Q = 100;
    vrp.q = std::vector<double>(5, 0.0);
    vrp.tw = std::vector<goc::Interval>(5, {0.0, 10.0});
    vrp.tau = Matrix<PWLFunction>(5, 5);
    vrp.pretau = Matrix<PWLFunction>(5, 5);
    vrp.arr = Matrix<PWLFunction>(5, 5);
    vrp.dep = Matrix<PWLFunction>(5, 5);
    vrp.LDT = Matrix<double>(5, 5);
    vrp.tau[0][0] = PWLFunction::ConstantFunction(0.0, {0.0, 10.0});
    return vrp;
}

void initialize(Label& label, PWLFunction duration, VertexSet unreachable) {
    label.parent = nullptr;
    label.v = 1;
    label.q = 1;
    label.p = label.cut_cost = 0;
    label.duration = duration;
    label.rw = duration.Domain();
    label.min_cost = std::min(img(duration));
    label.U = unreachable;
}

void check(bool step_exact, bool sort_by_cost) {
    StepExactArithmeticScope arithmetic(step_exact);
    const std::optional<TDNGRoutesParams> ng = std::nullopt;
    BidirectionalLabeling bidirectional(instance(), ng);
    require(!bidirectional.correcting, "Correction must remain disabled without diagnostic opt-in");
    bidirectional.correcting = true;
    bidirectional.closing_state = false;
    auto& mono = CorrectionInvariantTest::Mono(bidirectional);
    mono.correcting = true;
    mono.sort_by_cost = sort_by_cost;
    VertexSet a, b;
    a.set(1).set(2).set(4);
    b.set(1).set(3).set(4);
    auto* first = new Label;
    auto* second = new Label;
    PWLFunction rising;
    rising.AddPiece(LinearFunction({0.0, 0.0}, {10.0, 5.0}));
    initialize(*first, rising, a);
    initialize(*second, PWLFunction::ConstantFunction(0.5, {0.0, 10.0}), b);
    auto& bucket = mono.U[1].Insert(1, {});
    bucket = {first, second};
    CorrectionInvariantTest::Pool(bidirectional) = {first, second};
    Label arriving;
    initialize(arriving, PWLFunction::ConstantFunction(0.0, {0.0, 1.0}), a);
    require(CorrectionInvariantTest::Correct(mono, &arriving) == 0, "Expected partial correction only");
    require(first->min_cost == 1.0, "Expected partial correction to increase the minimum cost");
    require(bucket[0] == (sort_by_cost ? second : first),
            "Correction must repair the dominance order only when that order is enabled");
    Label query;
    initialize(query, PWLFunction::ConstantFunction(0.75, {0.0, 10.0}), b);
    require(CorrectionInvariantTest::Dominated(mono, &query), "Corrected index must retain a valid dominator");
    require(CorrectionInvariantTest::Changes(mono) == 1, "Changed merge bucket must be recorded");
    CorrectionInvariantTest::Refresh(bidirectional);
    require(CorrectionInvariantTest::Changes(mono) == 0, "Merge refresh must drain changed buckets");
    require(CorrectionInvariantTest::Pool(bidirectional)[0] == second,
            "Merge refresh must repair min-cost ordering before early breaks");

    Label null_forward, start, forward, null_backward, end, middle_first, middle_second;
    null_forward.parent = null_backward.parent = nullptr;
    start.parent = &null_forward;
    start.v = 0;
    start.cut_cost = 0;
    initialize(forward, PWLFunction::ConstantFunction(0.0, {0.0, 10.0}), {});
    forward.parent = &start;
    forward.p = 0.75;
    forward.min_cost = -0.75;
    forward.S.set(0).set(1);
    end.parent = &null_backward;
    end.v = 4;
    middle_first.parent = middle_second.parent = &end;
    middle_first.v = 2;
    middle_second.v = 3;
    first->parent = &middle_first;
    second->parent = &middle_second;
    first->S = a;
    second->S = b;
    require(CorrectionInvariantTest::MergePool(bidirectional, &forward) == 1,
            "Repaired merge index must retain the negative-cost route");
    CorrectionInvariantTest::Clean(bidirectional);

    for (bool partial : {true, false}) {
        mono.partial = partial;
        auto* retired = new Label;
        initialize(*retired, PWLFunction::ConstantFunction(1.0, {0.0, 10.0}), a);
        mono.U[1].Insert(1, {}).push_back(retired);
        CorrectionInvariantTest::Pool(bidirectional) = {retired};
        initialize(arriving, PWLFunction::ConstantFunction(0.0, {0.0, 10.0}), a);
        require(CorrectionInvariantTest::Correct(mono, &arriving) == 1, "Expected full correction");
        require(CorrectionInvariantTest::Retired(mono) == 1, "Removed parent must retain ownership until cleanup");
        if (partial) {
            require(retired->duration.Empty(), "Fully corrected duration must be empty");
            LazyLabel child{retired, 2, 0.0};
            require(CorrectionInvariantTest::Extend(mono, child) == nullptr,
                    "Lazy child must safely reject its live empty parent");
            CorrectionInvariantTest::Refresh(bidirectional);
            require(CorrectionInvariantTest::Pool(bidirectional).empty(),
                    "Merge index must remove empty corrected labels");
            require(CorrectionInvariantTest::MergePool(bidirectional, retired) == 0,
                    "Empty forward merge label must be ignored");
        } else {
            require(!retired->duration.Empty(), "Full-only correction must preserve its existing valid duration semantics");
        }
        CorrectionInvariantTest::Clean(bidirectional);
        require(CorrectionInvariantTest::Retired(mono) == 0, "Cleanup must release retired owners");
    }

    // A newly extended label can move beyond the current middle after pruning.
    // Such a label is requeued, so it must not retire the current stored label.
    mono.partial = true;
    mono.cross = false;
    mono.lazy_extension = false;
    mono.t_m = 5.0;
    auto* existing = new Label;
    initialize(*existing, PWLFunction::ConstantFunction(1.0, {8.0, 10.0}), a);
    mono.U[1].Insert(1, {}).push_back(existing);
    Label parent;
    initialize(parent, PWLFunction::ConstantFunction(0.0, {0.0, 10.0}), {});
    parent.v = 0;
    parent.q = 0;
    parent.length = 0;
    auto* beyond_middle = new Label;
    initialize(*beyond_middle, PWLFunction::ConstantFunction(0.0, {8.0, 10.0}), a);
    beyond_middle->parent = &parent;
    beyond_middle->length = 1;
    LazyLabel lazy{&parent, 1, 0.0};
    lazy.extension = beyond_middle;
    LBQueue queue;
    queue.push(lazy);
    MLBExecutionLog log(true);
    const auto processed = mono.Run(&queue, &log);
    require(processed.empty() && !queue.empty() && queue.top().makespan == 8.0,
            "Boundary-crossing candidate must be deferred");
    require(CorrectionInvariantTest::Retired(mono) == 0 && !existing->duration.Empty(),
            "A deferred candidate must not retire a stored label");
    CorrectionInvariantTest::Clean(bidirectional);
}

int main() {
    try {
        for (bool arithmetic : {false, true})
            for (bool sorting : {false, true}) check(arithmetic, sorting);
        std::cout << "Correction ordering, negative-route retention, ownership and deferred-label checks pass in both arithmetic modes and sorting settings\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
