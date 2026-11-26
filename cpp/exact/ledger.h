// Ledger of the Kayros exact layer (M2 design note of session 139, section 11).
// It is the only object that may declare a run optimal. It holds the instance of
// the exact pass, the incumbent, the open nodes of the search tree and the record
// of the operations that changed its state. Every number it decides on is a GMP
// rational. The only binary64 values it reads are the instance inputs, which the
// profile defines as binary64, and the optional time limit of close, which can
// only make the pass unresolved and the close refused.
//
// Operations. Each either succeeds or changes nothing, and a refused operation is
// never recorded.
// - offer(routes) evaluates the routes exactly and makes them the incumbent when
//   they form a solution of exact cost below U. Claim 3 needs U to be the exact
//   cost of a solution.
// - branch(node, i, j) replaces the node by its 2 children of Claim 2.
// - close(node, cuts, pi, sigma) runs the exact pass (Claim 9) and computes the
//   bound of Claim 1. The node is removed iff it has no feasible route or L >= U.
// - prune(node) removes the node iff the largest bound stored by an earlier close
//   is at least the current U. Claim 1 does not depend on U.
// certified() is Claim 3, true iff no node is open and an incumbent exists.
//
// A node is named by an integer handed out by the ledger, root 0, the children of
// a branching get the next 2 unused integers (left first). An id is never reused,
// so a removed or never issued id is refused.
//
// The record holds 1 line per accepted offer and branch, and 1 close line per node removed
// by close or prune. write() serialises it with a header that rebuilds the
// instance, and replay() rebuilds a new ledger from such a file and replays it
// (criterion 6, program ledger_verify).
#pragma once

#include "exact/exact_pricing.h"

#include <array>
#include <cstddef>
#include <iosfwd>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace kayros::exact {

// The constructor inputs of Instance, kept to write the header of the record.
struct InstanceData {
    long long capacity = 0;
    double lo = 0, hi = 0;
    std::optional<double> depot_close;
    std::vector<CustomerInput> customers;  // customers[k] is customer k+1
    std::vector<ArcInput> arcs;
};

// Branching decision on the arc (i, j) between 2 customers. right = the arc is used.
struct Decision {
    int i = 0, j = 0;
    bool right = false;
};

// The multipliers of a close and the bound they gave. delta and bound are absent
// when the pass found no feasible route at the node.
struct Certificate {
    std::vector<std::array<int, 3>> cuts;
    std::vector<Q> pi;     // n+1 entries, pi[0] = 0
    std::vector<Q> sigma;  // 1 per cut, each <= 0
    std::optional<Q> delta, bound;
};

// One line of the record.
struct Operation {
    enum class Kind { offer, branch, close } kind = Kind::offer;
    std::vector<std::vector<int>> routes;  // offer
    Q cost;                                // offer, exact cost of the routes (informative)
    int node = 0;                          // branch, close
    int i = 0, j = 0;                      // branch
    int left = 0, right = 0;               // branch, ids of the children (informative)
    Certificate certificate;               // close, delta and bound informative
    Q U;                                   // close, incumbent cost used (informative)
};

struct OfferOutcome {
    bool accepted = false;
    std::string reason;
    std::optional<Q> cost;  // set when the routes form a solution
};

struct BranchOutcome {
    bool accepted = false;
    std::string reason;
    int left = -1, right = -1;
};

struct CloseOutcome {
    bool removed = false;
    std::string reason;
    std::optional<Q> delta, bound;  // set when the pass finished with a feasible route
    long long negative_count = 0;
    std::vector<std::pair<std::vector<int>, Q>> negatives;  // routes of exactly negative reduced cost
};

struct PruneOutcome {
    bool removed = false;
    std::string reason;
};

// Resource bounds of the exact pass in close. They can only make it unresolved.
struct Limits {
    long long label_limit = 0;  // 0: none
    double time_limit = 0;      // seconds, 0: none
};

class Ledger {
public:
    // fleet is the limit K on the number of routes, absent when the instance has
    // none. Throws std::invalid_argument when K < 1, when a demand is negative or
    // above the capacity, when the capacity is negative or above 2^62, and on
    // every input the Instance constructor refuses (n outside 1..max_customers,
    // non-FIFO knots, unequal horizons, ...).
    Ledger(InstanceData data, std::optional<int> fleet);

    OfferOutcome offer(const std::vector<std::vector<int>> &routes);
    BranchOutcome branch(int node, int i, int j);
    CloseOutcome close(int node, const std::vector<std::array<int, 3>> &cuts, const std::vector<Q> &pi,
                       const std::vector<Q> &sigma, const Limits &limits = {});
    PruneOutcome prune(int node);

    bool certified() const { return open_.empty() && U_.has_value(); }
    const std::optional<Q> &U() const { return U_; }
    const std::vector<std::vector<int>> &incumbent() const { return incumbent_; }
    std::size_t open_count() const { return open_.size(); }
    bool is_open(int node) const { return open_.count(node) > 0; }
    const std::vector<Operation> &record() const { return record_; }
    const Instance &instance() const { return instance_; }
    // Forbidden arcs of an open node as the pass reads them, none for an unknown node.
    std::optional<std::vector<std::pair<int, int>>> forbidden(int node) const;
    // Header and record, 1 operation per line, every rational written exactly.
    void write(std::ostream &out) const;

private:
    struct Node {
        std::vector<Decision> decisions;
        std::optional<Certificate> best;  // largest bound of a close that did not remove the node
    };
    std::vector<std::pair<int, int>> forbidden_arcs(const std::vector<Decision> &decisions) const;
    void remove(int node, const Certificate &c);
    bool reaches(const Q &L) const;  // L >= U, the test of Claim 3

    InstanceData data_;
    std::optional<int> fleet_;
    Instance instance_;
    std::optional<Q> U_;
    std::vector<std::vector<int>> incumbent_;
    std::map<int, Node> open_;
    int next_id_ = 1;
    std::vector<Operation> record_;
};

// Result of replaying a record file.
struct Replay {
    bool certified = false;
    std::string message;  // first error, or the reason why the replay is not certified
    std::size_t operations = 0;
    std::optional<Q> U;
    std::vector<std::vector<int>> incumbent;
};

// Builds a new ledger from the header and applies every operation with its stored
// arguments only. An operation the ledger refuses, a close that does not remove
// its node, an informative field that differs from the recomputed value, or a
// malformed line is an error. certified iff no error and the ledger is certified.
Replay replay(std::istream &record);

// Exact text forms used by the record, exposed for the round-trip tests.
std::string hex_double(double x);       // %a, read back exactly by parse_double
double parse_double(const std::string &s);  // throws unless s is the %a form of a finite double
Q parse_rational(const std::string &s);     // throws unless s is the get_str form of a canonical rational

}  // namespace kayros::exact
