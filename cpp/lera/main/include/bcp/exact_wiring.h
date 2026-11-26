// kayros-owned helpers of the wiring between the search and the ledger of the
// exact layer (M2 design note of session 139, section 12). Not trusted. A
// mistake here can only make the ledger refuse an operation or the search
// stall, since only a verdict of the ledger removes a node.
//
// The 2 numberings meet here and nowhere else.
// - In the search, vertex 0 is the start depot, 1..n the customers, n+1 the end depot.
// - In the ledger and the exact pass, customers are 1..n, and vertex 0 is the start
//   depot as a tail and the end depot as a head.
// Customers keep their number, so only the depots are mapped.
#pragma once

#include <array>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <goc/goc.h>

#include "bcp/exact_cost.h"
#include "exact/ledger.h"
#include "exact_lp/exact_lp.h"

namespace solver::wiring {

using kayros::exact::Q;

struct VertexMap {
	int n = 0;  // number of customers

	bool customer(int v) const { return v >= 1 && v <= n; }
	// Tail of a search arc in the ledger numbering.
	int ledger_tail(goc::Vertex v) const {
		if (v == 0) return 0;
		if (customer(v)) return v;
		throw std::invalid_argument("the end depot is no tail");
	}
	// Head of a search arc in the ledger numbering.
	int ledger_head(goc::Vertex v) const {
		if (v == n + 1) return 0;
		if (customer(v)) return v;
		throw std::invalid_argument("the start depot is no head");
	}
	goc::Arc search_arc(int tail, int head) const {
		return {tail, head == 0 ? n + 1 : head};
	}
	// Search path 0, customers..., n+1 of a ledger route.
	goc::GraphPath search_path(const std::vector<int>& customers) const {
		goc::GraphPath p;
		p.push_back(0);
		for (int c: customers) {
			if (!customer(c)) throw std::invalid_argument("ledger route vertex not a customer");
			p.push_back(c);
		}
		p.push_back(n + 1);
		return p;
	}
	// Customers of a search path. Throws unless it starts at 0 and ends at n+1.
	std::vector<int> customers_of(const goc::GraphPath& p) const {
		if (p.size() < 2 || p.front() != 0 || p.back() != n + 1) throw std::invalid_argument("search path without depots");
		std::vector<int> out;
		for (std::size_t k = 1; k + 1 < p.size(); ++k) {
			if (!customer(p[k])) throw std::invalid_argument("search path vertex not a customer");
			out.push_back(p[k]);
		}
		return out;
	}
};

// Multipliers handed to Ledger::close.
struct Multipliers {
	std::vector<Q> pi;  // n+1 entries, pi[0] = 0
	std::vector<std::array<int, 3>> cuts;
	std::vector<Q> sigma;  // 1 per cut
};

// Multipliers of an optimal exact solution of SPF::ExactMaster. The master has
// n+2 depot and customer rows (row v is vertex v of the search) followed by 1
// row per cut. The 2 depot rows are empty equality rows, so their multipliers
// enter neither a reduced cost nor the dual value, and are dropped. cut_of(c)
// gives the customers of cut c.
template <class CutOf>
Multipliers multipliers_of(const kayros::exact_lp::Solution& sol, int n, int cut_count, CutOf cut_of) {
	if (sol.status != kayros::exact_lp::Status::Optimal) throw std::invalid_argument("exact solution not optimal");
	if ((int)sol.duals.size() != n + 2 + cut_count) throw std::invalid_argument("exact solution of another master");
	Multipliers m;
	m.pi.assign(n + 1, Q(0));
	for (int v = 1; v <= n; ++v) m.pi[v] = sol.duals[v];
	for (int c = 0; c < cut_count; ++c) {
		m.cuts.push_back(cut_of(c));
		m.sigma.push_back(sol.duals[n + 2 + c]);
	}
	return m;
}

// Exact cost of a route as the ledger saw it, from its exact reduced cost and
// the multipliers of the close, c(r) = rc + sum of pi over r + sum of sigma over
// the cuts that r visits at least twice (design note, section 1).
inline Q cost_from_reduced(const std::vector<int>& customers, const Q& rc, const Multipliers& m) {
	Q c = rc;
	for (int v: customers) c += m.pi.at(v);
	for (std::size_t k = 0; k < m.cuts.size(); ++k) {
		int visits = 0;
		for (int v: customers)
			if (v == m.cuts[k][0] || v == m.cuts[k][1] || v == m.cuts[k][2]) ++visits;
		if (visits >= 2) c += m.sigma[k];
	}
	c.canonicalize();
	return c;
}

// Columns of an exact primal solution when it is integral, i.e. when every
// nonzero value is exactly 1. Returns none when some value is fractional.
inline std::optional<std::vector<int>> integral_columns(const kayros::exact_lp::Solution& sol) {
	std::vector<int> cols;
	for (const auto& [j, x]: sol.primal) {
		if (x != 1) return std::nullopt;
		cols.push_back(j);
	}
	return cols;
}

// Forbidden arcs of a ledger node in the search numbering, split into the arcs
// of the search digraph D and the others. Self loops never occur in the ledger's
// set and arcs into the start depot or out of the end depot do not exist.
inline std::pair<std::vector<goc::Arc>, std::vector<goc::Arc>> split_forbidden(
		const std::vector<std::pair<int, int>>& ledger_arcs, const VertexMap& map, const goc::Digraph& D) {
	std::vector<goc::Arc> in_d, outside;
	for (const auto& [i, j]: ledger_arcs) {
		const goc::Arc e = map.search_arc(i, j);
		if (D.IncludesArc(e)) in_d.push_back(e); else outside.push_back(e);
	}
	return {in_d, outside};
}

}  // namespace solver::wiring
