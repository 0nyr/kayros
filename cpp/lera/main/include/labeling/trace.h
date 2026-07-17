//
// kayros (M5.9/M13.0) label-trace harness. Dev tool, zero cost when the env
// vars are unset. Shared by the monodirectional labeling (KAYROS_TRACE_PATH:
// per-label extension/domination/enumeration fates for prefix labels of the
// target vertex sequence) and the bidirectional merge (KAYROS_TRACE_MERGE: a
// full route 0,...,d whose forward-prefix x backward-suffix merge attempts in
// Merge/LastArcMerge are printed with their arithmetic). Both print to stderr.
//

#ifndef LABELING_TRACE_H
#define LABELING_TRACE_H

#include <cstdlib>
#include <string>
#include <vector>

#include "goc/goc.h"
#include "labeling/label.h"

namespace solver
{
namespace trace
{
inline std::vector<goc::Vertex> parse_env(const char* name)
{
	std::vector<goc::Vertex> v;
	const char* env = std::getenv(name);
	if (env)
	{
		int cur = -1;
		for (const char* c = env;; ++c)
		{
			if (*c >= '0' && *c <= '9') cur = (cur < 0 ? 0 : cur * 10) + (*c - '0');
			else { if (cur >= 0) v.push_back(cur); cur = -1; if (!*c) break; }
		}
	}
	return v;
}

inline const std::vector<goc::Vertex>& target()
{
	static std::vector<goc::Vertex> t = parse_env("KAYROS_TRACE_PATH");
	return t;
}

inline bool on() { return !target().empty(); }

// Returns the label's path length if it is a prefix of the target, else -1.
inline int prefix_len(const Label* l)
{
	if (!on()) return -1;
	goc::GraphPath path = l->Path();
	if (path.empty() || path.size() > target().size()) return -1;
	for (size_t k = 0; k < path.size(); ++k)
		if (path[k] != target()[k]) return -1;
	return (int) path.size();
}

inline std::string path_str(const Label* l)
{
	std::string s;
	for (auto v: l->Path()) s += (s.empty() ? "" : ",") + std::to_string(v);
	return s;
}

// --- merge tracing (M13.0) ---------------------------------------------------

inline const std::vector<goc::Vertex>& merge_target()
{
	static std::vector<goc::Vertex> t = parse_env("KAYROS_TRACE_MERGE");
	return t;
}

inline bool merge_on() { return !merge_target().empty(); }

// A forward label l (path 0..u) and a backward label m (path d..u in the
// reversed instance) merge into l->Path() + reverse(m->Path() minus its last
// vertex u). Returns true iff that combined route equals the merge target.
inline bool merge_matches(const Label* l, const Label* m)
{
	if (!merge_on()) return false;
	const auto& t = merge_target();
	goc::GraphPath lp = l->Path(), mp = m->Path();
	if (lp.empty() || mp.empty() || lp.back() != mp.back()) return false;
	if (lp.size() + mp.size() - 1 != t.size()) return false;
	for (size_t k = 0; k < lp.size(); ++k)
		if (lp[k] != t[k]) return false;
	for (size_t k = 0; k + 1 < mp.size(); ++k)
		if (mp[k] != t[t.size() - 1 - k]) return false;
	return true;
}

// Is l's path a prefix of the merge target (diagnostic for LastArcMerge)?
inline bool merge_fwd_prefix(const Label* l)
{
	if (!merge_on()) return false;
	const auto& t = merge_target();
	goc::GraphPath lp = l->Path();
	if (lp.empty() || lp.size() > t.size()) return false;
	for (size_t k = 0; k < lp.size(); ++k)
		if (lp[k] != t[k]) return false;
	return true;
}

// Is m's path a reversed suffix of the merge target?
inline bool merge_bwd_suffix(const Label* m)
{
	if (!merge_on()) return false;
	const auto& t = merge_target();
	goc::GraphPath mp = m->Path();
	if (mp.empty() || mp.size() > t.size()) return false;
	for (size_t k = 0; k < mp.size(); ++k)
		if (mp[k] != t[t.size() - 1 - k]) return false;
	return true;
}
} // namespace trace
} // namespace solver

#endif // LABELING_TRACE_H
