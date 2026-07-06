//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/graph/graph_path.h"

#include <unordered_set>

using namespace std;

namespace goc {

bool has_cycle(GraphPath p, int max_size)
{
    unordered_set<int> V;
    size_t window_size = max_size < 0 ? 0 : min(static_cast<size_t>(max_size), p.size());
    
    // Add to V all vertices in (p[0], ..., p[max_size-1]).
    // If any vertex is repeated, then it has a cycle.
    for (size_t i = 0; i < window_size; ++i)
    {
        if (V.find(p[i]) != V.end()) return true;
        V.insert(p[i]);
    }
    
    // Now move the window (p[i-max_size+1], ..., p[i]) until the end i==|p|-1.
    for (size_t i = window_size; i < p.size(); ++i)
    {
        V.erase(p[i-window_size]);
        if (V.find(p[i]) != V.end()) return true;
        V.insert(p[i]);
    }
    
    return false;
}

bool operator==(const GraphPath& p1, const GraphPath& p2)
{
    if (p1.size() != p2.size()) return false;
    for (size_t i = 0; i < p1.size(); ++i)
        if (p1[i] != p2[i]) return false;
    return true;
}

} // namespace goc
