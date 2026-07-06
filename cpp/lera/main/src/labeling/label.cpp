//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "labeling/label.h"

using namespace std;
using namespace goc;

namespace solver
{
GraphPath Label::Path() const
{
	if (!parent) return {};
	GraphPath parent_path = parent->Path();
	parent_path.push_back(v);
	return parent_path;
}

void Label::Print(ostream& os) const
{
	os << "{P: " << Path() << ", v: " << v << ", q: " << q << ", p: " << p << ", D: " << duration << ", cost: " << min_cost << "}";
}
} // namespace