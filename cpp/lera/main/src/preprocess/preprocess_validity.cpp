#include "preprocess/preprocess_validity.h"
#include "nyr/vrp/instance.h"

using namespace std;
using namespace goc;
using namespace nyr;
using namespace nlohmann;

namespace solver
{
void preprocess_validity(nlohmann::json& instance)
{
    // Step 1: Arc checks.
    VRPInstance vrp = instance;

    // Check that the start depot has no incoming arcs.
    assert (vrp.D.InboundArcs(vrp.o).empty() && "The start depot has incoming arcs.");

    // Check that the end depot has no outgoing arcs.
    assert (vrp.D.OutboundArcs(vrp.d).empty() && "The end depot has outgoing arcs.");

    // Check that all client vertices have at least one incoming and one outgoing arc.
    for (Vertex i: vrp.D.Vertices())
    {
        if (i == vrp.o || i == vrp.d) continue;
        assert (!vrp.D.InboundArcs(i).empty() && ("A client vertex has no incoming arcs. Culprit vertex: " + to_string(i)).c_str());
        assert (!vrp.D.OutboundArcs(i).empty() && ("A client vertex has no outgoing arcs. Culprit vertex: " + to_string(i)).c_str());
    }
}
} // namespace