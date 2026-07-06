#include "nyr/solutions/objectives.h"

namespace nyr
{

void throw_invalid_objective_function(
    const ObjectiveFunction& objective
) {
    throw std::runtime_error(
        std::string("Unknown/unimplemented ObjectiveFunction: ") + 
        std::string(magic_enum::enum_name(objective))
    );
}

} // namespace nyr