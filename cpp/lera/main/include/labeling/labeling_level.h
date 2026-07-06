#pragma once

namespace solver
{
/**
 * @enum LabelingLevel
 * @brief Enumeration of labeling levels for the labeling algorithm.
 *
 * The `LabelingLevel` enum class defines the various levels of resolution for the
 * labeling algorithm. These levels determine the strategy used, ranging from heuristic
 * approaches to exact resolution.
 * 
 * Note that the levels are ordered and should be used such 
 * that faster levels are tried first.
 */
enum class LabelingLevel {
    HeuristicCost, // Represents a heuristic resolution where the cost constraint is relaxed.
    HeuristicElementarity, // Represents a heuristic resolution where the elementarity constraints is relaxed.
    HeuristicNG, // Represents a heuristic resolution using the NG-Route relaxation (partial relaxation of elementarity).
    Exact // Represents an exact resolution of the labeling algorithm.
};
} // namespace solver
