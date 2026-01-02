#pragma once

#include <cstddef>
#include <vector>

namespace trk {

/**
 * @brief Common track ID list type used across pipeline stages and plugins.
 */
using IdList = std::vector<std::size_t>;

} // namespace trk
