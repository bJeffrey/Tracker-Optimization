#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace trk {

/**
 * @brief Common track ID list type used across pipeline stages and plugins.
 */
using IdList = std::vector<std::uint64_t>;

} // namespace trk
