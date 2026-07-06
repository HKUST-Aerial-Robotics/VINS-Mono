#pragma once

#include <algorithm>
#include <cstddef>
#include <initializer_list>

namespace tracking_safety
{

inline bool imageSizeIsValid(
    std::size_t width, std::size_t height,
    std::size_t configured_width, std::size_t configured_height,
    std::size_t camera_count)
{
    return width >= configured_width &&
           height >= configured_height * camera_count;
}

inline std::size_t commonSize(std::initializer_list<std::size_t> sizes)
{
    if (sizes.size() == 0)
        return 0;
    return *std::min_element(sizes.begin(), sizes.end());
}

inline bool timestampStepIsValid(double previous, double current)
{
    return current > previous;
}

inline bool pixelIsInside(int x, int y, int width, int height)
{
    return x >= 0 && y >= 0 && x < width && y < height;
}

}  // namespace tracking_safety
