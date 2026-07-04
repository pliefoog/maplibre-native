#include <mbgl/algorithm/contour/soundings.hpp>

#include <algorithm>

namespace mbgl {
namespace algorithm {
namespace contour {

std::vector<SpotSounding> generateSoundings(
    std::span<const std::int16_t> heights, int width, int height, int spacing, SpotSortOrder sortOrder) {
    std::vector<SpotSounding> soundings;

    if (spacing <= 0 || width <= 0 || height <= 0) return soundings;
    if (static_cast<std::size_t>(width) * static_cast<std::size_t>(height) > heights.size()) return soundings;

    const int cols = (width + spacing - 1) / spacing;
    const int rows = (height + spacing - 1) / spacing;
    soundings.reserve(static_cast<std::size_t>(std::max(cols, 0)) * static_cast<std::size_t>(std::max(rows, 0)));

    // Sample the center of each spacing x spacing cell (offset by spacing/2)
    // rather than the top-left corner, so labels aren't visually clustered
    // against the tile's top/left edge.
    //
    // No NaN/sentinel-magnitude filtering here (unlike the plan's original
    // `const double*` sketch): `heights` is `int16_t` (matching
    // isolines.cpp's convention), whose entire range (+-32767) is well
    // within any plausible depth/elevation sentinel, so such a check would
    // be dead code for this input type. If a future caller needs to mark
    // individual samples invalid, that requires an explicit validity mask
    // alongside the grid, not a magnitude heuristic.
    for (int row = spacing / 2; row < height; row += spacing) {
        for (int col = spacing / 2; col < width; col += spacing) {
            const double elevation = heights[static_cast<std::size_t>(row) * width + col];
            soundings.push_back({static_cast<double>(col), static_cast<double>(row), elevation});
        }
    }

    std::sort(soundings.begin(), soundings.end(), [sortOrder](const SpotSounding& a, const SpotSounding& b) {
        return sortOrder == SpotSortOrder::Ascending ? a.elevation < b.elevation : a.elevation > b.elevation;
    });

    return soundings;
}

} // namespace contour
} // namespace algorithm
} // namespace mbgl
