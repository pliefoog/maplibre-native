#pragma once

#include <cstdint>
#include <span>
#include <vector>

// Spot-sounding (grid-sampled depth label point) generation. Pure algorithm,
// no MapLibre dependencies — matches the grid/corner conventions used by
// isolines.cpp and polygons.cpp in this same directory.
//
// PROVENANCE: dave/maplibre-native's PR #4284 does not include a soundings
// module (it ships isolines, smoothing, intervals, units only). This file
// and soundings.cpp are new, written in-house following the same design as
// this app's existing web implementation (`CONTOUR_PARAMS.spotGridSpacing`
// / `spotSortOrder` in ChartMapContent.web.tsx, itself following the
// prozessor13/maplibre-contour JS fork's spot-sounding behaviour).

namespace mbgl {
namespace algorithm {
namespace contour {

struct SpotSounding {
    // Tile-local grid coordinates (fractional column/row, same coordinate
    // space as isolines.cpp/polygons.cpp; NOT yet scaled to MVT extent —
    // that scaling happens where these are emitted as vector-tile features,
    // same as contour lines and fill polygons).
    double x = 0.0;
    double y = 0.0;
    double elevation = 0.0;
};

enum class SpotSortOrder {
    Ascending,
    Descending,
};

/// Generate spot soundings by regularly sampling an elevation grid.
///
/// @param heights row-major height samples, same layout as
///   isolines.cpp::generateContours: heights[row*width + col].
/// @param width  number of columns of samples. Must be >= 1.
/// @param height number of rows of samples. Must be >= 1.
/// @param spacing grid sampling interval in samples (e.g. 32 — one sounding
///   roughly every 32 grid samples in each axis). A spacing of 0 produces no
///   soundings (avoids a division/step of zero).
/// @param sortOrder ascending or descending by elevation.
/// @return one SpotSounding per sampled grid point, in `sortOrder`.
std::vector<SpotSounding> generateSoundings(std::span<const std::int16_t> heights,
                                             int width,
                                             int height,
                                             int spacing,
                                             SpotSortOrder sortOrder);

} // namespace contour
} // namespace algorithm
} // namespace mbgl
