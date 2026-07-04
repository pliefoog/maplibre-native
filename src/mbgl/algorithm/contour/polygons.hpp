#pragma once

#include <cstdint>
#include <span>
#include <utility>
#include <vector>

// Filled elevation-band polygon generation ("isobands"), sharing the same
// grid-sampling model as isolines.cpp but emitting closed polygon rings for
// the region between consecutive elevation thresholds instead of open
// contour lines.
//
// NOTE ON PROVENANCE: dave/maplibre-native's PR #4284 ("Native contour
// source on shared raster-dem tile pyramid") ships isolines.cpp,
// smoothing.cpp, intervals.cpp and units.cpp but does NOT include a working
// polygon/fill module — this file and polygons.cpp are new, written
// in-house rather than ported, using the standard "clip a cell quad against
// a scalar threshold, keeping vertex attributes for the next clip"
// technique (a textbook two-pass Sutherland-Hodgman clip; see polygons.cpp
// for the implementation). This has NOT been visually validated against
// real DEM tiles the way isolines.cpp has (71 existing unit + render
// tests) — only unit-tested against synthetic grids (see
// test/algorithm/contour/polygons.test.cpp). Treat as a first
// implementation requiring render-test coverage before being trusted for
// navigational depth-fill rendering.

namespace mbgl {
namespace algorithm {
namespace contour {

struct PolygonBand {
    double minElevation = 0.0;
    double maxElevation = 0.0;
    // Closed polygon rings in tile-local (fractional grid) coordinates.
    // Each ring is a simple polygon; the last point is implicitly connected
    // back to the first (not duplicated). Rings are NOT merged across grid
    // cells — one small quad-derived ring per contributing cell is emitted
    // as-is. Merging adjacent same-band rings into larger polygons (fewer,
    // bigger features) is a valid future optimization but not required for
    // correctness: MVT polygon layers commonly carry many small rings.
    std::vector<std::vector<std::pair<double, double>>> rings;
};

/// Generate closed fill polygons from an elevation grid, partitioned into
/// bands by consecutive pairs of `levels` (levels.size() - 1 bands).
///
/// @param heights row-major height samples, same layout as
///   isolines.cpp::generateContours: heights[row*width + col] is the
///   sample at grid position (col, row). Produces (width-1)*(height-1)
///   cells.
/// @param width  number of columns of samples. Must be >= 2.
/// @param height number of rows of samples. Must be >= 2.
/// @param levels sorted ascending elevation boundaries defining bands, e.g.
///   [-100000, 0, 2, 5, 10, 20, 50, 100]. Must have at least 2 entries for
///   any band to be produced.
/// @return one PolygonBand per consecutive level pair, in the same order as
///   `levels` (bands[i] = [levels[i], levels[i+1]]). Bands with no
///   intersecting grid cells have empty `rings` (still present in the
///   output, for a stable 1:1 mapping to the input level pairs).
std::vector<PolygonBand> generatePolygons(std::span<const std::int16_t> heights,
                                           int width,
                                           int height,
                                           const std::vector<double>& levels);

} // namespace contour
} // namespace algorithm
} // namespace mbgl
