#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

// Marching-squares contour-line generation. Pure algorithm, no MapLibre
// dependencies — testable in isolation, reusable outside the renderer.
//
// - Single-pass cell traversal: emits all threshold levels per cell in one
//   sweep through the grid (~5× faster than per-level passes for typical
//   contour-density tiles).
// - Incremental polyline stitching: maintains start/end fragment maps during
//   traversal, no post-pass.
// - Saddle cells (cases 5, 10) emit both segments of one fixed pairing —
//   sufficient for typical DEM data where saddle points are sparse and
//   visually subtle. Center-average disambiguation is a future improvement.
// - Output in tile-local integer coordinates (default extent 4096), suitable
//   for direct emission into vector-tile features.

namespace mbgl {
namespace algorithm {
namespace contour {

struct ContourThresholds {
    // Vertical distance between contours in the same units as the grid.
    double interval = 0.0;
    // Tile-local coordinate scale (MVT default = 4096).
    int extent = 4096;
};

struct ContourLineString {
    // Threshold value this line traces, in the same units as the input grid.
    double elevation = 0.0;
    // Interleaved tile-local (x, y) integer coordinates: points[0] = x0,
    // points[1] = y0, points[2] = x1, points[3] = y1, … Always even length.
    std::vector<std::int32_t> points;
};

/// Generate contour lines from a height grid.
///
/// @param heights row-major height samples. heights[y*width + x] is the
///   height at the top-left corner of cell (x, y). Grid is treated as a
///   sample lattice of (width × height) values, producing
///   ((width-1) × (height-1)) cells.
/// @param width   number of columns of samples. Must be ≥ 2.
/// @param height  number of rows of samples. Must be ≥ 2.
/// @param thresholds configuration; `interval` must be > 0.
/// @return One ContourLineString per stitched polyline. Lines are not
///   simplified or smoothed — that is the consumer's responsibility.
std::vector<ContourLineString> generateContours(std::span<const std::int16_t> heights,
                                                int width,
                                                int height,
                                                const ContourThresholds& thresholds);

/// Generate contour lines at an EXPLICIT, arbitrarily-spaced set of
/// threshold levels, instead of a constant interval. Needed for maritime
/// depth-band charting convention (see algorithm/contour/levels.hpp): a
/// constant `ContourThresholds::interval` cannot reproduce a breakpoint
/// list like [0, 2, 5, 10, 20, 50, 100, 250, 500, 1000, 2000, 3000, 4000,
/// 5000] (deltas of 2, 3, 5, 10, 30, 50, 150, 250, 500, 1000 — not
/// constant).
///
/// Uses the same marching-squares cell traversal, saddle handling, and
/// fragment-stitching as `generateContours` (deliberately duplicated rather
/// than refactored into a shared helper here — this codebase's other
/// pure-algorithm modules, e.g. `generateContours` itself, are extensively
/// unit- and render-tested upstream; restructuring proven, stitching-
/// sensitive code without that same test harness available in this
/// environment is a real regression risk this file avoids by keeping the
/// two entry points independent).
///
/// @param heights same grid convention as `generateContours`.
/// @param width, height same as `generateContours`.
/// @param levels ascending, distinct threshold values. Levels outside a
///   given cell's [min, max] corner range are simply skipped for that
///   cell (same effect as `generateContours`' start/end-level clamping).
/// @param extent tile-local coordinate scale (MVT default = 4096).
/// @return One ContourLineString per stitched polyline, one group per
///   input level (levels with no crossings anywhere in the grid are
///   simply absent from the result, same as `generateContours`).
std::vector<ContourLineString> generateContoursAtLevels(std::span<const std::int16_t> heights,
                                                        int width,
                                                        int height,
                                                        const std::vector<double>& levels,
                                                        int extent = 4096);

} // namespace contour
} // namespace algorithm
} // namespace mbgl
