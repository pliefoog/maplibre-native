#pragma once

#include <utility>
#include <vector>

// Per-zoom EXPLICIT elevation-level schedule ("lineLevels" / "polygonLevels"
// in this app's style-generation schema — see
// boatingInstrumentsApp/scripts/generate-map-style.js and
// ChartMapContent.web.tsx's CONTOUR_PARAMS). This is a different data model
// from IntervalSchedule (intervals.hpp): IntervalSchedule picks a single
// CONSTANT interval width per zoom (e.g. "20m apart"); LevelSchedule instead
// gives an explicit, deliberately IRREGULAR list of elevation breakpoints
// per zoom (e.g. [0, 2, 5, 10, 20, 50, 100, 250, 500, 1000, 2000, 3000, 4000,
// 5000] -- fine resolution near the surface, coarse at depth, matching
// paper-chart / IHO depth-band convention). A constant interval CANNOT
// reproduce that list (its deltas are 2, 3, 5, 10, 30, 50, 150, 250, 500,
// 1000 -- not constant), so this is a genuinely separate schedule type, not
// a variant of IntervalSchedule.
//
// PROVENANCE: not part of dave/maplibre-native's PR #4284, which only
// supports `intervals` (IntervalSchedule). Added in-house so the native
// `contour` source can represent the same depth bands the web app's
// generated style JSON already ships (`lineLevels` / `polygonLevels`),
// which the web app's prozessor13/maplibre-contour JS fork already
// supports natively.

namespace mbgl {
namespace algorithm {
namespace contour {

struct LevelSchedule {
    // Ascending by zoom breakpoint. Each entry is (zoomBreakpoint, levels),
    // where `levels` is itself ascending (e.g. {5.0, {0,2,5,10,20,...}}
    // means "at zoom >= 5, use these breakpoints"). Mirrors the JS-side
    // schema's `{ "5": [0,2,5,...] }` object-of-arrays-keyed-by-zoom shape
    // (parsed into this ordered form by the style conversion layer; see
    // `convertContourSource` in style/conversion/source.cpp).
    std::vector<std::pair<double, std::vector<double>>> entries;
};

/// Resolve the level list to use at a given zoom: the entry with the
/// largest zoom breakpoint <= `zoom`, or -- if `zoom` is below every
/// breakpoint -- the entry with the SMALLEST breakpoint (so a tile below
/// the lowest configured zoom still gets *some* bands rather than none;
/// matches "use the coarsest configured detail level rather than nothing"
/// tile-rendering practice already used for the base `intervals` schedule's
/// output-above-last-stop behaviour).
///
/// Returns an empty vector for an empty or invalid schedule.
const std::vector<double>& resolveLevels(const LevelSchedule& schedule, double zoom);

/// True when the schedule is non-empty, breakpoints are strictly
/// increasing, and every level list has at least 2 strictly-increasing
/// entries (a single level can't define a band or a line threshold in a
/// list of >= 2 values).
bool isValid(const LevelSchedule& schedule);

} // namespace contour
} // namespace algorithm
} // namespace mbgl
