#include <mbgl/tile/contour_tile.hpp>

#include <mbgl/actor/scheduler.hpp>
#include <mbgl/algorithm/contour/isolines.hpp>
#include <mbgl/algorithm/contour/polygons.hpp>
#include <mbgl/algorithm/contour/smoothing.hpp>
#include <mbgl/algorithm/contour/soundings.hpp>
#include <mbgl/algorithm/contour/units.hpp>
#include <mbgl/geometry/dem_data.hpp>
#include <mbgl/renderer/buckets/hillshade_bucket.hpp>
#include <mbgl/renderer/tile_parameters.hpp>
#include <mbgl/tile/geojson_tile_data.hpp>
#include <mbgl/tile/raster_dem_tile.hpp>
#include <mbgl/util/constants.hpp>
#include <mbgl/util/identity.hpp>

#include <mapbox/feature.hpp>
#include <mapbox/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace mbgl {

namespace {

// Half-pixel-in-tile-local-units, used as the Douglas-Peucker simplification
// tolerance. With `util::EXTENT = 8192` across a 512-px MapLibre vector
// tile, one device pixel is 16 tile-local units; half a pixel is 8.
// Sub-pixel wiggle gets dropped before Chaikin's corner-cutting so the
// smoothing pass doesn't waste iterations on noise.
constexpr double kSimplifyEpsilon = 8.0;

// Two iterations of Chaikin gives a noticeable smoothing of the
// marching-squares staircase without ballooning the vertex count
// (each iteration roughly doubles vertices). Empirically a good
// quality / size balance.
constexpr int kChaikinIterations = 2;

// Sutherland-Hodgman-style polyline clip against an axis-aligned rect
// `[0, EXTENT] × [0, EXTENT]` in tile-local coords. Returns a list of
// sub-polylines covering only the interior portion of the input. Each
// sub-polyline starts and ends either at an interior vertex or at a
// segment-rect intersection point, so adjacent tiles' clipped lines
// share the boundary intersection exactly (both sides compute it from
// the same two MS vertices in the overlap region).
//
// Why this matters: Chaikin's corner-cutting preserves the first and
// last input vertex. If we trim each line so its endpoints land exactly
// on the tile boundary, the smoothed line in tile A and the smoothed
// line in tile B both terminate at the same boundary crossing point,
// so the rendered contours meet without a visible gap. Without trimming,
// each tile's smoothing window reached different vertices on the far
// side of the seam, producing slightly different smoothed curves and
// leaving misaligned ends.
std::vector<std::vector<algorithm::contour::Point2D>> clipPolylineToTile(
    const std::vector<algorithm::contour::Point2D>& line) {
    std::vector<std::vector<algorithm::contour::Point2D>> result;
    if (line.size() < 2) return result;

    constexpr double minX = 0.0;
    constexpr double minY = 0.0;
    constexpr double maxX = static_cast<double>(util::EXTENT);
    constexpr double maxY = static_cast<double>(util::EXTENT);
    constexpr double eps = 1e-9;

    auto isInside = [](const algorithm::contour::Point2D& p) {
        return p[0] >= minX - eps && p[0] <= maxX + eps && p[1] >= minY - eps && p[1] <= maxY + eps;
    };

    // Liang-Barsky line-rect intersection — return the parameter `t` (in
    // [0, 1]) at which segment a→b enters/exits the rect, or `nan` if
    // there's no intersection.
    auto intersectSegment = [&](const algorithm::contour::Point2D& a,
                                const algorithm::contour::Point2D& b,
                                bool wantEntry) -> std::optional<algorithm::contour::Point2D> {
        const double dx = b[0] - a[0];
        const double dy = b[1] - a[1];
        double tEnter = 0.0;
        double tExit = 1.0;
        const double p[4] = {-dx, dx, -dy, dy};
        const double q[4] = {a[0] - minX, maxX - a[0], a[1] - minY, maxY - a[1]};
        for (int i = 0; i < 4; i++) {
            if (p[i] == 0.0) {
                if (q[i] < 0.0) return std::nullopt;
            } else {
                const double t = q[i] / p[i];
                if (p[i] < 0.0) {
                    if (t > tExit) return std::nullopt;
                    if (t > tEnter) tEnter = t;
                } else {
                    if (t < tEnter) return std::nullopt;
                    if (t < tExit) tExit = t;
                }
            }
        }
        const double t = wantEntry ? tEnter : tExit;
        return algorithm::contour::Point2D{a[0] + t * dx, a[1] + t * dy};
    };

    std::vector<algorithm::contour::Point2D> current;
    current.reserve(line.size());
    bool prevInside = isInside(line[0]);
    if (prevInside) current.push_back(line[0]);

    for (std::size_t i = 1; i < line.size(); i++) {
        const auto& a = line[i - 1];
        const auto& b = line[i];
        const bool curInside = isInside(b);
        if (prevInside && curInside) {
            current.push_back(b);
        } else if (prevInside && !curInside) {
            if (auto exit = intersectSegment(a, b, false); exit) {
                current.push_back(*exit);
            }
            if (current.size() >= 2) result.push_back(std::move(current));
            current.clear();
        } else if (!prevInside && curInside) {
            if (auto entry = intersectSegment(a, b, true); entry) {
                current.push_back(*entry);
            }
            current.push_back(b);
        } else {
            // Both endpoints outside — segment may still cross the rect.
            // Test for an in-and-out crossing and emit it as a 2-vertex
            // sub-polyline.
            const double dx = b[0] - a[0];
            const double dy = b[1] - a[1];
            double tEnter = 0.0;
            double tExit = 1.0;
            const double p[4] = {-dx, dx, -dy, dy};
            const double q[4] = {a[0] - minX, maxX - a[0], a[1] - minY, maxY - a[1]};
            bool ok = true;
            for (int j = 0; j < 4 && ok; j++) {
                if (p[j] == 0.0) {
                    if (q[j] < 0.0) ok = false;
                } else {
                    const double t = q[j] / p[j];
                    if (p[j] < 0.0) {
                        if (t > tExit)
                            ok = false;
                        else if (t > tEnter)
                            tEnter = t;
                    } else {
                        if (t < tEnter)
                            ok = false;
                        else if (t < tExit)
                            tExit = t;
                    }
                }
            }
            if (ok && tEnter < tExit) {
                std::vector<algorithm::contour::Point2D> crossing;
                crossing.push_back({a[0] + tEnter * dx, a[1] + tEnter * dy});
                crossing.push_back({a[0] + tExit * dx, a[1] + tExit * dy});
                result.push_back(std::move(crossing));
            }
        }
        prevInside = curInside;
    }
    if (current.size() >= 2) result.push_back(std::move(current));
    return result;
}

// Convert raw marching-squares output (interleaved int32 tile-local coords
// with elevation in metres) to vector-tile features.
// Per-feature properties (per the maplibre-style-spec contour-source
// proposal, #583, PLUS a `level` property added for explicit-levels mode --
// see below):
//   ele      — elevation in display units (rounded int).
//   interval — the contour spacing this tile was generated at, in display
//              units. Constant-interval mode only; 0 in explicit-levels mode
//              (there is no single spacing to report).
//   major    — true iff the line's elevation is a multiple of
//              `interval × majorMultiplier` resolved at this tile's zoom.
//              Constant-interval mode only; always false in explicit-levels
//              mode.
//   level    — EXPLICIT-LEVELS MODE ONLY: this line's index within the
//              resolved `lineLevelsMeters` array, matching the web app's
//              CONTOUR_PARAMS.levelKey='level' schema (its
//              depth-contour-lines paint expression keys line-width off
//              this property, e.g. `['match', ['get', 'level'], 1, 1.2,
//              0.6]`). Absent from feature properties in constant-interval
//              mode (nothing meaningful to report).
//
// Pipeline per line:
//   1. Clip to the [0, EXTENT] tile bbox so endpoints land on the
//      boundary at the shared crossing point.
//   2. Douglas-Peucker simplification (drops sub-pixel wiggle so
//      Chaikin doesn't waste iterations on noise).
//   3. Chaikin corner-cutting (endpoints preserved by the algorithm,
//      and now also clamped to the tile boundary so adjacent tiles'
//      smoothed lines meet exactly).
//   4. Round to int16 MVT coords and emit.
mapbox::feature::feature_collection<std::int16_t> toFeatures(
    const std::vector<algorithm::contour::ContourLineString>& lines,
    const algorithm::contour::UnitConfig& unit,
    double intervalDisplayUnits,
    std::int64_t majorMultiplier,
    const std::vector<double>* explicitLevelsMeters = nullptr) {
    mapbox::feature::feature_collection<std::int16_t> features;
    features.reserve(lines.size());

    const std::int64_t intervalRound = static_cast<std::int64_t>(std::llround(intervalDisplayUnits));
    const std::int64_t majorMod = (majorMultiplier > 0 && intervalRound > 0) ? intervalRound * majorMultiplier : 0;

    std::vector<algorithm::contour::Point2D> work;
    for (const auto& line : lines) {
        if (line.points.size() < 4) continue;

        work.clear();
        work.reserve(line.points.size() / 2);
        for (std::size_t i = 0; i + 1 < line.points.size(); i += 2) {
            work.push_back({static_cast<double>(line.points[i]), static_cast<double>(line.points[i + 1])});
        }

        for (auto& clipped : clipPolylineToTile(work)) {
            if (clipped.size() < 2) continue;
            const auto simplified = algorithm::contour::douglasPeucker(clipped, kSimplifyEpsilon);
            if (simplified.size() < 2) continue;
            const auto smoothed = algorithm::contour::chaikin(simplified, kChaikinIterations);
            if (smoothed.size() < 2) continue;

            mapbox::geometry::line_string<std::int16_t> ls;
            ls.reserve(smoothed.size());
            for (const auto& p : smoothed) {
                ls.push_back(
                    {static_cast<std::int16_t>(std::lround(p[0])), static_cast<std::int16_t>(std::lround(p[1]))});
            }

            // Round elevation to integer display units before emission.
            const double elevDisplay = algorithm::contour::metersToUnit(line.elevation, unit);
            const std::int64_t elev = static_cast<std::int64_t>(std::llround(elevDisplay));

            mapbox::feature::feature<std::int16_t> f;
            f.geometry = std::move(ls);
            f.properties["ele"] = elev;
            if (explicitLevelsMeters != nullptr) {
                // Explicit-levels mode: report which configured level this
                // line traces (0-based index), matching the web app's
                // `levelKey='level'` schema. `line.elevation` is an exact
                // copy of one of `explicitLevelsMeters`' entries (passed
                // straight through generateContoursAtLevels), so an exact
                // equality find is safe -- no floating-point tolerance
                // needed.
                const auto it = std::find(explicitLevelsMeters->begin(), explicitLevelsMeters->end(), line.elevation);
                const std::int64_t levelIndex =
                    (it != explicitLevelsMeters->end())
                        ? static_cast<std::int64_t>(std::distance(explicitLevelsMeters->begin(), it))
                        : 0;
                f.properties["level"] = levelIndex;
                f.properties["interval"] = static_cast<std::int64_t>(0);
                f.properties["major"] = false;
            } else {
                f.properties["interval"] = intervalRound;
                f.properties["major"] = (majorMod > 0) && (elev % majorMod == 0);
            }
            features.push_back(std::move(f));
        }
    }
    return features;
}

// Sutherland-Hodgman clip of a CLOSED polygon ring (last point implicitly
// connects back to the first, per polygons.hpp's convention) against the
// axis-aligned `[0, EXTENT] x [0, EXTENT]` tile bbox. Unlike the polyline
// clip above (which must preserve multiple disjoint open sub-polylines),
// a ring clipped against a rectangle stays a single (possibly degenerate/
// empty) ring, so the textbook 4-edge sequential clip applies directly.
// The subject ring may be non-convex in the same saddle-cell cases
// documented in polygons.cpp; Sutherland-Hodgman against a rectangle
// (itself convex) is still well-defined for a non-convex subject, it just
// may occasionally emit a ring with a self-touching vertex at a clip
// edge -- an acceptable, already-documented tradeoff.
std::vector<algorithm::contour::Point2D> clipRingToTileBBox(const std::vector<algorithm::contour::Point2D>& ring) {
    if (ring.size() < 3) return {};

    auto clipEdge = [](const std::vector<algorithm::contour::Point2D>& poly,
                       auto inside,
                       auto intersect) -> std::vector<algorithm::contour::Point2D> {
        std::vector<algorithm::contour::Point2D> out;
        if (poly.empty()) return out;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const auto& cur = poly[i];
            const auto& prev = poly[(i + poly.size() - 1) % poly.size()];
            const bool curIn = inside(cur);
            const bool prevIn = inside(prev);
            if (curIn != prevIn) out.push_back(intersect(prev, cur));
            if (curIn) out.push_back(cur);
        }
        return out;
    };

    constexpr double minX = 0.0;
    constexpr double minY = 0.0;
    const double maxX = static_cast<double>(util::EXTENT);
    const double maxY = static_cast<double>(util::EXTENT);

    auto lerp = [](const algorithm::contour::Point2D& a, const algorithm::contour::Point2D& b, double t) {
        return algorithm::contour::Point2D{a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])};
    };

    std::vector<algorithm::contour::Point2D> poly = ring;
    poly = clipEdge(
        poly,
        [&](const algorithm::contour::Point2D& p) { return p[0] >= minX; },
        [&](const algorithm::contour::Point2D& a, const algorithm::contour::Point2D& b) {
            return lerp(a, b, (minX - a[0]) / (b[0] - a[0]));
        });
    poly = clipEdge(
        poly,
        [&](const algorithm::contour::Point2D& p) { return p[0] <= maxX; },
        [&](const algorithm::contour::Point2D& a, const algorithm::contour::Point2D& b) {
            return lerp(a, b, (maxX - a[0]) / (b[0] - a[0]));
        });
    poly = clipEdge(
        poly,
        [&](const algorithm::contour::Point2D& p) { return p[1] >= minY; },
        [&](const algorithm::contour::Point2D& a, const algorithm::contour::Point2D& b) {
            return lerp(a, b, (minY - a[1]) / (b[1] - a[1]));
        });
    poly = clipEdge(
        poly,
        [&](const algorithm::contour::Point2D& p) { return p[1] <= maxY; },
        [&](const algorithm::contour::Point2D& a, const algorithm::contour::Point2D& b) {
            return lerp(a, b, (maxY - a[1]) / (b[1] - a[1]));
        });

    return poly;
}

// Convert filled-band polygons into MVT polygon features, appending them
// to an existing (possibly non-empty, since polygons/soundings share one
// combined feature collection -- see contour_tile.hpp's doc comment on
// why) feature collection.
//
// Per-feature properties: `min` / `max` -- the band's elevation bounds, in
// display units.
//
// Pipeline per ring (mirrors toFeatures' line pipeline, minus smoothing --
// fill polygons are already a coarse per-cell approximation, so Douglas-
// Peucker/Chaikin would just move vertices without adding real accuracy):
//   1. Scale from grid-fractional coordinates to tile-local using the same
//      `multiplier` the line path uses, then apply the same pixel-center
//      `shift` -- both parameters passed in identically to how the caller
//      already computes them for lines, so polygon and line geometry stay
//      registered to the same coordinate frame.
//   2. Clip to the [0, EXTENT] tile bbox (clipRingToTileBBox) so adjacent
//      tiles' fill polygons meet at the seam without gaps or overlaps.
//   3. Round to int16 MVT coords and emit.
void appendPolygonFeatures(mapbox::feature::feature_collection<std::int16_t>& features,
                           const std::vector<algorithm::contour::PolygonBand>& bands,
                           const algorithm::contour::UnitConfig& unit,
                           double multiplier,
                           double shift) {
    for (const auto& band : bands) {
        for (const auto& ring : band.rings) {
            if (ring.size() < 3) continue;

            std::vector<algorithm::contour::Point2D> scaled;
            scaled.reserve(ring.size());
            for (const auto& [x, y] : ring) {
                scaled.push_back({x * multiplier - shift, y * multiplier - shift});
            }

            const auto clipped = clipRingToTileBBox(scaled);
            if (clipped.size() < 3) continue;

            mapbox::geometry::linear_ring<std::int16_t> mvtRing;
            mvtRing.reserve(clipped.size() + 1);
            for (const auto& p : clipped) {
                mvtRing.push_back(
                    {static_cast<std::int16_t>(std::lround(p[0])), static_cast<std::int16_t>(std::lround(p[1]))});
            }
            mvtRing.push_back(mvtRing.front()); // close the ring (mapbox::geometry convention)

            mapbox::geometry::polygon<std::int16_t> polygon;
            polygon.push_back(std::move(mvtRing));

            mapbox::feature::feature<std::int16_t> f;
            f.geometry = std::move(polygon);
            f.properties["min"] = static_cast<std::int64_t>(std::llround(algorithm::contour::metersToUnit(band.minElevation, unit)));
            f.properties["max"] = static_cast<std::int64_t>(std::llround(algorithm::contour::metersToUnit(band.maxElevation, unit)));
            features.push_back(std::move(f));
        }
    }
}

// Convert spot-sounding grid samples into MVT point features, appending to
// the shared combined collection. Per-feature property: `ele` (elevation
// in display units). Soundings outside the tile bbox after scaling/shift
// are simply dropped -- a single point either lands in the tile or it
// doesn't; no clipping is meaningful the way it is for lines/polygons.
void appendSoundingFeatures(mapbox::feature::feature_collection<std::int16_t>& features,
                            const std::vector<algorithm::contour::SpotSounding>& soundings,
                            const algorithm::contour::UnitConfig& unit,
                            double multiplier,
                            double shift) {
    const double maxCoord = static_cast<double>(util::EXTENT);
    for (const auto& s : soundings) {
        const double x = s.x * multiplier - shift;
        const double y = s.y * multiplier - shift;
        if (x < 0.0 || x > maxCoord || y < 0.0 || y > maxCoord) continue;

        mapbox::feature::feature<std::int16_t> f;
        f.geometry = mapbox::geometry::point<std::int16_t>{static_cast<std::int16_t>(std::lround(x)),
                                                            static_cast<std::int16_t>(std::lround(y))};
        f.properties["ele"] =
            static_cast<std::int64_t>(std::llround(algorithm::contour::metersToUnit(s.elevation, unit)));
        features.push_back(std::move(f));
    }
}

} // namespace

ContourTile::ContourTile(const OverscaledTileID& id_,
                         std::string sourceID_,
                         const TileParameters& parameters,
                         TileObserver* observer_)
    : GeometryTile(id_, std::move(sourceID_), parameters, observer_) {}

void ContourTile::populateFromDEM(const RasterDEMTile& demTile, const ResolvedContourParams& params) {
    HillshadeBucket* bucket = demTile.getBucket();
    if (bucket == nullptr) return;
    const DEMData& dem = bucket->getDEMData();

    // Snapshot the buffered DEMData view on the render thread. The DEM
    // source uses a pixel-CENTER convention (verified empirically:
    // rightmost column of tile L and leftmost column of tile R differ
    // by ~one elevation cell, not zero), so each tile covers an
    // exclusive pixel range — sample 0 of tile L sits at world
    // west_L + 0.5·cellWidth, sample dim-1 at east_L - 0.5·cellWidth.
    //
    // We sample the entire (dim + 2·border) buffered view rather than
    // just the 1-pixel inner ring. With border = 2 the marching-squares
    // skirt extends two cells past the tile edge on every side, which
    // gives the downstream Douglas-Peucker / Chaikin smoothing more
    // context near the boundary so adjacent tiles' smoothed contours
    // share a tangent direction at the seam, not just an endpoint. The
    // same buffered grid is reused for polygon and sounding generation
    // (single DEM decode/snapshot serving all three outputs — see
    // contour_tile.hpp's doc comment). Adjacent tiles' overlapping output
    // regions are clipped during rendering, so the extra width costs only
    // a small per-tile generation step.
    //
    // Snapshot here on the render thread because backfillBorder mutates
    // the same image asynchronously when more neighbours arrive.
    const int dim = dem.dim;
    const int border = DEMData::border;
    const int width = dim + 2 * border;
    auto heights = std::make_shared<std::vector<std::int16_t>>();
    heights->reserve(static_cast<std::size_t>(width) * width);
    for (int y = -border; y < dim + border; y++) {
        for (int x = -border; x < dim + border; x++) {
            // Apply the source's `multiplier` here, once, at decode time --
            // every downstream algorithm (lines, polygons, soundings) then
            // sees already-flipped/scaled elevation values, and every
            // level/interval/band threshold the style author configured is
            // compared directly against them with no further adjustment
            // needed at generation or emission time.
            heights->push_back(static_cast<std::int16_t>(std::lround(dem.get(x, y) * params.multiplier)));
        }
    }

    Scheduler::GetBackground()->scheduleAndReplyValue(
        util::SimpleIdentity::Empty,
        [heights, width, dim, params]() {
            // Pixel-center coordinate mapping (shared by lines, polygons,
            // and soundings — all sample the same buffered grid). We want:
            //   alg index `border`     (sample 0, world west_L + 0.5cw) → tile-local +0.5cw
            //   alg index `border+dim-1` (sample dim-1, east_L − 0.5cw) → EXTENT − 0.5cw
            //   alg indices [0, border-1] and [border+dim, width-1]    → outside the tile,
            //                                                            in the neighbour's interior
            //
            // The line algorithms internally use
            //   multiplier = extent / (width - 1)
            // and emit output at multiplier·index. We want
            //   multiplier = EXTENT / dim
            // so each cell spans one tile-local cell width, no matter how
            // wide the buffered view is. That gives
            //   extent = EXTENT × (width - 1) / dim
            //
            // The boundary cell on each tile (heights[border-1..border]
            // on the west, heights[width-2..width-1] on the east, after
            // backfillBorder) straddles the tile edge using identical
            // neighbour data, so adjacent tiles' features meet exactly at
            // the seam.
            const int extent = static_cast<int>(std::lround(
                static_cast<double>(util::EXTENT) * static_cast<double>(width - 1) / static_cast<double>(dim)));
            const double multiplier = static_cast<double>(extent) / static_cast<double>(width - 1);

            // Shift so alg index `border` lands at +0.5cw (sample 0's
            // pixel-center position). At border = 1 this is −0.5cw.
            // At border = 2 it is −1.5cw — alg index 0 (outer west
            // border) sits one cell further west than the inner border
            // cell did before. This shift is already in tile-local
            // (post-multiplier) units, so it's subtracted directly from
            // already-scaled coordinates (lines) or from grid-fractional
            // coordinates multiplied by `multiplier` first (polygons,
            // soundings) — see appendPolygonFeatures / appendSoundingFeatures.
            const double halfCellPx = 0.5 * static_cast<double>(util::EXTENT) / static_cast<double>(dim);
            const double shift = halfCellPx * static_cast<double>(2 * DEMData::border - 1);

            mapbox::feature::feature_collection<std::int16_t> combined;

            // --- Lines: exactly one of explicit-levels / constant-interval mode ---
            const bool explicitLevels = !params.lineLevelsMeters.empty();
            const auto lines = explicitLevels
                                    ? algorithm::contour::generateContoursAtLevels(
                                          *heights, width, width, params.lineLevelsMeters, extent)
                                    : algorithm::contour::generateContours(
                                          *heights, width, width, algorithm::contour::ContourThresholds{params.intervalMeters, extent});

            std::vector<algorithm::contour::ContourLineString> shiftedLines;
            shiftedLines.reserve(lines.size());
            for (const auto& line : lines) {
                algorithm::contour::ContourLineString s;
                s.elevation = line.elevation;
                s.points.reserve(line.points.size());
                for (std::size_t i = 0; i + 1 < line.points.size(); i += 2) {
                    s.points.push_back(static_cast<std::int32_t>(std::lround(line.points[i] - shift)));
                    s.points.push_back(static_cast<std::int32_t>(std::lround(line.points[i + 1] - shift)));
                }
                shiftedLines.push_back(std::move(s));
            }
            auto lineFeatures = toFeatures(shiftedLines,
                                          params.unit,
                                          params.intervalDisplayUnits,
                                          params.majorMultiplier,
                                          explicitLevels ? &params.lineLevelsMeters : nullptr);
            combined.reserve(lineFeatures.size());
            for (auto& f : lineFeatures) combined.push_back(std::move(f));

            // --- Polygons (optional) ---
            if (!params.polygonLevelsMeters.empty()) {
                const auto bands = algorithm::contour::generatePolygons(*heights, width, width, params.polygonLevelsMeters);
                appendPolygonFeatures(combined, bands, params.unit, multiplier, shift);
            }

            // --- Spot soundings (optional) ---
            if (params.spotGridSpacing > 0) {
                const auto soundings = algorithm::contour::generateSoundings(
                    *heights, width, width, params.spotGridSpacing, params.spotSortOrder);
                appendSoundingFeatures(combined, soundings, params.unit, multiplier, shift);
            }

            return combined;
        },
        [self = weakFactory.makeWeakPtr(), this](mapbox::feature::feature_collection<std::int16_t> features) {
            if (auto guard = self.lock(); self) {
                setData(std::make_unique<GeoJSONTileData>(std::move(features)));
            }
        });
}

} // namespace mbgl
