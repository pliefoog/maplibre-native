#pragma once

#include <mbgl/algorithm/contour/soundings.hpp>
#include <mbgl/algorithm/contour/units.hpp>
#include <mbgl/tile/geometry_tile.hpp>

#include <mapbox/std/weak.hpp>

#include <cstdint>
#include <vector>

namespace mbgl {

class RasterDEMTile;
class TileParameters;

// Resolved-for-this-tile's-zoom contour generation parameters. Bundles both
// the line-generation mode (constant interval OR explicit irregular
// levels -- see algorithm/contour/levels.hpp for why these are mutually
// exclusive) and the optional polygon-fill / spot-sounding parameters, all
// already resolved from the source's per-zoom schedules by
// RenderContourSource before calling `populateFromDEM`.
struct ResolvedContourParams {
    // Line generation: exactly one of these two is meaningful, selected by
    // `lineLevelsMeters.empty()`. Both are in METRES (DEM native unit) --
    // `unit` below only affects the *display* value baked into emitted
    // `ele`/`interval` feature properties, not which cells produce a line.
    double intervalMeters = 0.0;         // constant-interval mode (existing)
    std::vector<double> lineLevelsMeters; // explicit-levels mode (new); non-empty = active
    // Display-unit value of intervalMeters, needed for the `interval`
    // feature property in constant-interval mode (kept separate rather than
    // re-derived from intervalMeters + unit at emission time, since that
    // conversion already happened once in RenderContourSource and floating-
    // point round-trip through it a second time could disagree by an ULP
    // with the value RenderContourSource itself would compute).
    double intervalDisplayUnits = 0.0;

    // Fill polygons (optional). Empty = disabled. Values in metres.
    std::vector<double> polygonLevelsMeters;

    // Spot soundings (optional). 0 = disabled.
    int spotGridSpacing = 0;
    algorithm::contour::SpotSortOrder spotSortOrder = algorithm::contour::SpotSortOrder::Ascending;

    std::int64_t majorMultiplier = 0;
    algorithm::contour::UnitConfig unit;
};

// A vector-tile that renders contour-line, fill-polygon, and spot-sounding
// features for a single (z, x, y), all derived from the same upstream
// raster-dem tile.
//
// `populateFromDEM` snapshots the buffered (dim + 2·border) DEM image on
// the render thread, dispatches marching-squares (lines and, if
// configured, polygons) plus grid-sampling (soundings) to the background
// scheduler -- all reading the SAME decoded height buffer, one DEM decode
// serving all three outputs -- and on reply (render thread) converts the
// results into a single combined MVT feature collection.
//
// SINGLE COMBINED LAYER, NOT THREE: `GeoJSONTileData` (which this tile
// emits through) is explicitly single-layer -- "a GeoJSON tile can only
// have one layer, and it is always returned regardless of which layer is
// requested" (its own doc comment). So lines, polygons, and points are
// emitted into ONE feature collection; consuming styles differentiate them
// by geometry-type filter (`['==', ['geometry-type'], 'Polygon']` etc.)
// instead of by source-layer name -- exactly the pattern this app's own
// `channel-safety-area` web layer already uses. A `WeakPtrFactory` guards
// against the tile being destroyed (panned away) before the background
// work completes.
//
// Tile-edge continuity is preserved by sampling DEMData's neighbour-
// border (populated by `RenderRasterDEMSource` via `backfillBorder` as
// adjacent DEM tiles arrive). Lines and polygon rings crossing the tile
// edge are clipped against the tile bbox in `toFeatures` so adjacent
// tiles' geometry meets exactly at the seam.
class ContourTile final : public GeometryTile {
public:
    ContourTile(const OverscaledTileID&, std::string sourceID, const TileParameters&, TileObserver* = nullptr);

    // Populate the tile from an upstream DEM tile that just finished parsing.
    void populateFromDEM(const RasterDEMTile& demTile, const ResolvedContourParams& params);

private:
    mapbox::base::WeakPtrFactory<ContourTile> weakFactory{this};
    // Do not add members here, see `WeakPtrFactory`.
};

} // namespace mbgl

