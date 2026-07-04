#pragma once

#include <mbgl/algorithm/contour/intervals.hpp>
#include <mbgl/algorithm/contour/levels.hpp>
#include <mbgl/algorithm/contour/soundings.hpp>
#include <mbgl/algorithm/contour/units.hpp>
#include <mbgl/style/source.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace mbgl {
namespace style {

// Configuration for a `style::ContourSource`. Mirrors the `contour` source
// type from the MapLibre style spec proposal (maplibre-style-spec#583):
//
//   {
//     "type": "contour",
//     "source": "dem",
//     "intervals": [200, 12, 100, 14, 50, 15, 20],
//     "unit": "feet",
//     "majorMultiplier": [5, 14, 4, 15, 5],
//     "overzoom": 1
//   }
struct ContourSourceOptions {
    // ID of the upstream `raster-dem` source to derive contours from.
    std::string sourceID;
    // Per-zoom contour interval schedule (see algorithm/contour/intervals.hpp).
    // Mutually exclusive with `lineLevels` at the style level (the parser
    // requires exactly one of `intervals` / `lineLevels`); a source that
    // sets neither fails to parse. When `lineLevels` is set, `intervals`
    // is left at its default (empty, invalid) and unused for line
    // generation.
    algorithm::contour::IntervalSchedule intervals;
    // Explicit, deliberately IRREGULAR per-zoom elevation breakpoints for
    // contour LINES -- e.g. [0,2,5,10,20,50,100,250,500,1000,2000,3000,
    // 4000,5000], matching paper-chart/IHO depth-band convention (fine
    // near the surface, coarse at depth). A constant-width `intervals`
    // schedule cannot reproduce this; see algorithm/contour/levels.hpp for
    // why these are separate schedule types. Optional: absent means use
    // `intervals` instead (the two are mutually exclusive at the style
    // level -- see the parser in style/conversion/source.cpp).
    std::optional<algorithm::contour::LevelSchedule> lineLevels;
    // Explicit per-zoom elevation-BAND boundaries for the filled polygon
    // layer (adjacent pairs of this list define bands, e.g.
    // [-100000,0,2,5,10,20,50,100] -> bands [-100000,0), [0,2), [2,5), ...).
    // Optional: absent disables polygon-fill generation entirely (the
    // source still emits contour lines / soundings per their own config).
    std::optional<algorithm::contour::LevelSchedule> polygonLevels;
    // Grid-sampling interval (in DEM grid samples) for spot-sounding point
    // features. 0 (default) disables spot-sounding generation.
    int spotGridSpacing = 0;
    // Sort order for the emitted spot-sounding point features (some
    // renderers rely on paint order for label collision priority -- see
    // ChartMapContent.web.tsx's `spot-soundings-shallow` layer's
    // `symbol-sort-key`).
    algorithm::contour::SpotSortOrder spotSortOrder = algorithm::contour::SpotSortOrder::Ascending;
    // Source-layer names for the three feature kinds this source can emit.
    // Defaults match this app's generate-map-style.js schema
    // (`contourLayer: 'contours'`, `polygonLayer: 'bathymetry'`,
    // `spotLayer: 'soundings'`).
    std::string contourLayer = "contours";
    std::string polygonLayer = "bathymetry";
    std::string spotLayer = "soundings";
    // Multiply every decoded DEM elevation sample by this factor before
    // running any generation algorithm (lines, polygons, or soundings).
    // Matches the web app's `CONTOUR_PARAMS.multiplier` (e.g. -1 so ocean
    // depth reads as a positive number instead of the DEM's natural
    // negative-below-sea-level convention). Applied once, at DEM decode
    // time (see ContourTile::populateFromDEM), not per-algorithm.
    double multiplier = 1.0;
    // Display unit for emitted `ele` / `interval` feature attributes. Defaults
    // to metres; the underlying DEM is always interpreted as metres.
    algorithm::contour::UnitConfig unit;
    // Per-zoom step-by-zoom schedule of "every Nth contour line is tagged
    // `major: true`" multipliers. Same shape as `intervals` (odd-length
    // step-by-zoom array). The per-tile multiplier resolves to a single
    // positive integer; a line at elevation `e` with tile-zoom interval `i`
    // and resolved multiplier `m` emits `major: true` iff `e % (i*m) == 0`.
    // Default: a single-output schedule of `{5}` (every 5th line is major
    // at every zoom).
    algorithm::contour::IntervalSchedule majorMultiplier{{5.0}};
    // Number of zoom levels to overzoom from the upstream DEM source. 0 means
    // sample the DEM at the contour-tile zoom directly.
    std::uint8_t overzoom = 0;
};

// Source-on-source: reads decoded DEM data from an upstream `raster-dem`
// source's tile pyramid and emits contour-line features. This class is the
// style-spec-side handle that the parser produces; the render-side
// integration lives in `RenderContourSource`.
class ContourSource final : public Source {
public:
    ContourSource(std::string id, ContourSourceOptions options);
    ~ContourSource() override;

    const std::string& getDEMSourceID() const;
    const algorithm::contour::IntervalSchedule& getIntervals() const;
    const std::optional<algorithm::contour::LevelSchedule>& getLineLevels() const;
    const std::optional<algorithm::contour::LevelSchedule>& getPolygonLevels() const;
    int getSpotGridSpacing() const;
    algorithm::contour::SpotSortOrder getSpotSortOrder() const;
    const std::string& getContourLayer() const;
    const std::string& getPolygonLayer() const;
    const std::string& getSpotLayer() const;
    double getMultiplier() const;
    const algorithm::contour::UnitConfig& getUnit() const;
    const algorithm::contour::IntervalSchedule& getMajorMultiplier() const;
    std::uint8_t getOverzoom() const;

    class Impl;
    const Impl& impl() const;

    void loadDescription(FileSource&) final;

    bool supportsLayerType(const mbgl::style::LayerTypeInfo*) const override;

    mapbox::base::WeakPtr<Source> makeWeakPtr() override { return weakFactory.makeWeakPtr(); }

protected:
    Mutable<Source::Impl> createMutable() const noexcept final;

private:
    mapbox::base::WeakPtrFactory<Source> weakFactory{this};
    // Do not add members here, see `WeakPtrFactory`.
};

template <>
inline bool Source::is<ContourSource>() const {
    return getType() == SourceType::Contour;
}

} // namespace style
} // namespace mbgl
