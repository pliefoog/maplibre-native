#include <mbgl/algorithm/contour/polygons.hpp>

#include <algorithm>
#include <array>

namespace mbgl {
namespace algorithm {
namespace contour {

namespace {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct ClippedVertex {
    Vec2 p;
    double value = 0.0;
};

// Textbook Sutherland–Hodgman polygon clip, generalized from clipping
// against a straight edge to clipping against a scalar-field threshold:
// each vertex carries the field value that was known at the point it was
// created (either an original grid corner, or linearly interpolated along
// an edge during a *previous* clip pass). Because the value at every
// surviving/created vertex is tracked, clip passes compose cleanly — this
// is exactly how a two-threshold "band" (isoband) is built: clip once
// keeping values >= lower, then clip that result again keeping
// values < upper.
//
// Vertices created by *this* clip are exactly ON the current threshold, so
// their `value` is set to `threshold` — correct for chaining into a further
// clip against a different threshold, since no additional interpolation
// happens along the artificial cut edge introduced by clipping (consistent
// with marching squares' own edge-only linear-interpolation assumption;
// see isolines.cpp's `edgePosition`).
//
// LIMITATION (documented, matches isolines.cpp's existing precedent): a
// cell whose four corners span all three states relative to two thresholds
// in a "checkerboard" (saddle) arrangement can, after two sequential clips,
// produce a self-intersecting ("bowtie") polygon rather than two separate
// simple polygons. isolines.cpp accepts the equivalent ambiguity for line
// generation with a comment: "sufficient for typical DEM data where saddle
// points are sparse ... center-average disambiguation is a future
// improvement." The same tradeoff is made here.
std::vector<ClippedVertex> clipPolygonByThreshold(const std::vector<ClippedVertex>& poly,
                                                   double threshold,
                                                   bool keepAboveOrEqual) {
    if (poly.empty()) return {};
    std::vector<ClippedVertex> out;
    out.reserve(poly.size() + 1);
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const ClippedVertex& cur = poly[i];
        const ClippedVertex& nxt = poly[(i + 1) % n];
        const bool curIn = keepAboveOrEqual ? (cur.value >= threshold) : (cur.value < threshold);
        const bool nxtIn = keepAboveOrEqual ? (nxt.value >= threshold) : (nxt.value < threshold);
        if (curIn) {
            out.push_back(cur);
        }
        if (curIn != nxtIn) {
            const double denom = nxt.value - cur.value;
            // denom == 0 would mean cur/nxt disagree on being "in" while
            // having equal values, which the boolean tests above make
            // impossible (both booleans are pure functions of value vs
            // threshold), so this is unreachable in practice; guarded
            // defensively rather than assumed.
            const double t = denom != 0.0 ? (threshold - cur.value) / denom : 0.0;
            out.push_back({{cur.p.x + t * (nxt.p.x - cur.p.x), cur.p.y + t * (nxt.p.y - cur.p.y)}, threshold});
        }
    }
    return out;
}

// Clip a single grid cell's quad (corners in tile-local coordinates, values
// at those corners) down to the polygon covering [lower, upper).
std::vector<std::pair<double, double>> cellBandPolygon(
    double tl, double tr, double br, double bl, int col, int row, double lower, double upper) {
    std::vector<ClippedVertex> quad = {
        {{static_cast<double>(col), static_cast<double>(row)}, tl},
        {{static_cast<double>(col + 1), static_cast<double>(row)}, tr},
        {{static_cast<double>(col + 1), static_cast<double>(row + 1)}, br},
        {{static_cast<double>(col), static_cast<double>(row + 1)}, bl},
    };
    auto clipped = clipPolygonByThreshold(quad, lower, /*keepAboveOrEqual=*/true);
    clipped = clipPolygonByThreshold(clipped, upper, /*keepAboveOrEqual=*/false);

    std::vector<std::pair<double, double>> ring;
    if (clipped.size() < 3) return ring; // degenerate: point or line, not an area
    ring.reserve(clipped.size());
    for (const auto& v : clipped) ring.emplace_back(v.p.x, v.p.y);
    return ring;
}

} // namespace

std::vector<PolygonBand> generatePolygons(std::span<const std::int16_t> heights,
                                           int width,
                                           int height,
                                           const std::vector<double>& levels) {
    std::vector<PolygonBand> bands;
    if (levels.size() < 2 || width < 2 || height < 2) return bands;
    if (static_cast<std::size_t>(width) * static_cast<std::size_t>(height) > heights.size()) return bands;

    bands.reserve(levels.size() - 1);
    for (std::size_t i = 0; i + 1 < levels.size(); ++i) {
        PolygonBand band;
        band.minElevation = levels[i];
        band.maxElevation = levels[i + 1];
        bands.push_back(std::move(band));
    }

    for (int row = 0; row < height - 1; ++row) {
        for (int col = 0; col < width - 1; ++col) {
            const double tl = heights[static_cast<std::size_t>(row) * width + col];
            const double tr = heights[static_cast<std::size_t>(row) * width + col + 1];
            const double bl = heights[static_cast<std::size_t>(row + 1) * width + col];
            const double br = heights[static_cast<std::size_t>(row + 1) * width + col + 1];

            const double cellMin = std::min({tl, tr, bl, br});
            const double cellMax = std::max({tl, tr, bl, br});

            for (auto& band : bands) {
                // Cheap reject: this cell's value range doesn't overlap the band at all.
                if (cellMax < band.minElevation || cellMin >= band.maxElevation) continue;

                auto ring = cellBandPolygon(tl, tr, br, bl, col, row, band.minElevation, band.maxElevation);
                if (!ring.empty()) band.rings.push_back(std::move(ring));
            }
        }
    }

    return bands;
}

} // namespace contour
} // namespace algorithm
} // namespace mbgl
