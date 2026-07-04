#include <mbgl/algorithm/contour/polygons.hpp>

#include <gtest/gtest.h>

#include <cmath>

using namespace mbgl::algorithm::contour;

namespace {

double ringArea(const std::vector<std::pair<double, double>>& ring) {
    // Shoelace formula.
    double area = 0.0;
    const std::size_t n = ring.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& [x1, y1] = ring[i];
        const auto& [x2, y2] = ring[(i + 1) % n];
        area += x1 * y2 - x2 * y1;
    }
    return std::abs(area) / 2.0;
}

double totalArea(const std::vector<PolygonBand>& bands) {
    double total = 0.0;
    for (const auto& band : bands) {
        for (const auto& ring : band.rings) total += ringArea(ring);
    }
    return total;
}

} // namespace

TEST(PolygonGeneration, EmptyLevelsProducesNoBands) {
    std::vector<std::int16_t> grid = {1, 2, 2, 3};
    std::vector<double> levels = {};
    auto bands = generatePolygons(grid, 2, 2, levels);
    EXPECT_TRUE(bands.empty());
}

TEST(PolygonGeneration, SingleLevelProducesNoBands) {
    std::vector<std::int16_t> grid = {1, 2, 2, 3};
    std::vector<double> levels = {5.0};
    auto bands = generatePolygons(grid, 2, 2, levels);
    EXPECT_TRUE(bands.empty());
}

TEST(PolygonGeneration, UniformGridEntirelyInsideSingleBandCoversWholeCell) {
    // 2x2 grid (one cell), all corners = 5, band [0, 10) fully contains it.
    std::vector<std::int16_t> grid = {5, 5, 5, 5};
    std::vector<double> levels = {0.0, 10.0};
    auto bands = generatePolygons(grid, 2, 2, levels);
    ASSERT_EQ(bands.size(), 1u);
    EXPECT_EQ(bands[0].minElevation, 0.0);
    EXPECT_EQ(bands[0].maxElevation, 10.0);
    ASSERT_EQ(bands[0].rings.size(), 1u);
    EXPECT_NEAR(ringArea(bands[0].rings[0]), 1.0, 1e-9); // one full unit cell
}

TEST(PolygonGeneration, UniformGridOutsideBandProducesNoRings) {
    std::vector<std::int16_t> grid = {5, 5, 5, 5};
    std::vector<double> levels = {100.0, 200.0};
    auto bands = generatePolygons(grid, 2, 2, levels);
    ASSERT_EQ(bands.size(), 1u);
    EXPECT_TRUE(bands[0].rings.empty());
}

TEST(PolygonGeneration, HalfCellStraddlingUpperThresholdHasHalfArea) {
    // Left column = 0, right column = 10. Band [0, 5) should cover
    // (approximately) the left half of the unit cell — the threshold
    // crossing (value == 5) falls exactly at the cell's horizontal midline.
    std::vector<std::int16_t> grid = {
        0, 10, // row 0: TL, TR
        0, 10, // row 1: BL, BR
    };
    std::vector<double> levels = {0.0, 5.0};
    auto bands = generatePolygons(grid, 2, 2, levels);
    ASSERT_EQ(bands.size(), 1u);
    ASSERT_EQ(bands[0].rings.size(), 1u);
    EXPECT_NEAR(ringArea(bands[0].rings[0]), 0.5, 1e-9);
}

TEST(PolygonGeneration, MultipleBandsPartitionFullRangeWithoutGapsOrOverlap) {
    // A cell whose four corners span 1..9. Bands [0,2],[2,5],[5,10] exactly
    // cover the full observed value range with no gaps — so the combined
    // area of all three bands' rings for this cell must equal exactly the
    // area of the whole unit cell (1.0). This is the key correctness
    // invariant: bands partition, they don't lose or double-count area.
    std::vector<std::int16_t> grid = {
        1, 3, // TL, TR
        4, 9, // BL, BR
    };
    std::vector<double> levels = {0.0, 2.0, 5.0, 10.0};
    auto bands = generatePolygons(grid, 2, 2, levels);
    ASSERT_EQ(bands.size(), 3u);
    EXPECT_NEAR(totalArea(bands), 1.0, 1e-9);
}

TEST(PolygonGeneration, LargerGridSumsToTotalGridAreaAcrossAllBands) {
    // 3x3 sample grid -> 2x2 = 4 cells, values spanning 1..9. Bands chosen
    // to span the full range with no gaps ([0,10) split into quarters).
    // Total polygon area across every band must equal 4.0 (the grid's total
    // cell area), regardless of how the 4 cells individually partition
    // across bands.
    std::vector<std::int16_t> grid = {
        1, 3, 7, //
        2, 5, 8, //
        4, 6, 9, //
    };
    std::vector<double> levels = {0.0, 2.5, 5.0, 7.5, 10.0};
    auto bands = generatePolygons(grid, 3, 3, levels);
    ASSERT_EQ(bands.size(), 4u);
    EXPECT_NEAR(totalArea(bands), 4.0, 1e-6);
}

TEST(PolygonGeneration, BandsWithNoIntersectingCellsAreStillPresentButEmpty) {
    // Grid entirely within [0,2); the [5,10) band should still appear in the
    // output (stable 1:1 mapping to input level pairs) but with no rings.
    std::vector<std::int16_t> grid = {1, 1, 1, 1};
    std::vector<double> levels = {0.0, 2.0, 5.0, 10.0};
    auto bands = generatePolygons(grid, 2, 2, levels);
    ASSERT_EQ(bands.size(), 3u);
    EXPECT_FALSE(bands[0].rings.empty());
    EXPECT_TRUE(bands[1].rings.empty());
    EXPECT_TRUE(bands[2].rings.empty());
}

TEST(PolygonGeneration, TooSmallGridProducesNoBands) {
    std::vector<std::int16_t> grid = {1, 2, 3};
    std::vector<double> levels = {0.0, 10.0};
    auto bands = generatePolygons(grid, 1, 3, levels); // width < 2
    EXPECT_TRUE(bands.empty());
}
