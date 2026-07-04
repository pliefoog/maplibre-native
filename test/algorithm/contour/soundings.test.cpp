#include <mbgl/algorithm/contour/soundings.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace mbgl::algorithm::contour;

TEST(SoundingsGeneration, BasicGridSampledAtSpacing) {
    // 4x4 grid, spacing 2 -> samples at (row,col) offsets 1 and 3 in each
    // axis -> 2x2 = 4 soundings.
    std::vector<std::int16_t> grid = {
        1, 2, 3, 4, //
        2, 3, 4, 5, //
        3, 4, 5, 6, //
        4, 5, 6, 7, //
    };
    auto s = generateSoundings(grid, 4, 4, 2, SpotSortOrder::Ascending);
    EXPECT_EQ(s.size(), 4u);
    for (std::size_t i = 1; i < s.size(); ++i) {
        EXPECT_LE(s[i - 1].elevation, s[i].elevation);
    }
}

TEST(SoundingsGeneration, DescendingSortOrder) {
    std::vector<std::int16_t> grid = {
        1, 2, 3, 4, //
        2, 3, 4, 5, //
        3, 4, 5, 6, //
        4, 5, 6, 7, //
    };
    auto s = generateSoundings(grid, 4, 4, 2, SpotSortOrder::Descending);
    ASSERT_EQ(s.size(), 4u);
    for (std::size_t i = 1; i < s.size(); ++i) {
        EXPECT_GE(s[i - 1].elevation, s[i].elevation);
    }
}

TEST(SoundingsGeneration, ZeroSpacingProducesNoSoundings) {
    std::vector<std::int16_t> grid = {1, 2, 3, 4};
    auto s = generateSoundings(grid, 2, 2, 0, SpotSortOrder::Ascending);
    EXPECT_TRUE(s.empty());
}

TEST(SoundingsGeneration, NegativeSpacingProducesNoSoundings) {
    std::vector<std::int16_t> grid = {1, 2, 3, 4};
    auto s = generateSoundings(grid, 2, 2, -1, SpotSortOrder::Ascending);
    EXPECT_TRUE(s.empty());
}

TEST(SoundingsGeneration, SamplesEveryPointAtSpacingOne) {
    // spacing 1 -> no points skipped -> one sounding per grid sample.
    std::vector<std::int16_t> grid = {
        1, 2, //
        3, 4, //
    };
    auto s = generateSoundings(grid, 2, 2, 1, SpotSortOrder::Ascending);
    EXPECT_EQ(s.size(), 4u);
}

TEST(SoundingsGeneration, SpacingLargerThanGridProducesNoSoundings) {
    // spacing 10 over a 2x2 grid: first sample offset (spacing/2 = 5) is
    // already past both grid dimensions, so no sample point exists. This is
    // the correct, honest behavior of pure regular-interval sampling (as
    // opposed to guaranteeing at least one "center" sample) — in practice
    // spacing is always much smaller than the tile's grid dimensions
    // (e.g. spacing=32 against a 256+-sample DEM tile), so this is an edge
    // case, not a real-world configuration.
    std::vector<std::int16_t> grid = {
        5, 6, //
        7, 8, //
    };
    auto s = generateSoundings(grid, 2, 2, 10, SpotSortOrder::Ascending);
    EXPECT_TRUE(s.empty());
}

TEST(SoundingsGeneration, SampledCoordinatesAreOffsetToCellCenters) {
    // spacing 4 over a 4x4 grid -> exactly one sample, at (2,2) (spacing/2),
    // not (0,0).
    std::vector<std::int16_t> grid(16, 3);
    auto s = generateSoundings(grid, 4, 4, 4, SpotSortOrder::Ascending);
    ASSERT_EQ(s.size(), 1u);
    EXPECT_EQ(s[0].x, 2.0);
    EXPECT_EQ(s[0].y, 2.0);
}

TEST(SoundingsGeneration, EmptyGridDimensionsProduceNoSoundings) {
    std::vector<std::int16_t> grid = {};
    auto s = generateSoundings(grid, 0, 0, 1, SpotSortOrder::Ascending);
    EXPECT_TRUE(s.empty());
}
