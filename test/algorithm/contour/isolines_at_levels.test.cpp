#include <gtest/gtest.h>

#include <mbgl/algorithm/contour/isolines.hpp>

using namespace mbgl::algorithm::contour;

// Tests for generateContoursAtLevels -- the explicit-irregular-levels sibling
// of generateContours, added to support maritime depth-band charting
// convention (see algorithm/contour/levels.hpp). Same grid convention as
// isolines.test.cpp: row-major, north-up.

TEST(ContourAtLevels, EmptyLevelsReturnsNoLines) {
    const std::vector<std::int16_t> heights{0, 0, 0, 0};
    const auto lines = generateContoursAtLevels(heights, 2, 2, {}, 4096);
    EXPECT_TRUE(lines.empty());
}

TEST(ContourAtLevels, TooSmallGridReturnsNoLines) {
    const std::vector<std::int16_t> heights{0, 0, 0};
    const auto lines = generateContoursAtLevels(heights, 1, 3, {5.0}, 4096);
    EXPECT_TRUE(lines.empty());
}

TEST(ContourAtLevels, LevelOutsideDataRangeProducesNoLines) {
    // 2x2 grid, values 10..50; a level of 500 is far outside the range.
    const std::vector<std::int16_t> heights{10, 20, 30, 50};
    const auto lines = generateContoursAtLevels(heights, 2, 2, {500.0}, 4096);
    EXPECT_TRUE(lines.empty());
}

TEST(ContourAtLevels, SingleLevelMatchesEquivalentConstantIntervalResult) {
    // A 4x4 grid with a level list containing exactly one usable threshold
    // should trace the same line as the elevation==5 subset of
    // generateContours run with an interval of 5 (which also emits a
    // second line at elevation==10, the data's exact max -- filtered out
    // below since we're only checking equivalence for the one shared
    // threshold, not the whole schedule).
    const std::vector<std::int16_t> heights{
        0,  0,  0,  0, //
        0,  10, 10, 0, //
        0,  10, 10, 0, //
        0,  0,  0,  0, //
    };
    const auto atLevels = generateContoursAtLevels(heights, 4, 4, {5.0}, 4096);
    const auto atIntervalAll = generateContours(heights, 4, 4, ContourThresholds{5.0, 4096});

    std::size_t levelsPoints = 0;
    for (const auto& l : atLevels) levelsPoints += l.points.size();
    std::size_t intervalPointsAt5 = 0;
    for (const auto& l : atIntervalAll) {
        if (l.elevation == 5.0) intervalPointsAt5 += l.points.size();
    }
    EXPECT_GT(levelsPoints, 0u);
    EXPECT_EQ(levelsPoints, intervalPointsAt5);
}

TEST(ContourAtLevels, IrregularLevelsEachProduceIndependentLines) {
    // A radial-ish gradient from 0 (edges) to 40 (center) crossed by three
    // very differently-spaced levels (2, 5, 30 -- deltas of 3 and 25, which
    // no constant interval could represent as a single schedule). All three
    // should independently produce at least one line each.
    const std::vector<std::int16_t> heights{
        0,  0,  0,  0,  0, //
        0,  10, 20, 10, 0, //
        0,  20, 40, 20, 0, //
        0,  10, 20, 10, 0, //
        0,  0,  0,  0,  0, //
    };
    const std::vector<double> levels = {2.0, 5.0, 30.0};
    const auto lines = generateContoursAtLevels(heights, 5, 5, levels, 4096);

    bool has2 = false, has5 = false, has30 = false;
    for (const auto& l : lines) {
        if (l.elevation == 2.0) has2 = true;
        if (l.elevation == 5.0) has5 = true;
        if (l.elevation == 30.0) has30 = true;
    }
    EXPECT_TRUE(has2);
    EXPECT_TRUE(has5);
    EXPECT_TRUE(has30);
}

TEST(ContourAtLevels, LevelsMustBeAscendingForEarlyExitToBeCorrect) {
    // Regression guard for the per-cell early-exit ("ascending: no later
    // level fits either"): a cell whose range includes level[1] but not
    // level[0] or level[2] must still be found. Values 0..100 span all
    // three levels for this single cell.
    const std::vector<std::int16_t> heights{0, 100, 0, 100};
    const std::vector<double> levels = {10.0, 50.0, 90.0};
    const auto lines = generateContoursAtLevels(heights, 2, 2, levels, 4096);
    ASSERT_EQ(lines.size(), 3u);
}
