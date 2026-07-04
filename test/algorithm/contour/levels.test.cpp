#include <mbgl/algorithm/contour/levels.hpp>

#include <gtest/gtest.h>

using namespace mbgl::algorithm::contour;

TEST(LevelSchedule, EmptyScheduleIsInvalidAndResolvesEmpty) {
    LevelSchedule schedule;
    EXPECT_FALSE(isValid(schedule));
    EXPECT_TRUE(resolveLevels(schedule, 10.0).empty());
}

TEST(LevelSchedule, SingleEntryUsedAtAnyZoomAtOrAboveItsBreakpoint) {
    LevelSchedule schedule;
    schedule.entries.push_back({5.0, {0.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0, 250.0, 500.0, 1000.0}});
    ASSERT_TRUE(isValid(schedule));

    EXPECT_EQ(resolveLevels(schedule, 5.0), schedule.entries[0].second);
    EXPECT_EQ(resolveLevels(schedule, 12.0), schedule.entries[0].second);
}

TEST(LevelSchedule, BelowLowestBreakpointFallsBackToSmallestBreakpointEntry) {
    // Matches the app's real bathymetry schedule: single entry at zoom 5.
    // A tile requested at zoom 2 (below the lowest configured breakpoint)
    // should still get *some* bands, not none.
    LevelSchedule schedule;
    schedule.entries.push_back({5.0, {-100000.0, 0.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0}});
    ASSERT_TRUE(isValid(schedule));

    EXPECT_EQ(resolveLevels(schedule, 2.0), schedule.entries[0].second);
}

TEST(LevelSchedule, MultipleBreakpointsPickTheLargestNotExceedingZoom) {
    LevelSchedule schedule;
    schedule.entries.push_back({0.0, {0.0, 10.0, 100.0}});       // coarse, low zoom
    schedule.entries.push_back({8.0, {0.0, 2.0, 5.0, 10.0}});    // finer, mid zoom
    schedule.entries.push_back({14.0, {0.0, 1.0, 2.0, 5.0}});    // finest, high zoom
    ASSERT_TRUE(isValid(schedule));

    EXPECT_EQ(resolveLevels(schedule, 0.0), schedule.entries[0].second);
    EXPECT_EQ(resolveLevels(schedule, 5.0), schedule.entries[0].second);
    EXPECT_EQ(resolveLevels(schedule, 8.0), schedule.entries[1].second);
    EXPECT_EQ(resolveLevels(schedule, 10.0), schedule.entries[1].second);
    EXPECT_EQ(resolveLevels(schedule, 14.0), schedule.entries[2].second);
    EXPECT_EQ(resolveLevels(schedule, 20.0), schedule.entries[2].second);
}

TEST(LevelSchedule, NonStrictlyIncreasingBreakpointsIsInvalid) {
    LevelSchedule schedule;
    schedule.entries.push_back({5.0, {0.0, 10.0}});
    schedule.entries.push_back({5.0, {0.0, 5.0}}); // duplicate breakpoint
    EXPECT_FALSE(isValid(schedule));
}

TEST(LevelSchedule, DescendingBreakpointsIsInvalid) {
    LevelSchedule schedule;
    schedule.entries.push_back({10.0, {0.0, 10.0}});
    schedule.entries.push_back({5.0, {0.0, 5.0}}); // out of order
    EXPECT_FALSE(isValid(schedule));
}

TEST(LevelSchedule, SingleLevelEntryIsInvalid) {
    // A single value can't define even one band or line threshold pair.
    LevelSchedule schedule;
    schedule.entries.push_back({5.0, {10.0}});
    EXPECT_FALSE(isValid(schedule));
}

TEST(LevelSchedule, NonStrictlyIncreasingLevelsIsInvalid) {
    LevelSchedule schedule;
    schedule.entries.push_back({5.0, {0.0, 5.0, 5.0, 10.0}}); // duplicate level
    EXPECT_FALSE(isValid(schedule));
}

TEST(LevelSchedule, DescendingLevelsIsInvalid) {
    LevelSchedule schedule;
    schedule.entries.push_back({5.0, {0.0, 10.0, 5.0}}); // out of order
    EXPECT_FALSE(isValid(schedule));
}

TEST(LevelSchedule, RealAppBathymetrySchedulesResolveAsExpected) {
    // Exact values from generate-map-style.js's BATHYMETRY_CONTOUR_SOURCE_NATIVE.
    LevelSchedule lineLevels;
    lineLevels.entries.push_back(
        {5.0, {0, 2, 5, 10, 20, 50, 100, 250, 500, 1000, 2000, 3000, 4000, 5000}});
    ASSERT_TRUE(isValid(lineLevels));
    EXPECT_EQ(resolveLevels(lineLevels, 12.0).size(), 14u);

    LevelSchedule polygonLevels;
    polygonLevels.entries.push_back({5.0, {-100000, 0, 2, 5, 10, 20, 50, 100}});
    ASSERT_TRUE(isValid(polygonLevels));
    EXPECT_EQ(resolveLevels(polygonLevels, 12.0).size(), 8u);
}
