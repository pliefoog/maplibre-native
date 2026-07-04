#include <mbgl/algorithm/contour/levels.hpp>

#include <limits>

namespace mbgl {
namespace algorithm {
namespace contour {

namespace {
const std::vector<double> kEmpty{};
} // namespace

const std::vector<double>& resolveLevels(const LevelSchedule& schedule, double zoom) {
    if (schedule.entries.empty()) return kEmpty;

    // entries is ascending by zoom breakpoint (enforced by isValid, and by
    // the parser that constructs it -- see convertContourSource). Walk
    // forward, keeping the last entry whose breakpoint is <= zoom.
    const std::vector<double>* best = &schedule.entries.front().second; // fallback: smallest breakpoint
    for (const auto& [breakpoint, levels] : schedule.entries) {
        if (breakpoint <= zoom) {
            best = &levels;
        } else {
            // Entries are ascending by breakpoint, so once one breakpoint
            // exceeds `zoom`, every later one does too -- stop early.
            break;
        }
    }
    return *best;
}

bool isValid(const LevelSchedule& schedule) {
    if (schedule.entries.empty()) return false;
    double prevBreakpoint = -std::numeric_limits<double>::infinity();
    for (const auto& [breakpoint, levels] : schedule.entries) {
        if (breakpoint <= prevBreakpoint) return false; // breakpoints must strictly increase
        prevBreakpoint = breakpoint;

        if (levels.size() < 2) return false;
        for (std::size_t i = 1; i < levels.size(); ++i) {
            if (levels[i] <= levels[i - 1]) return false; // levels must strictly increase
        }
    }
    return true;
}

} // namespace contour
} // namespace algorithm
} // namespace mbgl
