#include "lidar_processor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double INF = std::numeric_limits<double>::infinity();
}

LidarProcessor::LidarProcessor(
    std::size_t sectorCount,
    double levelElevationDeg)
    :
    sectorCount_(std::max<std::size_t>(36, sectorCount)),
    levelElevationRad_(levelElevationDeg * PI / 180.0)
{
}

double LidarProcessor::normalizeAngle(double a)
{
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

LidarAnalysis LidarProcessor::analyze(const LidarFrame& frame) const
{
    LidarAnalysis out;
    out.sectors.resize(sectorCount_);

    const double sectorWidth = 2.0 * PI / static_cast<double>(sectorCount_);

    for (std::size_t i = 0; i < sectorCount_; ++i)
    {
        out.sectors[i].angleRad =
            -PI + (static_cast<double>(i) + 0.5) * sectorWidth;
        out.sectors[i].distanceM = INF;
    }

    double upper = INF;
    double lower = INF;

    // Use a narrow band around level for horizontal flight planning.
    // This avoids interpreting the floor, seen by downward beams, as a wall.
    for (const auto& beam : frame.beams)
    {
        if (std::isnan(beam.distanceM) || beam.distanceM <= 0.0)
            continue; // invalid (below min range / bad sample)

        // "No return" (+inf or >= range_max) is OBSERVED FREE SPACE.
        const double maxRange =
            frame.rangeMaxM > 0.0 ? frame.rangeMaxM : 20.0;
        const double dist =
            (!std::isfinite(beam.distanceM) || beam.distanceM >= maxRange)
                ? maxRange
                : beam.distanceM;

        if (beam.elevationRad > levelElevationRad_)
            upper = std::min(upper, dist);

        if (beam.elevationRad < -levelElevationRad_)
            lower = std::min(lower, dist);

        if (std::abs(beam.elevationRad) > levelElevationRad_)
            continue;

        // Gazebo azimuth is CCW-positive (left). The planner works in
        // BODY_NED (x forward, y right), so negate: sector angle + = RIGHT.
        double a = normalizeAngle(-beam.azimuthRad);
        std::size_t idx = static_cast<std::size_t>(
            std::floor((a + PI) / sectorWidth));

        if (idx >= sectorCount_)
            idx = sectorCount_ - 1;

        out.sectors[idx].observed = true;
        out.sectors[idx].distanceM =
            std::min(out.sectors[idx].distanceM, dist);
    }

    out.upperClearanceM = upper;
    out.lowerClearanceM = lower;

    auto minAround = [&](double center, double halfWidth, bool& observed)
    {
        double result = INF;
        observed = false;

        for (const auto& s : out.sectors)
        {
            if (std::abs(normalizeAngle(s.angleRad - center)) <= halfWidth &&
                s.observed)
            {
                observed = true;
                result = std::min(result, s.distanceM);
            }
        }
        return result;
    };

    out.levelFrontM = minAround(0.0, 30.0 * PI / 180.0, out.frontObserved);
    out.levelLeftM  = minAround(-PI / 2.0, 60.0 * PI / 180.0, out.leftObserved);
    out.levelRightM = minAround(+PI / 2.0, 60.0 * PI / 180.0, out.rightObserved);
    out.levelRearM  = minAround(PI, 60.0 * PI / 180.0, out.rearObserved);

    return out;
}
