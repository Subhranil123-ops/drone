#include "navigator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace
{
constexpr double PI = 3.14159265358979323846;
constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double kInflateRad = 20.0 * PI / 180.0;

double clamp(double x, double lo, double hi)
{
    return std::max(lo, std::min(hi, x));
}
}

Navigator::Navigator(
    double safetyDistanceM,
    double emergencyDistanceM,
    double maxHorizontalSpeedMps)
    :
    safetyDistanceM_(std::max(0.5, safetyDistanceM)),
    emergencyDistanceM_(std::max(0.25, emergencyDistanceM)),
    maxSpeedMps_(std::max(0.2, maxHorizontalSpeedMps))
{
}

double Navigator::normalizeAngle(double a)
{
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}

NavigationCommand Navigator::update(
    const VehicleState& vehicle,
    const Goal& goal,
    const LidarAnalysis& lidar)
{
    NavigationCommand cmd;

    // ArduPilot LOCAL_POSITION_NED:
    // X=north, Y=east, Z=down.
    const double dn = goal.northM - vehicle.localX;
    const double de = goal.eastM - vehicle.localY;
    const double horizontalDistance = std::hypot(dn, de);

    cmd.distanceToGoalM = horizontalDistance;

    const double altitudeErrorDown =
        goal.downM - vehicle.localZ;

    if (horizontalDistance <= goal.positionToleranceM)
    {
        // Hold horizontally. Correct altitude before declaring success.
        if (std::abs(altitudeErrorDown) > 0.4)
            cmd.downMps = static_cast<float>(
                clamp(altitudeErrorDown * 0.5, -0.6, 0.6));

        if (std::abs(altitudeErrorDown) <= 0.4)
            cmd.reached = true;

        return cmd;
    }

    // Convert desired LOCAL_NED goal direction into BODY_NED.
    const double yaw = vehicle.yaw;
    const double desiredBodyForward =
        std::cos(yaw) * dn + std::sin(yaw) * de;
    const double desiredBodyRight =
        -std::sin(yaw) * dn + std::cos(yaw) * de;

    const double goalAngle =
        std::atan2(desiredBodyRight, desiredBodyForward);

    // Score every horizontal sector.
    // High score = points toward goal + plenty of clearance.
    double bestScore = -INF;
    double bestAngle = goalAngle;
    double bestDistance = 0.0;

    // Inflate obstacles angularly (drone width + margin): a heading is only
    // as clear as the closest return within +/- kInflateRad of it.
    const std::size_t n = lidar.sectors.size();
    std::vector<double> eff(n, INF);
    if (n > 0)
    {
        const double width = 2.0 * PI / static_cast<double>(n);
        const int k = static_cast<int>(std::ceil(kInflateRad / width));
        for (std::size_t i = 0; i < n; ++i)
        {
            double m = INF;
            for (int j = -k; j <= k; ++j)
            {
                const std::size_t idx =
                    (i + n + static_cast<std::size_t>(j + static_cast<int>(n))) % n;
                if (lidar.sectors[idx].observed)
                    m = std::min(m, lidar.sectors[idx].distanceM);
            }
            eff[i] = m;
        }
    }

    // Pass 1: only headings with full safety clearance. Pass 2 (surrounded):
    // accept anything beyond the emergency distance, speed-limited below.
    for (const double minClear : {safetyDistanceM_, emergencyDistanceM_})
    {
        for (std::size_t si = 0; si < n; ++si)
        {
            const auto& sector = lidar.sectors[si];
            if (!sector.observed)
                continue;

            const double d = eff[si];
            if (!std::isfinite(d) || d < minClear)
                continue;

            const double angularError =
                std::abs(normalizeAngle(sector.angleRad - goalAngle));

            // Alignment is dominant. Clearance is the second objective.
            const double alignment =
                std::cos(angularError);

            const double clearance =
                clamp(d / std::max(1.0, safetyDistanceM_ * 3.0), 0.0, 1.0);

            // Mild hysteresis prevents left/right oscillation.
            const double hysteresis =
                0.15 * std::cos(
                    normalizeAngle(sector.angleRad - previousAngleRad_));

            const double score =
                3.0 * alignment +
                1.5 * clearance +
                hysteresis;

            if (score > bestScore)
            {
                bestScore = score;
                bestAngle = sector.angleRad;
                bestDistance = d;
            }
        }
        if (bestScore > -INF)
            break;
    }

    // If the goal-facing sector is blocked, the selected free sector
    // naturally bends around the obstacle. No obstacle coordinates are used.
    if (bestScore == -INF)
    {
        ++blockedCycles_;

        // Completely blocked: stop instead of guessing.
        cmd.forwardMps = 0.0f;
        cmd.rightMps = 0.0f;

        // If there is clear space above and it is not dangerously close,
        // climb slowly. Otherwise wait for a new scan.
        if (std::isfinite(lidar.upperClearanceM) &&
            lidar.upperClearanceM < emergencyDistanceM_)
        {
            cmd.downMps = 0.0f;
        }
        else if (std::isfinite(lidar.upperClearanceM) &&
                 lidar.upperClearanceM > safetyDistanceM_ * 1.5)
        {
            cmd.downMps = -0.35f;
        }

        return cmd;
    }

    blockedCycles_ = 0;
    previousAngleRad_ = bestAngle;

    // Slow down as the selected corridor narrows.
    const double clearanceFactor =
        clamp((bestDistance - emergencyDistanceM_) /
              std::max(0.5, safetyDistanceM_),
              0.25, 1.0);

    double speed =
        maxSpeedMps_ * clearanceFactor;

    // Slow down near the goal.
    speed = std::min(
        speed,
        std::max(0.25, horizontalDistance * 0.35));

    cmd.forwardMps = static_cast<float>(
        speed * std::cos(bestAngle));

    cmd.rightMps = static_cast<float>(
        speed * std::sin(bestAngle));

    // NED down is positive. altitudeError = goalDown - currentDown;
    // negative => must climb => negative down-velocity (P gain is +).
    const double altitudeError =
        goal.downM - vehicle.localZ;

    if (std::abs(altitudeError) > 0.25)
    {
        cmd.downMps = static_cast<float>(
            clamp(0.45 * altitudeError, -0.6, 0.6));
    }

    // Velocity limiter (all 360 deg): remove the velocity component pointing
    // toward ANY sector closer than the safety distance, fully at/below the
    // emergency distance. Protects against tangential passes beside obstacles.
    {
        double vf = cmd.forwardMps, vr = cmd.rightMps;
        for (const auto& sec : lidar.sectors)
        {
            if (!sec.observed || !std::isfinite(sec.distanceM) ||
                sec.distanceM >= safetyDistanceM_)
                continue;

            const double ux = std::cos(sec.angleRad);
            const double uy = std::sin(sec.angleRad);
            const double toward = vf * ux + vr * uy;
            if (toward <= 0.0)
                continue;

            const double w = clamp(
                (safetyDistanceM_ - sec.distanceM) /
                    std::max(0.1, safetyDistanceM_ - emergencyDistanceM_),
                0.0, 1.0);
            vf -= ux * toward * w;
            vr -= uy * toward * w;
        }
        cmd.forwardMps = static_cast<float>(vf);
        cmd.rightMps = static_cast<float>(vr);
    }

    // Conservative emergency braking if the forward cone is closing.
    if (lidar.frontObserved &&
        lidar.levelFrontM < emergencyDistanceM_ * 1.25)
    {
        const double scale =
            clamp(lidar.levelFrontM /
                  (emergencyDistanceM_ * 1.25),
                  0.0, 1.0);

        cmd.forwardMps = static_cast<float>(
            cmd.forwardMps * scale);
        cmd.rightMps = static_cast<float>(
            cmd.rightMps * scale);
    }

    return cmd;
}
