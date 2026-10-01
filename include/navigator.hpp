#pragma once

#include "lidar_processor.hpp"
#include "mavlink_client.hpp"

struct Goal
{
    double northM = 0.0;
    double eastM = 0.0;
    double downM = -3.0; // absolute LOCAL_NED Z target
    double positionToleranceM = 1.0;
};

struct NavigationCommand
{
    float forwardMps = 0.0f;
    float rightMps = 0.0f;
    float downMps = 0.0f;
    bool reached = false;
    double distanceToGoalM = 0.0;
    double chosenAngleRad = 0.0;
};

class Navigator
{
public:
    Navigator(
        double safetyDistanceM,
        double emergencyDistanceM,
        double maxHorizontalSpeedMps);

    NavigationCommand update(
        const VehicleState& vehicle,
        const Goal& goal,
        const LidarAnalysis& lidar);

private:
    static double normalizeAngle(double a);

    double safetyDistanceM_;
    double emergencyDistanceM_;
    double maxSpeedMps_;

    double previousAngleRad_ = 0.0;
    int blockedCycles_ = 0;
};
