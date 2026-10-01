#pragma once

#include "gazebo_lidar.hpp"

#include <cstddef>
#include <vector>

struct LidarSector
{
    double angleRad = 0.0; // BODY_NED azimuth: 0 forward, + RIGHT
    double distanceM = 0.0;
    bool observed = false;
};

struct LidarAnalysis
{
    std::vector<LidarSector> sectors;

    double levelFrontM = 0.0;
    double levelLeftM = 0.0;
    double levelRightM = 0.0;
    double levelRearM = 0.0;

    double upperClearanceM = 0.0;
    double lowerClearanceM = 0.0;

    bool frontObserved = false;
    bool leftObserved = false;
    bool rightObserved = false;
    bool rearObserved = false;
};

class LidarProcessor
{
public:
    explicit LidarProcessor(
        std::size_t sectorCount = 72,
        double levelElevationDeg = 12.0);

    LidarAnalysis analyze(const LidarFrame& frame) const;

private:
    static double normalizeAngle(double a);

    std::size_t sectorCount_;
    double levelElevationRad_;
};
