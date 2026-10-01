#pragma once

#include <atomic>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

struct LidarBeam
{
    double azimuthRad = 0.0;   // RAW Gazebo convention: 0 = sensor forward, + = left (CCW)
    double elevationRad = 0.0; // + = up, - = down
    double distanceM = 0.0;
};

struct LidarFrame
{
    std::vector<LidarBeam> beams;
    std::size_t horizontalCount = 0;
    std::size_t verticalCount = 0;
    double azimuthMinRad = 0.0;
    double azimuthMaxRad = 0.0;
    double azimuthStepRad = 0.0;
    double elevationMinRad = 0.0;
    double elevationMaxRad = 0.0;
    double elevationStepRad = 0.0;
    double rangeMinM = 0.0;
    double rangeMaxM = 0.0;
    uint64_t sequence = 0;
};

class GazeboLidar
{
public:
    GazeboLidar() = default;
    ~GazeboLidar();

    bool subscribe(const std::string& topic);
    bool hasData() const;
    LidarFrame getFrame() const;
    std::size_t sampleCount() const;
    void shutdown();

private:
    void callback(const gz::msgs::LaserScan& message);

    gz::transport::Node node_;
    mutable std::mutex mutex_;
    LidarFrame frame_;
    std::atomic<bool> received_{false};
    std::atomic<bool> subscribed_{false};
};
