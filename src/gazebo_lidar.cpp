#include "gazebo_lidar.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace
{
constexpr double PI = 3.14159265358979323846;

double normalizeAngle(double a)
{
    while (a > PI) a -= 2.0 * PI;
    while (a < -PI) a += 2.0 * PI;
    return a;
}
}

GazeboLidar::~GazeboLidar()
{
    shutdown();
}

bool GazeboLidar::subscribe(const std::string& topic)
{
    if (subscribed_.load())
        return true;

    std::cout << "[LIDAR] subscribing to " << topic << '\n';

    const bool ok = node_.Subscribe(topic, &GazeboLidar::callback, this);
    subscribed_.store(ok);

    if (!ok)
        std::cerr << "[LIDAR] subscription failed: " << topic << '\n';
    else
        std::cout << "[LIDAR] subscription OK\n";

    return ok;
}

bool GazeboLidar::hasData() const
{
    return received_.load();
}

LidarFrame GazeboLidar::getFrame() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_;
}

std::size_t GazeboLidar::sampleCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return frame_.beams.size();
}

void GazeboLidar::callback(const gz::msgs::LaserScan& message)
{
    const int hCount = message.count() > 0
        ? static_cast<int>(message.count())
        : message.ranges_size();

    const int vCount = message.vertical_count() > 0
        ? static_cast<int>(message.vertical_count())
        : 1;

    if (hCount <= 0 || vCount <= 0 || message.ranges_size() <= 0)
        return;

    // Gazebo stores multi-layer scans as vertical layers of horizontal scans.
    const int available = message.ranges_size();
    const int expected = hCount * vCount;
    const int actual = std::min(available, expected);

    LidarFrame next;
    next.horizontalCount = static_cast<std::size_t>(hCount);
    next.verticalCount = static_cast<std::size_t>(vCount);
    next.azimuthMinRad = message.angle_min();
    next.azimuthMaxRad = message.angle_max();
    next.azimuthStepRad = message.angle_step();
    next.elevationMinRad = message.vertical_angle_min();
    next.elevationMaxRad = message.vertical_angle_max();
    next.elevationStepRad = message.vertical_angle_step();
    next.rangeMinM = message.range_min();
    next.rangeMaxM = message.range_max();

    next.beams.reserve(static_cast<std::size_t>(actual));

    for (int v = 0; v < vCount; ++v)
    {
        const double elevation =
            vCount == 1
                ? 0.0
                : next.elevationMinRad +
                  static_cast<double>(v) * next.elevationStepRad;

        for (int h = 0; h < hCount; ++h)
        {
            const int index = v * hCount + h;
            if (index >= actual)
                break;

            const double raw = message.ranges(index);
            double distance = raw;

            // Gazebo reports "no return" as +inf (or >= range_max): that is
            // OBSERVED FREE SPACE out to range_max. NaN / -inf / <=0 are
            // invalid (below min range) and stay non-finite so the
            // processor ignores them.
            const double maxRange =
                next.rangeMaxM > 0.0 ? next.rangeMaxM : 20.0;

            if (std::isnan(distance) || distance <= 0.0)
            {
                distance = std::numeric_limits<double>::quiet_NaN();
            }
            else if (!std::isfinite(distance) || distance >= maxRange)
            {
                distance = maxRange;
            }

            const double azimuth = normalizeAngle(
                next.azimuthMinRad +
                static_cast<double>(h) * next.azimuthStepRad);

            next.beams.push_back({azimuth, elevation, distance});
        }
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        next.sequence = frame_.sequence + 1;
        frame_ = std::move(next);
    }

    received_.store(true);
}

void GazeboLidar::shutdown()
{
    subscribed_.store(false);
}
