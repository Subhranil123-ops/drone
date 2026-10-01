#include <csignal>

#include "gazebo_lidar.hpp"
#include "lidar_processor.hpp"
#include "mavlink_client.hpp"
#include "navigator.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace
{
std::atomic<bool> running{true};

void signalHandler(int)
{
    running.store(false);
}

std::string envString(const char* name, const std::string& fallback)
{
    const char* v = std::getenv(name);
    return (v && *v) ? std::string(v) : fallback;
}

double envDouble(const char* name, double fallback)
{
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;

    try { return std::stod(v); }
    catch (...) { return fallback; }
}

int envInt(const char* name, int fallback)
{
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;

    try { return std::stoi(v); }
    catch (...) { return fallback; }
}

bool envBool(const char* name, bool fallback)
{
    const char* v = std::getenv(name);
    if (!v || !*v) return fallback;

    const std::string s(v);
    return s == "1" || s == "true" || s == "TRUE" ||
           s == "yes" || s == "YES";
}

bool waitUntil(
    const std::function<bool()>& predicate,
    int timeoutMs,
    int sleepMs = 50)
{
    const auto start = std::chrono::steady_clock::now();

    while (running.load())
    {
        if (predicate())
            return true;

        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();

        if (elapsed >= timeoutMs)
            return false;

        std::this_thread::sleep_for(
            std::chrono::milliseconds(sleepMs));
    }

    return false;
}
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const std::string lidarTopic =
        envString("GZ_LIDAR_TOPIC", "/scan");

    const std::string mavHost =
        envString("MAVLINK_HOST", "127.0.0.1");

    const int mavPort =
        envInt("MAVLINK_PORT", 5760);

    const double safetyDistance =
        envDouble("SAFETY_DISTANCE_M", 2.2);

    const double emergencyDistance =
        envDouble("EMERGENCY_DISTANCE_M", 1.0);

    const double maxSpeed =
        envDouble("MAX_SPEED_MPS", 1.2);

    const double goalNorthOffset =
        envDouble("GOAL_NORTH_M", 20.0);

    const double goalEastOffset =
        envDouble("GOAL_EAST_M", 24.0);

    const double goalAltitude =
        envDouble("GOAL_ALTITUDE_M", 3.0);

    const double goalTolerance =
        envDouble("GOAL_TOLERANCE_M", 1.0);

    const double controlHz =
        std::max(5.0, envDouble("CONTROL_HZ", 10.0));

    const bool autoArm =
        envBool("AUTO_ARM", false);

    const bool autoTakeoff =
        envBool("AUTO_TAKEOFF", false);

    const bool autoLand =
        envBool("AUTO_LAND", true);

    std::cout
        << "\n============================================================\n"
        << " AUTONOMOUS DRONE NAVIGATOR\n"
        << " ============================================================\n"
        << "LiDAR topic       : " << lidarTopic << '\n'
        << "MAVLink           : " << mavHost << ':' << mavPort << '\n'
        << "Safety distance   : " << safetyDistance << " m\n"
        << "Emergency distance: " << emergencyDistance << " m\n"
        << "Max speed         : " << maxSpeed << " m/s\n"
        << "Goal N offset     : " << goalNorthOffset << " m\n"
        << "Goal E offset     : " << goalEastOffset << " m\n"
        << "Goal altitude     : " << goalAltitude << " m\n"
        << "Auto arm          : " << (autoArm ? "YES" : "NO") << '\n'
        << "Auto takeoff      : " << (autoTakeoff ? "YES" : "NO") << '\n'
        << "Auto land         : " << (autoLand ? "YES" : "NO") << '\n'
        << "============================================================\n";

    MavlinkClient mavlink;
    GazeboLidar lidar;
    LidarProcessor processor(72, 12.0);
    Navigator navigator(
        safetyDistance,
        emergencyDistance,
        maxSpeed);

    if (!mavlink.connect(
            mavHost,
            static_cast<uint16_t>(mavPort)))
    {
        std::cerr << "[FATAL] MAVLink connection failed\n";
        return 1;
    }

    if (!mavlink.waitForHeartbeat(10000))
    {
        std::cerr << "[FATAL] No ArduPilot heartbeat\n";
        mavlink.disconnect();
        return 1;
    }

    // Ask ArduPilot for the telemetry streams required by the controller.
    // Failure here is non-fatal because SITL may already stream them.
    mavlink.requestMessageInterval(MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10.0);
    mavlink.requestMessageInterval(MAVLINK_MSG_ID_ATTITUDE, 20.0);
    mavlink.requestMessageInterval(MAVLINK_MSG_ID_HEARTBEAT, 2.0);

    if (!mavlink.waitForPosition(5000))
    {
        std::cerr << "[FATAL] No LOCAL_POSITION_NED received\n";
        mavlink.disconnect();
        return 1;
    }

    if (!lidar.subscribe(lidarTopic))
    {
        std::cerr << "[FATAL] LiDAR subscription failed\n";
        mavlink.disconnect();
        return 1;
    }

    if (!waitUntil([&] { return lidar.hasData(); }, 10000))
    {
        std::cerr << "[FATAL] No LiDAR frame received\n";
        mavlink.sendHold();
        mavlink.disconnect();
        return 1;
    }

    const VehicleState startState = mavlink.getState();

    Goal goal;
    goal.northM = startState.localX + goalNorthOffset;
    goal.eastM = startState.localY + goalEastOffset;
    goal.downM = startState.localZ - goalAltitude;
    goal.positionToleranceM = goalTolerance;

    std::cout << std::fixed << std::setprecision(2)
              << "[GOAL] start N/E/D = "
              << startState.localX << ", "
              << startState.localY << ", "
              << startState.localZ << '\n'
              << "[GOAL] target N/E/D = "
              << goal.northM << ", "
              << goal.eastM << ", "
              << goal.downM << '\n'
              << "[LIDAR] samples = "
              << lidar.sampleCount() << '\n';

    // The SDF is expected to produce 360 x 16 = 5760 beams.
    // We don't hard-fail if a simulator exposes a different resolution,
    // but we print it so a wrong sensor configuration is obvious.
    if (lidar.sampleCount() < 1000)
    {
        std::cerr
            << "[WARNING] LiDAR frame has only "
            << lidar.sampleCount()
            << " ranges. The autonomous profile expects a multi-layer "
               "360-degree scan (normally 360 x 16).\n";
    }

    if (!mavlink.setGuidedMode())
    {
        std::cerr << "[FATAL] Could not send GUIDED request\n";
        mavlink.disconnect();
        return 1;
    }

    if (!waitUntil(
            [&]
            {
                return mavlink.getState().customMode == 4;
            },
            3000))
    {
        std::cerr
            << "[WARNING] GUIDED was not confirmed in 3 seconds. "
               "Continuing only if telemetry remains healthy.\n";
    }

    if (autoArm)
    {
        if (!mavlink.arm() ||
            !waitUntil(
                [&] { return mavlink.getState().armed; },
                7000))
        {
            std::cerr << "[FATAL] ARM failed or was not confirmed\n";
            mavlink.sendHold();
            mavlink.disconnect();
            return 1;
        }
    }

    if (autoTakeoff)
    {
        if (!mavlink.getState().armed)
        {
            std::cerr
                << "[FATAL] AUTO_TAKEOFF requested but vehicle is not armed\n";
            mavlink.disconnect();
            return 1;
        }

        if (!mavlink.takeoff(goalAltitude))
        {
            std::cerr << "[FATAL] TAKEOFF command failed\n";
            mavlink.disconnect();
            return 1;
        }

        const bool airborne = waitUntil(
            [&]
            {
                const auto s = mavlink.getState();
                const double relativeAltitude =
                    startState.localZ - s.localZ;
                return relativeAltitude >=
                       std::max(1.0, goalAltitude * 0.75);
            },
            15000);

        if (!airborne)
        {
            std::cerr << "[FATAL] Takeoff altitude was not reached\n";
            mavlink.land();
            mavlink.disconnect();
            return 1;
        }
    }

    std::cout
        << "\n[NAV] CLOSED-LOOP NAVIGATION STARTED\n"
        << "[NAV] No obstacle coordinates are hard-coded.\n"
        << "[NAV] Goal is relative to the vehicle's startup LOCAL_NED pose.\n";

    const auto period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / controlHz));

    auto nextTick = std::chrono::steady_clock::now();
    uint64_t lastScanSequence = 0;
    auto lastScanTime = std::chrono::steady_clock::now();

    bool reached = false;

    while (running.load())
    {
        nextTick += period;

        if (!mavlink.heartbeatFresh(1500) ||
            !mavlink.positionFresh(1500))
        {
            std::cerr
                << "[SAFETY] stale MAVLink telemetry -> HOLD\n";
            mavlink.sendHold();
            std::this_thread::sleep_until(nextTick);
            continue;
        }

        const LidarFrame frame = lidar.getFrame();

        if (frame.sequence != lastScanSequence)
        {
            lastScanSequence = frame.sequence;
            lastScanTime = std::chrono::steady_clock::now();
        }

        const auto scanAge =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - lastScanTime).count();

        if (scanAge > 1200)
        {
            std::cerr
                << "[SAFETY] LiDAR stale (" << scanAge
                << " ms) -> HOLD\n";

            mavlink.sendHold();

            if (scanAge > 2500 && autoLand)
                mavlink.land();

            std::this_thread::sleep_until(nextTick);
            continue;
        }

        const VehicleState state = mavlink.getState();
        const LidarAnalysis analysis = processor.analyze(frame);
        const NavigationCommand command =
            navigator.update(state, goal, analysis);

        if (command.reached)
        {
            if (!reached)
            {
                std::cout
                    << "\n[NAV] DESTINATION REACHED within "
                    << goal.positionToleranceM
                    << " m\n";
                reached = true;
            }

            mavlink.sendHold();

            if (autoLand)
            {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(1000));
                mavlink.land();
            }

            break;
        }

        // If the emergency cone is almost blocked, Navigator reduces
        // horizontal speed. This extra guard prevents a stale/unsafe
        // direction from ever being sent.
        float forward = command.forwardMps;
        float right = command.rightMps;

        if (analysis.frontObserved &&
            analysis.levelFrontM <= emergencyDistance)
        {
            forward = 0.0f;

            // Allow lateral escape only when a lateral sector is known safe.
            if (!analysis.leftObserved || analysis.levelLeftM <= emergencyDistance)
                right = std::max(0.0f, right);

            if (!analysis.rightObserved || analysis.levelRightM <= emergencyDistance)
                right = std::min(0.0f, right);
        }

        const bool sent =
            mavlink.sendBodyVelocity(
                forward,
                right,
                command.downMps);

        if (!sent)
            std::cerr << "[MAVLINK] velocity send failed\n";

        static int printCounter = 0;
        if (++printCounter >= 10)
        {
            printCounter = 0;

            std::cout
                << std::fixed << std::setprecision(2)
                << "[NAV] dist=" << command.distanceToGoalM
                << "m F=" << forward
                << " R=" << right
                << " D=" << command.downMps
                << " front=" << analysis.levelFrontM
                << " left=" << analysis.levelLeftM
                << " right=" << analysis.levelRightM
                << " lidar=" << frame.horizontalCount
                << "x" << frame.verticalCount
                << '\n';
        }

        std::this_thread::sleep_until(nextTick);
    }

    mavlink.sendHold();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    if (!running.load())
        std::cout << "[NAV] stopped by user\n";

    mavlink.disconnect();
    lidar.shutdown();

    std::cout << "[DONE] autonomous controller stopped\n";
    return 0;
}
