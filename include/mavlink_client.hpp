#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

extern "C"
{
#include <ardupilotmega/mavlink.h>
}

struct VehicleState
{
    bool connected = false;
    bool armed = false;
    bool positionValid = false;

    uint8_t systemId = 1;
    uint8_t componentId = 1;

    uint32_t customMode = 0;

    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    // ArduPilot LOCAL_POSITION_NED.
    double localX = 0.0; // north
    double localY = 0.0; // east
    double localZ = 0.0; // down

    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;
    double yaw = 0.0;

    uint64_t lastHeartbeatMs = 0;
    uint64_t lastPositionMs = 0;
};

class MavlinkClient
{
public:
    MavlinkClient();
    ~MavlinkClient();

    bool connect(const std::string& host, uint16_t port);
    void disconnect();

    bool isConnected() const;
    VehicleState getState() const;

    bool waitForHeartbeat(int timeoutMs);
    bool waitForPosition(int timeoutMs);

    bool heartbeatFresh(uint64_t maxAgeMs) const;
    bool positionFresh(uint64_t maxAgeMs) const;

    bool requestMessageInterval(uint32_t msgId, double hz);

    bool setGuidedMode();
    bool arm();
    bool disarm();
    bool takeoff(double altitudeMeters);
    bool land();

    bool sendBodyVelocity(
        float forward,
        float right,
        float down,
        float yawRate = 0.0f);

    bool sendVelocityNED(
        float north,
        float east,
        float down,
        float yawRate = 0.0f);

    bool sendHold();

private:
    bool sendMessage(const mavlink_message_t& message);

    bool sendCommandLong(
        uint16_t command,
        float p1 = 0.0f,
        float p2 = 0.0f,
        float p3 = 0.0f,
        float p4 = 0.0f,
        float p5 = 0.0f,
        float p6 = 0.0f,
        float p7 = 0.0f);

    uint32_t timeBootMs() const;
    void receiveLoop();
    void handleMessage(const mavlink_message_t& message);

    int socketFd_ = -1;
    std::thread receiveThread_;
    std::atomic<bool> running_{false};

    mutable std::mutex mutex_;
    mutable std::mutex sendMutex_;

    VehicleState state_;
    mavlink_status_t parserStatus_{};

    uint8_t targetSystem_ = 1;
    uint8_t targetComponent_ = 1;

    std::chrono::steady_clock::time_point startTime_;
};
