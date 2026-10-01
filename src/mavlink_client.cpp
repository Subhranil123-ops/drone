#include "mavlink_client.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
constexpr uint8_t GCS_SYSTEM_ID = 255;
constexpr uint8_t GCS_COMPONENT_ID = MAV_COMP_ID_ONBOARD_COMPUTER;
constexpr uint32_t GUIDED_CUSTOM_MODE = 4;

constexpr uint16_t TYPE_MASK_VELOCITY =
    (1u << 0) | (1u << 1) | (1u << 2) |
    (1u << 6) | (1u << 7) | (1u << 8) |
    (1u << 10); // ignore yaw, use yaw-rate bit 11 if supplied

uint64_t steadyMilliseconds()
{
    static const auto epoch = std::chrono::steady_clock::now();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - epoch).count());
}
}

MavlinkClient::MavlinkClient()
    : startTime_(std::chrono::steady_clock::now())
{
}

MavlinkClient::~MavlinkClient()
{
    disconnect();
}

bool MavlinkClient::connect(const std::string& host, uint16_t port)
{
    disconnect();

    socketFd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socketFd_ < 0)
    {
        std::cerr << "[MAVLINK] socket failed: "
                  << std::strerror(errno) << '\n';
        return false;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);

    if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1)
    {
        std::cerr << "[MAVLINK] invalid IPv4 address: " << host << '\n';
        disconnect();
        return false;
    }

    if (::connect(
            socketFd_,
            reinterpret_cast<sockaddr*>(&address),
            sizeof(address)) < 0)
    {
        std::cerr << "[MAVLINK] TCP connection failed: "
                  << std::strerror(errno) << '\n';
        disconnect();
        return false;
    }

    startTime_ = std::chrono::steady_clock::now();
    running_.store(true);

    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = VehicleState{};
        state_.connected = false;
    }

    receiveThread_ = std::thread(&MavlinkClient::receiveLoop, this);

    std::cout << "[MAVLINK] connected to "
              << host << ':' << port << '\n';

    return true;
}

void MavlinkClient::disconnect()
{
    running_.store(false);

    if (socketFd_ >= 0)
    {
        ::shutdown(socketFd_, SHUT_RDWR);
        ::close(socketFd_);
        socketFd_ = -1;
    }

    if (receiveThread_.joinable())
        receiveThread_.join();

    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
}

bool MavlinkClient::isConnected() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.connected;
}

VehicleState MavlinkClient::getState() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

bool MavlinkClient::waitForHeartbeat(int timeoutMs)
{
    const auto start = std::chrono::steady_clock::now();

    while (true)
    {
        if (heartbeatFresh(1500))
            return true;

        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();

        if (elapsed >= timeoutMs)
            return false;

        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

bool MavlinkClient::waitForPosition(int timeoutMs)
{
    const auto start = std::chrono::steady_clock::now();

    while (true)
    {
        if (positionFresh(1500))
            return true;

        const auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start).count();

        if (elapsed >= timeoutMs)
            return false;

        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

bool MavlinkClient::heartbeatFresh(uint64_t maxAgeMs) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.connected || state_.lastHeartbeatMs == 0)
        return false;

    return steadyMilliseconds() - state_.lastHeartbeatMs <= maxAgeMs;
}

bool MavlinkClient::positionFresh(uint64_t maxAgeMs) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.positionValid || state_.lastPositionMs == 0)
        return false;

    return steadyMilliseconds() - state_.lastPositionMs <= maxAgeMs;
}

uint32_t MavlinkClient::timeBootMs() const
{
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - startTime_).count());
}

bool MavlinkClient::sendMessage(const mavlink_message_t& message)
{
    std::lock_guard<std::mutex> sendLock(sendMutex_);

    if (socketFd_ < 0 || !running_.load())
        return false;

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length =
        mavlink_msg_to_send_buffer(buffer, &message);

    std::size_t sentTotal = 0;
    while (sentTotal < length)
    {
        const ssize_t sent = ::send(
            socketFd_,
            buffer + sentTotal,
            length - sentTotal,
            MSG_NOSIGNAL);

        if (sent <= 0)
            return false;

        sentTotal += static_cast<std::size_t>(sent);
    }

    return true;
}

bool MavlinkClient::sendCommandLong(
    uint16_t command,
    float p1, float p2, float p3, float p4,
    float p5, float p6, float p7)
{
    mavlink_message_t message{};

    mavlink_msg_command_long_pack(
        GCS_SYSTEM_ID,
        GCS_COMPONENT_ID,
        &message,
        targetSystem_,
        targetComponent_,
        command,
        0,
        p1, p2, p3, p4, p5, p6, p7);

    return sendMessage(message);
}

bool MavlinkClient::requestMessageInterval(uint32_t msgId, double hz)
{
    if (hz <= 0.0)
        return false;

    // MAV_CMD_SET_MESSAGE_INTERVAL = 511.
    const double intervalUs = 1'000'000.0 / hz;

    return sendCommandLong(
        MAV_CMD_SET_MESSAGE_INTERVAL,
        static_cast<float>(msgId),
        static_cast<float>(intervalUs));
}

bool MavlinkClient::setGuidedMode()
{
    mavlink_message_t message{};

    mavlink_msg_set_mode_pack(
        GCS_SYSTEM_ID,
        GCS_COMPONENT_ID,
        &message,
        targetSystem_,
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        GUIDED_CUSTOM_MODE);

    std::cout << "[MAVLINK] requesting GUIDED\n";
    return sendMessage(message);
}

bool MavlinkClient::arm()
{
    std::cout << "[MAVLINK] ARM\n";
    return sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

bool MavlinkClient::disarm()
{
    return sendCommandLong(MAV_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

bool MavlinkClient::takeoff(double altitudeMeters)
{
    if (altitudeMeters <= 0.0)
        return false;

    std::cout << "[MAVLINK] TAKEOFF " << altitudeMeters << " m\n";

    return sendCommandLong(
        MAV_CMD_NAV_TAKEOFF,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f,
        static_cast<float>(altitudeMeters));
}

bool MavlinkClient::land()
{
    std::cout << "[MAVLINK] LAND\n";
    return sendCommandLong(MAV_CMD_NAV_LAND);
}

bool MavlinkClient::sendBodyVelocity(
    float forward,
    float right,
    float down,
    float yawRate)
{
    mavlink_message_t message{};

    mavlink_msg_set_position_target_local_ned_pack(
        GCS_SYSTEM_ID,
        GCS_COMPONENT_ID,
        &message,
        timeBootMs(),
        targetSystem_,
        targetComponent_,
        MAV_FRAME_BODY_NED,
        TYPE_MASK_VELOCITY,
        0.0f, 0.0f, 0.0f,
        forward, right, down,
        0.0f, 0.0f, 0.0f,
        0.0f, yawRate);

    return sendMessage(message);
}

bool MavlinkClient::sendVelocityNED(
    float north,
    float east,
    float down,
    float yawRate)
{
    mavlink_message_t message{};

    mavlink_msg_set_position_target_local_ned_pack(
        GCS_SYSTEM_ID,
        GCS_COMPONENT_ID,
        &message,
        timeBootMs(),
        targetSystem_,
        targetComponent_,
        MAV_FRAME_LOCAL_NED,
        TYPE_MASK_VELOCITY,
        0.0f, 0.0f, 0.0f,
        north, east, down,
        0.0f, 0.0f, 0.0f,
        0.0f, yawRate);

    return sendMessage(message);
}

bool MavlinkClient::sendHold()
{
    return sendBodyVelocity(0.0f, 0.0f, 0.0f, 0.0f);
}

void MavlinkClient::receiveLoop()
{
    uint8_t buffer[4096];

    while (running_.load())
    {
        const ssize_t received =
            ::recv(socketFd_, buffer, sizeof(buffer), 0);

        if (received <= 0)
            break;

        for (ssize_t i = 0; i < received; ++i)
        {
            mavlink_message_t message{};

            if (mavlink_parse_char(
                    MAVLINK_COMM_0,
                    buffer[i],
                    &message,
                    &parserStatus_))
            {
                handleMessage(message);
            }
        }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
}

void MavlinkClient::handleMessage(const mavlink_message_t& message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    switch (message.msgid)
    {
        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            mavlink_heartbeat_t hb{};
            mavlink_msg_heartbeat_decode(&message, &hb);

            if (hb.autopilot == MAV_AUTOPILOT_ARDUPILOTMEGA)
            {
                state_.connected = true;
                state_.armed =
                    (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                state_.customMode = hb.custom_mode;
                state_.systemId = message.sysid;
                state_.componentId = message.compid;
                targetSystem_ = message.sysid;
                targetComponent_ = message.compid;
                state_.lastHeartbeatMs = steadyMilliseconds();
            }
            break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            mavlink_local_position_ned_t p{};
            mavlink_msg_local_position_ned_decode(&message, &p);

            state_.localX = p.x;
            state_.localY = p.y;
            state_.localZ = p.z;
            state_.vx = p.vx;
            state_.vy = p.vy;
            state_.vz = p.vz;
            state_.positionValid = true;
            state_.lastPositionMs = steadyMilliseconds();
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
            mavlink_attitude_t a{};
            mavlink_msg_attitude_decode(&message, &a);
            state_.yaw = a.yaw;
            break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            mavlink_global_position_int_t p{};
            mavlink_msg_global_position_int_decode(&message, &p);

            state_.latitude = static_cast<double>(p.lat) / 1e7;
            state_.longitude = static_cast<double>(p.lon) / 1e7;
            state_.altitude = static_cast<double>(p.relative_alt) / 1000.0;

            if (p.hdg != UINT16_MAX)
            {
                state_.yaw =
                    static_cast<double>(p.hdg) / 100.0 *
                    3.14159265358979323846 / 180.0;
            }
            break;
        }

        default:
            break;
    }
}
