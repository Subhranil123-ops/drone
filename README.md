# Autonomous Drone Navigation

This version converts the original rule-based prototype into a closed-loop SITL controller.

## Data path

Gazebo 360° x 16-channel LiDAR
-> C++ LiDAR parser
-> 72-sector local obstacle map
-> goal-directed local planner
-> BODY_NED velocity
-> MAVLink TCP
-> ArduPilot GUIDED
-> simulated Iris

## Important behavior

- No obstacle coordinates are used by the controller.
- Destination is supplied as a relative LOCAL_NED offset.
- The planner continuously uses the latest LiDAR scan and current vehicle pose.
- The controller slows down in narrow corridors.
- Stale MAVLink or LiDAR data causes a HOLD; prolonged LiDAR loss can LAND.
- Reaching the configured position tolerance triggers HOLD and, by default, LAND.
- LiDAR is configured for 360 horizontal samples and 16 vertical channels.

## Build / run

Start ArduPilot SITL and Gazebo as you normally do, then:

```bash
cd ~/drone-navigation
./run_autonomous.sh
```

For the current test world:

```bash
GOAL_NORTH_M=20 GOAL_EAST_M=24 ./run_autonomous.sh
```

If your ArduPilot/Gazebo coordinate transform makes the test destination correspond to another N/E direction, change only these two values. The obstacle layout never enters the controller.

## Safety

This profile is intended for ArduPilot SITL/Gazebo. Do not enable automatic arming/takeoff on a real vehicle until a qualified operator has verified geofence, failsafes, RC kill/disarm, flight mode, estimator health and the exact coordinate/frame mapping.

The controller is a reactive local planner, not a mathematical guarantee of collision-free flight in every possible environment. Its correctness depends on the LiDAR coverage, update rate, vehicle dynamics and the environment being within the configured sensing/planning assumptions.
