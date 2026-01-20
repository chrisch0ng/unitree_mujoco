# test_unitree_sdk2.cpp - Line by Line Explanation

## Overview

This test program communicates with the Unitree MuJoCo simulator via DDS (Data Distribution Service). It receives robot state information and sends motor torque commands.

---

## Includes (Lines 1-6)

```cpp
#include <unitree/robot/channel/channel_subscriber.hpp>
```
DDS subscriber for receiving messages from the simulator.

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
```
DDS publisher for sending messages to the simulator.

```cpp
#include <unitree/common/time/time_tool.hpp>
```
Time utilities (included but not actually used in this code).

```cpp
#include <unitree/idl/go2/LowState_.hpp>
```
Message type for low-level robot state (IMU data, motor states).

```cpp
#include <unitree/idl/go2/SportModeState_.hpp>
```
Message type for high-level state (computed position, velocity).

```cpp
#include <unitree/idl/go2/LowCmd_.hpp>
```
Message type for low-level motor commands.

---

## Topic Definitions (Lines 8-10)

```cpp
#define TOPIC_LOWSTATE "rt/lowstate"
```
Topic name to receive raw sensor data from the robot.

```cpp
#define TOPIC_HIGHSTATE "rt/sportmodestate"
```
Topic name to receive computed position and velocity.

```cpp
#define TOPIC_LOWCMD "rt/lowcmd"
```
Topic name to send motor commands to the robot.

---

## Namespace Declarations (Lines 12-13)

```cpp
using namespace unitree::robot;
using namespace unitree::common;
```
Brings Unitree SDK namespaces into scope to avoid typing full paths.

---

## LowStateHandler Callback (Lines 15-23)

```cpp
void LowStateHandler(const void *msg)
{
    const unitree_go::msg::dds_::LowState_ *s = (const unitree_go::msg::dds_::LowState_ *)msg;

    std::cout << "Quaternion: "
              << s->imu_state().quaternion()[0] << " "
              << s->imu_state().quaternion()[1] << " "
              << s->imu_state().quaternion()[2] << " "
              << s->imu_state().quaternion()[3] << " " << std::endl;
}
```

**Purpose:** Called automatically when a LowState message arrives from the simulator.

**What it does:**
1. Casts the raw void pointer to a LowState message type
2. Extracts the IMU quaternion (robot orientation as w, x, y, z components)
3. Prints the quaternion values to the console

---

## HighStateHandler Callback (Lines 25-34)

```cpp
void HighStateHandler(const void *msg)
{
    const unitree_go::msg::dds_::SportModeState_ *s = (const unitree_go::msg::dds_::SportModeState_ *)msg;

    std::cout << "Position: "
              << s->position()[0] << " "
              << s->position()[1] << " "
              << s->position()[2] << " " << std::endl;
}
```

**Purpose:** Called automatically when a SportModeState message arrives.

**What it does:**
1. Casts the raw void pointer to a SportModeState message type
2. Extracts the robot position in world frame (x, y, z coordinates)
3. Prints the position values to the console

---

## Main Function

### DDS Initialization (Line 38)

```cpp
ChannelFactory::Instance()->Init(1, "lo");
```

**Purpose:** Initialize the DDS communication system.

**Parameters:**
- `1` = Domain ID (must match simulator's config.yaml setting)
- `"lo"` = Network interface (loopback for local communication)

---

### Subscriber Setup (Lines 40-44)

```cpp
ChannelSubscriber<unitree_go::msg::dds_::LowState_> lowstate_suber(TOPIC_LOWSTATE);
lowstate_suber.InitChannel(LowStateHandler);
```
Creates a subscriber for low-level state and registers the callback function.

```cpp
ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> highstate_suber(TOPIC_HIGHSTATE);
highstate_suber.InitChannel(HighStateHandler);
```
Creates a subscriber for sport mode state and registers the callback function.

---

### Publisher Setup (Lines 46-47)

```cpp
ChannelPublisher<unitree_go::msg::dds_::LowCmd_> low_cmd_puber(TOPIC_LOWCMD);
low_cmd_puber.InitChannel();
```
Creates a publisher for sending motor commands to the simulator.

---

### Control Loop (Lines 51-60)

```cpp
while (true)
{
    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].q() = 0;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 1;
    }
    low_cmd_puber.Write(low_cmd);
    usleep(2000);
}
```

**Purpose:** Continuously send motor commands to the robot.

**Line by line:**

| Line | Code | Explanation |
|------|------|-------------|
| 1 | `while (true)` | Infinite loop - runs until program is killed |
| 2 | `LowCmd_ low_cmd{}` | Create empty command message |
| 3 | `for (int i = 0; i < 20; i++)` | Loop through all 20 motor slots (Go2 has 12 motors, extras ignored) |
| 4 | `.q() = 0` | Target position = 0 (not used since kp=0) |
| 5 | `.kp() = 0` | Position gain = 0 (disables position control) |
| 6 | `.dq() = 0` | Target velocity = 0 (not used since kd=0) |
| 7 | `.kd() = 0` | Velocity gain = 0 (disables velocity control) |
| 8 | `.tau() = 1` | Direct torque = 1 Newton-meter |
| 9 | `low_cmd_puber.Write(low_cmd)` | Send command message to simulator |
| 10 | `usleep(2000)` | Wait 2000 microseconds (2ms) = ~500Hz control rate |

---

## Motor Control Equation

The motor controller uses this equation:

```
τ = τ_ff + Kp × (q_target - q_actual) + Kd × (dq_target - dq_actual)
```

Where:
- `τ` = final torque applied
- `τ_ff` = feedforward torque (the `.tau()` value)
- `Kp` = position gain (the `.kp()` value)
- `Kd` = velocity gain (the `.kd()` value)
- `q` = joint position
- `dq` = joint velocity

In this test, since Kp=0 and Kd=0, only the feedforward torque (1 Nm) is applied.

---

## Summary

This test program:
1. **Subscribes** to robot state topics and prints IMU orientation and position
2. **Publishes** motor commands sending 1Nm torque to all motors
3. Runs at approximately 500Hz control frequency
4. Causes the robot to move/twitch in the simulator due to the constant torque

---

## Usage

1. Start the simulator in one terminal:
   ```bash
   ./unitree_mujoco -r go2 -s scene_terrain.xml
   ```

2. Run the test in another terminal:
   ```bash
   ./test
   ```

3. Press Ctrl+C to stop the test.
