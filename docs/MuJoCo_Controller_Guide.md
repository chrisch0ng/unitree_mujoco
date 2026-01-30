# Unitree MuJoCo Simulation Controller Guide

This guide explains how to create controllers that communicate with the Unitree MuJoCo simulation, based on hands-on development and troubleshooting.

---

## Table of Contents

1. [Overview](#overview)
2. [Communication Architecture](#communication-architecture)
3. [Domain ID Configuration](#domain-id-configuration)
4. [DDS Topics](#dds-topics)
5. [Motor Control Basics](#motor-control-basics)
6. [Go2W Motor Mapping](#go2w-motor-mapping)
7. [Creating a Basic Controller](#creating-a-basic-controller)
8. [High-Level vs Low-Level Control](#high-level-vs-low-level-control)
9. [Common Issues and Solutions](#common-issues-and-solutions)
10. [Example Controllers](#example-controllers)

---

## Overview

The Unitree MuJoCo simulation provides a physics-accurate environment for testing robot controllers before deploying to real hardware. Controllers communicate with the simulation using the **Unitree SDK2** library over **DDS (Data Distribution Service)**.

Key components:
- **MuJoCo Simulator** (`unitree_mujoco`): Runs the physics simulation and robot visualization
- **Controller Program**: Your code that sends motor commands and receives robot state
- **DDS Layer**: Handles communication between simulator and controller

---

## Communication Architecture

```
┌─────────────────────┐         DDS (localhost)        ┌─────────────────────┐
│                     │ ◄──────────────────────────────│                     │
│  MuJoCo Simulator   │       rt/lowstate              │    Your Controller  │
│                     │                                │                     │
│  - Physics engine   │ ──────────────────────────────►│  - Read robot state │
│  - Visualization    │       rt/lowcmd                │  - Send commands    │
│  - Robot model      │                                │  - Control logic    │
└─────────────────────┘                                └─────────────────────┘
```

Both programs must:
1. Use the same **domain_id**
2. Use the same **network interface** (typically "lo" for localhost)
3. Communicate on the same **topics**

---

## Domain ID Configuration

The **domain_id** is a number that creates isolated communication channels. Programs can only communicate if they share the same domain_id.

### Why it matters

If your controller uses `domain_id: 1` but the simulator uses `domain_id: 0`, they cannot see each other's messages - your controller won't work!

### Simulator Configuration

The simulator reads its domain_id from `simulate/config.yaml`:

```yaml
robot: "go2w"                    # Robot model to simulate
robot_scene: "scene.xml"         # Scene file
domain_id: 0                     # <-- Must match your controller!
interface: "lo"                  # Network interface (lo = localhost)
use_joystick: 0                  # Disable built-in joystick
```

### Controller Configuration

In your C++ controller, initialize DDS with matching values:

```cpp
#include <unitree/robot/channel/channel_factory.hpp>

int main() {
    // Initialize DDS: domain_id=0, interface="lo"
    unitree::robot::ChannelFactory::Instance()->Init(0, "lo");

    // ... rest of your code
}
```

### Checking for Mismatches

If you see output like this but nothing happens in the simulator:
```
1769454452.894851 [0] controller: selected interface "lo" is not multicast-capable: disabling multicast
```

The `[0]` shows domain_id 0. Make sure your `config.yaml` also has `domain_id: 0`.

---

## DDS Topics

Topics are named channels for specific types of data:

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `rt/lowcmd` | Controller → Simulator | Motor commands (position, velocity, torque) |
| `rt/lowstate` | Simulator → Controller | Robot state (joint positions, IMU, etc.) |
| `rt/wirelesscontroller` | Simulator → Controller | Joystick/gamepad input (if enabled) |
| `rt/sportmodestate` | Simulator → Controller | High-level state info |

### Publishing Commands (rt/lowcmd)

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

// Create publisher
ChannelPublisher<unitree_go::msg::dds_::LowCmd_> publisher("rt/lowcmd");
publisher.InitChannel();

// Create and send command
unitree_go::msg::dds_::LowCmd_ cmd;
// ... fill in command ...
publisher.Write(cmd);
```

### Subscribing to State (rt/lowstate)

```cpp
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>

// Callback function
void StateHandler(const void* message) {
    auto state = *(unitree_go::msg::dds_::LowState_*)message;
    // Access state data:
    // state.motor_state()[i].q()  - joint position
    // state.motor_state()[i].dq() - joint velocity
    // state.imu_state()           - IMU data
}

// Create subscriber
ChannelSubscriber<unitree_go::msg::dds_::LowState_> subscriber("rt/lowstate");
subscriber.InitChannel(StateHandler, 1);
```

---

## Motor Control Basics

Each motor is controlled using PD (Proportional-Derivative) control with feedforward torque:

```
τ = τ_ff + Kp × (q_target - q_actual) + Kd × (dq_target - dq_actual)
```

Where:
- `τ` = output torque
- `τ_ff` = feedforward torque (`tau`)
- `Kp` = position gain (`kp`)
- `Kd` = velocity gain (`kd`)
- `q_target` = target position (`q`)
- `q_actual` = current position (from lowstate)
- `dq_target` = target velocity (`dq`)
- `dq_actual` = current velocity (from lowstate)

### Position Control

Hold a specific joint angle:

```cpp
low_cmd.motor_cmd()[i].q() = target_position;  // Target angle (radians)
low_cmd.motor_cmd()[i].dq() = 0;               // Target velocity
low_cmd.motor_cmd()[i].kp() = 70.0;            // Position stiffness
low_cmd.motor_cmd()[i].kd() = 5.0;             // Damping
low_cmd.motor_cmd()[i].tau() = 0;              // No feedforward
```

### Velocity Control (for wheels)

Spin at a target velocity:

```cpp
low_cmd.motor_cmd()[i].q() = 0;                // Not used
low_cmd.motor_cmd()[i].dq() = target_velocity; // Target velocity (rad/s)
low_cmd.motor_cmd()[i].kp() = 0;               // No position control
low_cmd.motor_cmd()[i].kd() = 3.0;             // Velocity tracking gain
low_cmd.motor_cmd()[i].tau() = 0;
```

### Torque Control

Direct torque command:

```cpp
low_cmd.motor_cmd()[i].q() = PosStopF;         // Special value to disable position
low_cmd.motor_cmd()[i].dq() = VelStopF;        // Special value to disable velocity
low_cmd.motor_cmd()[i].kp() = 0;
low_cmd.motor_cmd()[i].kd() = 0;
low_cmd.motor_cmd()[i].tau() = torque_value;   // Direct torque (Nm)
```

---

## Go2W Motor Mapping

The Go2W robot has **16 motors**: 12 leg joints + 4 wheels.

### Leg Motors (0-11)

| Index | Joint | Location |
|-------|-------|----------|
| 0 | FR Hip (abduction) | Front Right |
| 1 | FR Thigh | Front Right |
| 2 | FR Calf | Front Right |
| 3 | FL Hip | Front Left |
| 4 | FL Thigh | Front Left |
| 5 | FL Calf | Front Left |
| 6 | RR Hip | Rear Right |
| 7 | RR Thigh | Rear Right |
| 8 | RR Calf | Rear Right |
| 9 | RL Hip | Rear Left |
| 10 | RL Thigh | Rear Left |
| 11 | RL Calf | Rear Left |

### Wheel Motors (12-15)

| Index | Wheel |
|-------|-------|
| 12 | Front Right |
| 13 | Front Left |
| 14 | Rear Right |
| 15 | Rear Left |

### Important: Wheel Direction Convention

In the Go2W MuJoCo model, **all wheels use the same dq sign for forward motion**. This is different from typical differential drive where left/right wheels spin opposite directions.

```cpp
// Forward motion - all wheels same sign
float speed = 5.0;
low_cmd.motor_cmd()[12].dq() = speed;  // FR
low_cmd.motor_cmd()[13].dq() = speed;  // FL
low_cmd.motor_cmd()[14].dq() = speed;  // RR
low_cmd.motor_cmd()[15].dq() = speed;  // RL

// Turning - differential steering
float turn = 2.0;
float right_speed = speed - turn;
float left_speed = speed + turn;
low_cmd.motor_cmd()[12].dq() = right_speed;  // FR
low_cmd.motor_cmd()[13].dq() = left_speed;   // FL
low_cmd.motor_cmd()[14].dq() = right_speed;  // RR
low_cmd.motor_cmd()[15].dq() = left_speed;   // RL
```

---

## Creating a Basic Controller

### Step 1: Include Headers

```cpp
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::robot;
using namespace unitree::common;
```

### Step 2: Initialize Command Structure

```cpp
void InitLowCmd(unitree_go::msg::dds_::LowCmd_& cmd) {
    cmd.head()[0] = 0xFE;
    cmd.head()[1] = 0xEF;
    cmd.level_flag() = 0xFF;
    cmd.gpio() = 0;

    for (int i = 0; i < 20; i++) {
        cmd.motor_cmd()[i].mode() = 0x01;  // Enable motor
        cmd.motor_cmd()[i].q() = 0;
        cmd.motor_cmd()[i].kp() = 0;
        cmd.motor_cmd()[i].dq() = 0;
        cmd.motor_cmd()[i].kd() = 0;
        cmd.motor_cmd()[i].tau() = 0;
    }
}
```

### Step 3: CRC Checksum (Required!)

Commands must include a valid CRC or they'll be ignored:

```cpp
uint32_t crc32_core(uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t polynomial = 0x04c11db7;

    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= polynomial;
            } else {
                CRC32 <<= 1;
            }
            if (data & xbit)
                CRC32 ^= polynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// Before publishing:
low_cmd.crc() = crc32_core((uint32_t*)&low_cmd,
                           (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
```

### Step 4: Main Loop

```cpp
int main() {
    // Initialize DDS - MUST match config.yaml!
    ChannelFactory::Instance()->Init(0, "lo");

    // Create publisher and subscriber
    ChannelPublisher<unitree_go::msg::dds_::LowCmd_> pub("rt/lowcmd");
    pub.InitChannel();

    ChannelSubscriber<unitree_go::msg::dds_::LowState_> sub("rt/lowstate");
    sub.InitChannel(StateHandler, 1);

    // Initialize command
    unitree_go::msg::dds_::LowCmd_ cmd;
    InitLowCmd(cmd);

    // Control loop (500 Hz)
    while (running) {
        // Set motor commands
        for (int i = 0; i < 12; i++) {
            cmd.motor_cmd()[i].q() = target_positions[i];
            cmd.motor_cmd()[i].kp() = 70.0;
            cmd.motor_cmd()[i].kd() = 5.0;
        }

        // Calculate CRC and publish
        cmd.crc() = crc32_core((uint32_t*)&cmd,
                               (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
        pub.Write(cmd);

        usleep(2000);  // 500 Hz
    }

    return 0;
}
```

### Step 5: Add to CMakeLists.txt

```cmake
add_executable(my_controller test/my_controller.cpp)
```

### Step 6: Build

```bash
cd simulate/build
cmake ..
make -j4 my_controller
```

---

## High-Level vs Low-Level Control

### Low-Level Control (Works with MuJoCo)

- Direct motor commands via `rt/lowcmd`
- You control individual joint positions/velocities/torques
- Full flexibility but requires more code
- **This is what MuJoCo understands**

### High-Level Control (Real Robot Only)

- Uses `SportClient` API with commands like `StandUp()`, `Move()`, etc.
- The robot's onboard computer interprets these
- Easier to use but **does NOT work with MuJoCo**
- Example from `unitree_sdk2/example/go2w/go2w_sport_client.cpp`:

```cpp
// This does NOT work with MuJoCo!
unitree::robot::go2::SportClient sport_client;
sport_client.Init();
sport_client.StandUp();   // Robot's internal controller handles this
sport_client.Move(0.5, 0, 0);  // Move forward
```

To test high-level behaviors in MuJoCo, you must implement them yourself using low-level commands (see `sport_control.cpp` example).

---

## Common Issues and Solutions

### Issue: Controller runs but robot doesn't move

**Cause**: Domain ID mismatch

**Solution**: Check both match:
```bash
# In config.yaml
domain_id: 0

# In your code
ChannelFactory::Instance()->Init(0, "lo");  // First arg is domain_id
```

### Issue: "selected interface 'lo' is not multicast-capable"

**Cause**: Normal warning, not an error

**Solution**: Ignore it - DDS falls back to unicast which works fine locally.

### Issue: Robot rotates instead of driving straight

**Cause**: Incorrect wheel direction signs

**Solution**: Go2W uses same dq sign for all wheels:
```cpp
// WRONG - typical differential drive assumption
left_wheels = speed;
right_wheels = -speed;

// CORRECT - Go2W convention
left_wheels = speed;
right_wheels = speed;
```

### Issue: Robot falls over while driving

**Cause**: Leg stiffness too low

**Solution**: Increase PD gains:
```cpp
// Too soft
kp = 50.0; kd = 4.0;

// Better for driving
kp = 120.0; kd = 6.0;
```

### Issue: Commands work but are choppy

**Cause**: Terminal key repeat delay with keyboard control

**Solution**: Only decay commands when key is NOT pressed, use slower decay rate.

---

## Example Controllers

The following example controllers are included in `simulate/test/`:

### test_unitree_sdk2.cpp
Basic DDS communication test - sends small torque to all motors.

### keyboard_control.cpp
Real-time keyboard control for Go2W:
- WASD for driving
- R/F for height adjustment
- SPACE/C for stand/sit

### sport_control.cpp
Menu-driven controller mimicking high-level commands:
- stand_up, stand_down
- forward, backward, left, right
- Works with MuJoCo (unlike the real SportClient)

### Running Examples

Terminal 1 - Start simulator:
```bash
cd simulate/build
./unitree_mujoco
```

Terminal 2 - Run controller:
```bash
cd simulate/build
./keyboard_control
# or
./sport_control
```

---

## Quick Reference

### Config File Location
`simulate/config.yaml`

### Key Settings
```yaml
robot: "go2w"      # go2, go2w, b2, h1, g1, etc.
domain_id: 0       # Must match controller
interface: "lo"    # localhost
use_joystick: 0    # 0=disabled, 1=enabled
```

### Typical PD Gains
| Application | Kp | Kd |
|-------------|----|----|
| Standing | 70 | 5 |
| Driving | 120 | 6 |
| Wheels | 0 | 3-5 |

### Control Loop Rate
500 Hz (2000 microseconds) is typical.

---

## Conclusion

Creating controllers for Unitree MuJoCo simulation requires:

1. **Matching domain_id** between simulator and controller
2. **Using low-level commands** (rt/lowcmd topic)
3. **Proper CRC checksum** on all commands
4. **Understanding motor mapping** for your specific robot
5. **Appropriate PD gains** for stable behavior

The high-level SportClient API from unitree_sdk2 does NOT work with MuJoCo - you must implement behaviors using direct motor commands.

---

*Document generated from hands-on development session, January 2026*
