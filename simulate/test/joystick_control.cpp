// Joystick Controller for Go2W (wheeled quadruped)
// Drive Mako around with Xbox controller using wheels + legs
//
// Controls:
// - Left stick Y: Drive forward/backward (wheels)
// - Left stick X: Strafe (not used for wheeled mode)
// - Right stick X: Turn (differential wheel speed)
// - A button: Stand up
// - B button: Sit down
// - LB/RB: Raise/lower body height

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_JOYSTICK "rt/wirelesscontroller"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// Motor indices for Go2W (16 motors total):
// Legs:
//   0: FR_hip    1: FR_thigh    2: FR_calf
//   3: FL_hip    4: FL_thigh    5: FL_calf
//   6: RR_hip    7: RR_thigh    8: RR_calf
//   9: RL_hip   10: RL_thigh   11: RL_calf
// Wheels:
//  12: FR_wheel  13: FL_wheel  14: RR_wheel  15: RL_wheel

class JoystickControl
{
public:
    JoystickControl(){};
    ~JoystickControl(){};
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *msg);
    void JoystickHandler(const void *msg);
    void LowCmdWrite();

private:
    // Sitting position (crouched, wheels on ground)
    double sit_pos[12] = {
        0.0, 1.1, -2.0,      // FR: hip, thigh, calf
       -0.0, 1.1, -2.0,      // FL
        0.0, 1.1, -2.0,      // RR
       -0.0, 1.1, -2.0       // RL
    };

    // Standing position (wheels on ground, body raised)
    double stand_pos[12] = {
        0.0,  0.6, -1.2,     // FR
       -0.0,  0.6, -1.2,     // FL
        0.0,  0.8, -1.5,     // RR (rear slightly different for wheel contact)
       -0.0,  0.8, -1.5      // RL
    };

    // High stance (body raised higher)
    double high_pos[12] = {
        0.0,  0.3, -0.7,     // FR
       -0.0,  0.3, -0.7,     // FL
        0.0,  0.5, -1.0,     // RR
       -0.0,  0.5, -1.0      // RL
    };

    // Joystick inputs (normalized -1 to 1)
    double lx = 0.0;  // Left stick X
    double ly = 0.0;  // Left stick Y (forward/back)
    double rx = 0.0;  // Right stick X (turn)
    double ry = 0.0;  // Right stick Y
    uint16_t keys = 0;

    // Robot state
    bool standing = true;
    double stand_phase = 0.0;
    double height_adjust = 0.0;  // -1 to 1, adjusts between stand and high pos
    bool joystick_connected = false;

    // Wheel parameters
    double max_wheel_speed = 20.0;   // Max wheel velocity
    double max_turn_rate = 10.0;     // Max differential for turning
    double wheel_kd = 3.0;           // Wheel damping gain

    double dt = 0.002;
    double running_time = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ChannelSubscriberPtr<unitree_go::msg::dds_::WirelessController_> joystick_subscriber;
    ThreadPtr lowCmdWriteThreadPtr;
};

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void JoystickControl::Init()
{
    InitLowCmd();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&JoystickControl::LowStateMessageHandler, this, std::placeholders::_1), 1);

    joystick_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::WirelessController_>(TOPIC_JOYSTICK));
    joystick_subscriber->InitChannel(std::bind(&JoystickControl::JoystickHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("joystick_ctrl", UT_CPU_ID_NONE, int(dt * 1000000), &JoystickControl::LowCmdWrite, this);
}

void JoystickControl::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = 0x01;
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

void JoystickControl::LowStateMessageHandler(const void *msg)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)msg;
}

void JoystickControl::JoystickHandler(const void *msg)
{
    const unitree_go::msg::dds_::WirelessController_ *js =
        (const unitree_go::msg::dds_::WirelessController_ *)msg;

    lx = js->lx();
    ly = js->ly();
    rx = js->rx();
    ry = js->ry();
    keys = js->keys();

    // Apply deadzone
    if (fabs(lx) < 0.1) lx = 0;
    if (fabs(ly) < 0.1) ly = 0;
    if (fabs(rx) < 0.1) rx = 0;

    joystick_connected = true;

    // Debug: print keys when pressed
    if (keys != 0)
    {
        printf("Keys: 0x%04X\n", keys);
    }

    // Button handling (try common mappings)
    // A button = stand
    if (keys & 0x01) standing = true;
    // B button = sit
    if (keys & 0x02) standing = false;
    // LB = lower body
    if (keys & 0x10) height_adjust -= 0.02;
    // RB = raise body
    if (keys & 0x20) height_adjust += 0.02;

    // Clamp height
    if (height_adjust < -0.5) height_adjust = -0.5;
    if (height_adjust > 1.0) height_adjust = 1.0;
}

void JoystickControl::LowCmdWrite()
{
    running_time += dt;

    // Smooth stand/sit transition
    if (standing && stand_phase < 1.0)
    {
        stand_phase += dt * 2.0;
        if (stand_phase > 1.0) stand_phase = 1.0;
    }
    else if (!standing && stand_phase > 0.0)
    {
        stand_phase -= dt * 2.0;
        if (stand_phase < 0.0) stand_phase = 0.0;
    }

    // Calculate target leg positions
    double target_pos[12];
    double kp = 50.0;
    double kd = 4.0;

    // Interpolate based on stand phase and height adjustment
    for (int i = 0; i < 12; i++)
    {
        // Base: sit to stand
        double base_pos = stand_phase * stand_pos[i] + (1 - stand_phase) * sit_pos[i];

        // Apply height adjustment when standing
        if (stand_phase > 0.5 && height_adjust > 0)
        {
            base_pos = base_pos + height_adjust * (high_pos[i] - stand_pos[i]);
        }

        target_pos[i] = base_pos;
    }

    // Lean into turns for stability
    if (stand_phase > 0.9 && fabs(rx) > 0.1)
    {
        double lean = rx * 0.1;  // Lean amount
        // Adjust hip abduction for leaning
        target_pos[0] -= lean;   // FR hip
        target_pos[3] -= lean;   // FL hip
        target_pos[6] -= lean;   // RR hip
        target_pos[9] -= lean;   // RL hip
    }

    // Apply leg commands
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = target_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = kp;
        low_cmd.motor_cmd()[i].kd() = kd;
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    // Wheel control - only when standing
    double wheel_speed = 0;
    double turn_diff = 0;

    if (stand_phase > 0.8)
    {
        wheel_speed = ly * max_wheel_speed;
        turn_diff = rx * max_turn_rate;
    }

    // FR wheel (12) - right front
    low_cmd.motor_cmd()[12].q() = 0;
    low_cmd.motor_cmd()[12].dq() = wheel_speed + turn_diff;
    low_cmd.motor_cmd()[12].kp() = 0;
    low_cmd.motor_cmd()[12].kd() = wheel_kd;
    low_cmd.motor_cmd()[12].tau() = 0;

    // FL wheel (13) - left front (reversed direction)
    low_cmd.motor_cmd()[13].q() = 0;
    low_cmd.motor_cmd()[13].dq() = -(wheel_speed - turn_diff);
    low_cmd.motor_cmd()[13].kp() = 0;
    low_cmd.motor_cmd()[13].kd() = wheel_kd;
    low_cmd.motor_cmd()[13].tau() = 0;

    // RR wheel (14) - right rear
    low_cmd.motor_cmd()[14].q() = 0;
    low_cmd.motor_cmd()[14].dq() = wheel_speed + turn_diff;
    low_cmd.motor_cmd()[14].kp() = 0;
    low_cmd.motor_cmd()[14].kd() = wheel_kd;
    low_cmd.motor_cmd()[14].tau() = 0;

    // RL wheel (15) - left rear (reversed direction)
    low_cmd.motor_cmd()[15].q() = 0;
    low_cmd.motor_cmd()[15].dq() = -(wheel_speed - turn_diff);
    low_cmd.motor_cmd()[15].kp() = 0;
    low_cmd.motor_cmd()[15].kd() = wheel_kd;
    low_cmd.motor_cmd()[15].tau() = 0;

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);

    // Status output
    static double last_print = 0;
    if (running_time - last_print > 0.5)
    {
        printf("Wheels: %.1f | Turn: %.1f | Height: %.2f | %s | JS: %s\n",
               wheel_speed, turn_diff, height_adjust,
               standing ? "STANDING" : "SITTING",
               joystick_connected ? "OK" : "waiting...");
        last_print = running_time;
    }
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }

    std::cout << "=== Go2W Joystick Controller ===" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  Left Stick Y  : Drive forward/backward" << std::endl;
    std::cout << "  Right Stick X : Turn left/right" << std::endl;
    std::cout << "  A Button      : Stand up" << std::endl;
    std::cout << "  B Button      : Sit down" << std::endl;
    std::cout << "  LB / RB       : Lower / Raise body" << std::endl;
    std::cout << std::endl;
    std::cout << "Press Enter to start...";
    std::cin.get();

    JoystickControl controller;
    controller.Init();

    std::cout << "Controller active! Robot will stand up automatically." << std::endl;

    while (1)
    {
        sleep(10);
    }

    return 0;
}
