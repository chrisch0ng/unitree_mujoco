// Walking controller for Go2 robot
// Makes "Mako" walk forward using a trotting gait

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::common;
using namespace unitree::robot;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// Motor indices for Go2:
// 0: FR_hip    1: FR_thigh    2: FR_calf
// 3: FL_hip    4: FL_thigh    5: FL_calf
// 6: RR_hip    7: RR_thigh    8: RR_calf
// 9: RL_hip   10: RL_thigh   11: RL_calf

class Walk
{
public:
    Walk(){};
    ~Walk(){};
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

private:
    // Standing position (neutral stance)
    double stand_pos[12] = {
        0.0,  0.7, -1.4,     // FR: hip, thigh, calf
       -0.0,  0.7, -1.4,     // FL
        0.0,  0.7, -1.4,     // RR
       -0.0,  0.7, -1.4      // RL
    };

    // Gait parameters
    double gait_freq = 1.2;          // Gait frequency in Hz (slower = more stable)
    double swing_height = 0.3;       // How high to lift feet (radians for calf)
    double step_length = 0.2;        // Forward step size (radians for thigh)

    double dt = 0.002;               // 2ms control period (500Hz)
    double running_time = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
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

void Walk::Init()
{
    InitLowCmd();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Walk::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("walk", UT_CPU_ID_NONE, int(dt * 1000000), &Walk::LowCmdWrite, this);
}

void Walk::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = 0x01;  // servo mode
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

void Walk::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void Walk::LowCmdWrite()
{
    running_time += dt;

    double target_pos[12];
    double kp = 40.0;
    double kd = 3.0;

    // Phase for gait cycle (0 to 2*PI)
    double phase = fmod(running_time * gait_freq * 2.0 * M_PI, 2.0 * M_PI);

    // First 2 seconds: stand up smoothly
    if (running_time < 2.0)
    {
        double standup_phase = tanh(running_time / 0.8);
        double crouch_pos[12] = {
            0.0, 1.22, -2.44,
           -0.0, 1.22, -2.44,
            0.0, 1.22, -2.44,
           -0.0, 1.22, -2.44
        };

        for (int i = 0; i < 12; i++)
        {
            target_pos[i] = standup_phase * stand_pos[i] + (1 - standup_phase) * crouch_pos[i];
        }
        kp = 50.0;
    }
    // After 2 seconds: start walking with trot gait
    else
    {
        // Trot gait: diagonal legs move together
        // FR + RL move together (phase 0)
        // FL + RR move together (phase PI)

        for (int i = 0; i < 12; i++)
        {
            target_pos[i] = stand_pos[i];
        }

        // Calculate leg phases
        // FR (0,1,2) and RL (9,10,11) are in phase
        // FL (3,4,5) and RR (6,7,8) are 180 degrees out of phase
        double phase_FR_RL = phase;
        double phase_FL_RR = phase + M_PI;

        // Swing phase: leg is in the air (phase 0 to PI)
        // Stance phase: leg is on ground (phase PI to 2*PI)

        // Generate smooth swing trajectory using sin
        auto leg_motion = [&](double leg_phase, int thigh_idx, int calf_idx) {
            double swing = sin(leg_phase);

            // Thigh: positive swing = move forward (increase thigh angle to step forward)
            // When swing > 0: leg swings forward (in air)
            // When swing < 0: leg pushes back (on ground, propelling robot forward)
            target_pos[thigh_idx] = stand_pos[thigh_idx] + step_length * swing;

            if (swing > 0)  // Swing phase - leg in air
            {
                // Lift foot by making calf angle more positive (less bent)
                target_pos[calf_idx] = stand_pos[calf_idx] + swing_height * swing;
            }
            else  // Stance phase - leg on ground
            {
                target_pos[calf_idx] = stand_pos[calf_idx];
            }
        };

        // Apply motion to each leg pair
        // FR (thigh=1, calf=2)
        leg_motion(phase_FR_RL, 1, 2);
        // RL (thigh=10, calf=11)
        leg_motion(phase_FR_RL, 10, 11);
        // FL (thigh=4, calf=5)
        leg_motion(phase_FL_RR, 4, 5);
        // RR (thigh=7, calf=8)
        leg_motion(phase_FL_RR, 7, 8);

        kp = 45.0;
        kd = 3.5;
    }

    // Apply commands to all motors
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = target_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = kp;
        low_cmd.motor_cmd()[i].kd() = kd;
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);
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

    std::cout << "=== Mako Walking Controller ===" << std::endl;
    std::cout << "Press Enter to make Mako walk...";
    std::cin.get();

    Walk walk;
    walk.Init();

    std::cout << "Mako is walking!" << std::endl;

    while (1)
    {
        sleep(10);
    }

    return 0;
}
