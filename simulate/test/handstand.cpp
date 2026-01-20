// Handstand controller for Go2 robot
// Makes "Mako" do a handstand by balancing on front legs

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

class Handstand
{
public:
    Handstand(){};
    ~Handstand(){};
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

private:
    // Phase 0: Starting position (crouched)
    double start_pos[12] = {
        0.0, 1.22, -2.44,    // FR: hip, thigh, calf
       -0.0, 1.22, -2.44,    // FL
        0.0, 1.22, -2.44,    // RR
       -0.0, 1.22, -2.44     // RL
    };

    // Phase 1: Shift weight forward - lean onto front legs
    double lean_forward_pos[12] = {
        // Front legs - bend forward to shift COM ahead
        0.0,  0.8, -1.6,     // FR: thigh more vertical, knee bent
       -0.0,  0.8, -1.6,     // FL

        // Rear legs - push body forward, extend back
        0.0,  1.8, -2.4,     // RR: thigh pushes forward
       -0.0,  1.8, -2.4      // RL
    };

    // Phase 2: Final handstand - balanced on front two legs
    double handstand_pos[12] = {
        // Front legs - straight down, supporting all weight
        0.0,  0.4, -0.85,    // FR: nearly straight leg
       -0.0,  0.4, -0.85,    // FL

        // Rear legs - lifted up in the air
        0.0,  4.0, -2.0,     // RR: fully rotated up
       -0.0,  4.0, -2.0      // RL
    };

    double dt = 0.002;           // 2ms control period (500Hz)
    double running_time = 0.0;
    double phase = 0.0;

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

void Handstand::Init()
{
    InitLowCmd();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Handstand::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("handstand", UT_CPU_ID_NONE, int(dt * 1000000), &Handstand::LowCmdWrite, this);
}

void Handstand::InitLowCmd()
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

void Handstand::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;
}

void Handstand::LowCmdWrite()
{
    running_time += dt;

    double target_pos[12];
    double kp_front = 50.0;
    double kp_rear = 50.0;
    double kd = 4.0;

    // Phase 1 (0-2s): Lean forward, shift weight onto front legs
    if (running_time < 2.0)
    {
        phase = tanh(running_time / 0.8);
        for (int i = 0; i < 12; i++)
        {
            target_pos[i] = phase * lean_forward_pos[i] + (1 - phase) * start_pos[i];
        }
        kp_front = 60.0;
        kp_rear = 60.0;
    }
    // Phase 2 (2-5s): Slowly lift rear legs while keeping balance
    else if (running_time < 5.0)
    {
        phase = tanh((running_time - 2.0) / 1.5);
        for (int i = 0; i < 12; i++)
        {
            target_pos[i] = phase * handstand_pos[i] + (1 - phase) * lean_forward_pos[i];
        }
        kp_front = 100.0;  // Very stiff front legs
        kp_rear = 40.0;
    }
    // Phase 3 (5s+): Hold handstand
    else
    {
        for (int i = 0; i < 12; i++)
        {
            target_pos[i] = handstand_pos[i];
        }
        kp_front = 120.0;
        kp_rear = 50.0;
    }

    // Apply commands
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = target_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;

        if (i < 6)  // Front legs
        {
            low_cmd.motor_cmd()[i].kp() = kp_front;
            low_cmd.motor_cmd()[i].kd() = kd + 1.0;
        }
        else  // Rear legs
        {
            low_cmd.motor_cmd()[i].kp() = kp_rear;
            low_cmd.motor_cmd()[i].kd() = kd;
        }
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

    std::cout << "=== Mako Handstand Controller ===" << std::endl;
    std::cout << "Press Enter to make Mako do a handstand...";
    std::cin.get();

    Handstand handstand;
    handstand.Init();

    std::cout << "Mako is attempting a handstand!" << std::endl;

    while (1)
    {
        sleep(10);
    }

    return 0;
}
