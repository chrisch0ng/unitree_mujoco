// PD Balance Controller for Go2 Handstand
// Uses IMU feedback to actively balance on front two legs

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

class HandstandPD
{
public:
    HandstandPD(){};
    ~HandstandPD(){};
    void Init();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *messages);
    void LowCmdWrite();

    // Convert quaternion to euler angles (roll, pitch, yaw)
    void QuatToEuler(const float quat[4], double &roll, double &pitch, double &yaw);

private:
    // Starting position (crouched)
    double start_pos[12] = {
        0.0, 1.22, -2.44,    // FR
       -0.0, 1.22, -2.44,    // FL
        0.0, 1.22, -2.44,    // RR
       -0.0, 1.22, -2.44     // RL
    };

    // Handstand base position (before balance corrections)
    // Front legs straight down, rear legs up
    double handstand_base[12] = {
        0.0,  0.2, -0.5,     // FR: front leg extended
       -0.0,  0.2, -0.5,     // FL
        0.0,  3.0, -1.8,     // RR: rear leg lifted
       -0.0,  3.0, -1.8      // RL
    };

    // Balance PD gains
    double pitch_kp = 0.8;    // Proportional gain for pitch error
    double pitch_kd = 0.15;   // Derivative gain for pitch rate
    double roll_kp = 0.5;     // Proportional gain for roll error
    double roll_kd = 0.1;     // Derivative gain for roll rate

    // Target pitch angle for handstand (radians, positive = leaning forward)
    double target_pitch = 0.75;  // Slightly forward to counteract rear leg weight

    // State
    double dt = 0.002;
    double running_time = 0.0;
    double last_pitch_error = 0.0;
    double last_roll_error = 0.0;
    bool state_received = false;

    // Current IMU readings
    double current_roll = 0.0;
    double current_pitch = 0.0;
    double current_yaw = 0.0;
    double gyro_x = 0.0;  // Roll rate
    double gyro_y = 0.0;  // Pitch rate

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

void HandstandPD::QuatToEuler(const float quat[4], double &roll, double &pitch, double &yaw)
{
    // Quaternion: w, x, y, z
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
}

void HandstandPD::Init()
{
    InitLowCmd();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&HandstandPD::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("handstand_pd", UT_CPU_ID_NONE, int(dt * 1000000), &HandstandPD::LowCmdWrite, this);
}

void HandstandPD::InitLowCmd()
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

void HandstandPD::LowStateMessageHandler(const void *message)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)message;

    // Extract IMU data
    const float *quat = low_state.imu_state().quaternion().data();
    QuatToEuler(quat, current_roll, current_pitch, current_yaw);

    // Get angular velocities from gyroscope
    gyro_x = low_state.imu_state().gyroscope()[0];
    gyro_y = low_state.imu_state().gyroscope()[1];

    state_received = true;
}

void HandstandPD::LowCmdWrite()
{
    running_time += dt;

    double target_pos[12];
    double kp_motor = 60.0;
    double kd_motor = 4.0;

    // Phase 1 (0-3s): Transition to handstand position
    if (running_time < 3.0)
    {
        double phase = tanh(running_time / 1.2);

        for (int i = 0; i < 12; i++)
        {
            target_pos[i] = phase * handstand_base[i] + (1 - phase) * start_pos[i];
        }

        kp_motor = 60.0 + phase * 40.0;
        kd_motor = 4.0 + phase * 2.0;
    }
    // Phase 2 (3s+): Active balance control
    else
    {
        // Start from base handstand position
        for (int i = 0; i < 12; i++)
        {
            target_pos[i] = handstand_base[i];
        }

        if (state_received)
        {
            // Calculate pitch error (how far from target tilt)
            double pitch_error = target_pitch - current_pitch;

            // Calculate roll error (want to stay level side-to-side)
            double roll_error = 0.0 - current_roll;

            // PD control for pitch (front-back balance)
            // If falling backward (pitch too low), extend front legs more
            // If falling forward (pitch too high), bend front legs
            double pitch_correction = pitch_kp * pitch_error - pitch_kd * gyro_y;

            // PD control for roll (left-right balance)
            // Adjust hip abduction to correct sideways tilt
            double roll_correction = roll_kp * roll_error - roll_kd * gyro_x;

            // Apply pitch correction to front leg thighs
            // Positive correction = extend more (smaller thigh angle)
            target_pos[1] += pitch_correction;   // FR thigh
            target_pos[4] += pitch_correction;   // FL thigh

            // Also adjust calf to keep foot position reasonable
            target_pos[2] -= pitch_correction * 0.5;   // FR calf
            target_pos[5] -= pitch_correction * 0.5;   // FL calf

            // Apply roll correction to front hip abduction
            target_pos[0] += roll_correction;    // FR hip (positive = leg out)
            target_pos[3] -= roll_correction;    // FL hip (negative = leg out, mirrored)

            // Move rear legs to counterbalance
            // Swing them opposite to pitch correction
            target_pos[7] -= pitch_correction * 0.3;   // RR thigh
            target_pos[10] -= pitch_correction * 0.3;  // RL thigh

            // Debug output every 0.5 seconds
            static double last_print = 0;
            if (running_time - last_print > 0.5)
            {
                printf("Pitch: %.2f (target: %.2f, err: %.2f) | Roll: %.2f | Correction: %.3f\n",
                       current_pitch, target_pitch, pitch_error, current_roll, pitch_correction);
                last_print = running_time;
            }

            last_pitch_error = pitch_error;
            last_roll_error = roll_error;
        }

        kp_motor = 100.0;
        kd_motor = 6.0;
    }

    // Apply commands to motors
    for (int i = 0; i < 12; i++)
    {
        low_cmd.motor_cmd()[i].q() = target_pos[i];
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;

        if (i < 6)  // Front legs - higher gains for stability
        {
            low_cmd.motor_cmd()[i].kp() = kp_motor * 1.2;
            low_cmd.motor_cmd()[i].kd() = kd_motor * 1.2;
        }
        else  // Rear legs
        {
            low_cmd.motor_cmd()[i].kp() = kp_motor * 0.8;
            low_cmd.motor_cmd()[i].kd() = kd_motor;
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

    std::cout << "=== Go2 Handstand PD Balance Controller ===" << std::endl;
    std::cout << "This controller uses IMU feedback to actively balance." << std::endl;
    std::cout << "Press Enter to start...";
    std::cin.get();

    HandstandPD controller;
    controller.Init();

    std::cout << "Attempting handstand with active balance control!" << std::endl;

    while (1)
    {
        sleep(10);
    }

    return 0;
}
