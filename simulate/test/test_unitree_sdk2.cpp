// DDS subscriber for receiving messages from the simulator
#include <unitree/robot/channel/channel_subscriber.hpp>
// DDS publisher for sending messages to the simulator
#include <unitree/robot/channel/channel_publisher.hpp>
// Time utilities
#include <unitree/common/time/time_tool.hpp>
// Message type for low-level robot state (IMU data, motor states)
#include <unitree/idl/go2/LowState_.hpp>
// Message type for high-level state (computed position, velocity)
#include <unitree/idl/go2/SportModeState_.hpp>
// Message type for low-level motor commands
#include <unitree/idl/go2/LowCmd_.hpp>

// Topic names for DDS communication
#define TOPIC_LOWSTATE "rt/lowstate"        // Receive raw sensor data
#define TOPIC_HIGHSTATE "rt/sportmodestate" // Receive computed position/velocity
#define TOPIC_LOWCMD "rt/lowcmd"            // Send motor commands

using namespace unitree::robot;
using namespace unitree::common;

// Callback function called when a LowState message arrives from the simulator
// Extracts and prints the IMU quaternion (robot orientation as w, x, y, z)
void LowStateHandler(const void *msg)
{
    // Cast raw pointer to LowState message type
    const unitree_go::msg::dds_::LowState_ *s = (const unitree_go::msg::dds_::LowState_ *)msg;

    // Print IMU quaternion values
    std::cout << "Quaternion: "
              << s->imu_state().quaternion()[0] << " "
              << s->imu_state().quaternion()[1] << " "
              << s->imu_state().quaternion()[2] << " "
              << s->imu_state().quaternion()[3] << " " << std::endl;
}

// Callback function called when a SportModeState message arrives
// Extracts and prints the robot position in world frame (x, y, z)
void HighStateHandler(const void *msg)
{
    // Cast raw pointer to SportModeState message type
    const unitree_go::msg::dds_::SportModeState_ *s = (const unitree_go::msg::dds_::SportModeState_ *)msg;

    // Print robot position in world coordinates
    std::cout << "Position: "
              << s->position()[0] << " "
              << s->position()[1] << " "
              << s->position()[2] << " " << std::endl;
}

int main()
{
    // Initialize DDS communication
    // Parameter 1: Domain ID (must match simulator's config.yaml)
    // Parameter 2: Network interface ("lo" = loopback for local communication)
    ChannelFactory::Instance()->Init(1, "lo");

    // Create subscriber for low-level state and register callback
    ChannelSubscriber<unitree_go::msg::dds_::LowState_> lowstate_suber(TOPIC_LOWSTATE);
    lowstate_suber.InitChannel(LowStateHandler);

    // Create subscriber for sport mode state and register callback
    ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> highstate_suber(TOPIC_HIGHSTATE);
    highstate_suber.InitChannel(HighStateHandler);

    // Create publisher for sending motor commands
    ChannelPublisher<unitree_go::msg::dds_::LowCmd_> low_cmd_puber(TOPIC_LOWCMD);
    low_cmd_puber.InitChannel();

    // Main control loop - runs at ~500Hz (2ms period)
    while (true)
    {
        // Create empty command message
        unitree_go::msg::dds_::LowCmd_ low_cmd{};

        // Configure all 20 motor slots (Go2 has 12 motors, extras ignored)
        // Motor control equation: τ = τ_ff + Kp*(q_target - q_actual) + Kd*(dq_target - dq_actual)
        for (int i = 0; i < 20; i++)
        {
            low_cmd.motor_cmd()[i].q() = 0;     // Target position (unused since kp=0)
            low_cmd.motor_cmd()[i].kp() = 0;    // Position gain (0 = no position control)
            low_cmd.motor_cmd()[i].dq() = 0;    // Target velocity (unused since kd=0)
            low_cmd.motor_cmd()[i].kd() = 0;    // Velocity gain (0 = no velocity control)
            low_cmd.motor_cmd()[i].tau() = 1;   // Feedforward torque = 1 Nm
        }

        // Send command to simulator
        low_cmd_puber.Write(low_cmd);

        // Wait 2ms before next command (~500Hz control rate)
        usleep(2000);
    }

    return 0;
}
