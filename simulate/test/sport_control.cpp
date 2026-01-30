// Sport-like control for Go2W in MuJoCo using low-level motor commands
// Usage: ./sport_control
// Commands: list, stand_up, stand_down, move, stop, balance, damp, quit

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string>
#include <atomic>
#include <thread>
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

// Command options
enum Command {
    CMD_NONE = -1,
    CMD_DAMP = 0,
    CMD_STAND_UP = 1,
    CMD_STAND_DOWN = 2,
    CMD_MOVE_FORWARD = 3,
    CMD_MOVE_BACKWARD = 4,
    CMD_TURN_LEFT = 5,
    CMD_TURN_RIGHT = 6,
    CMD_STOP = 7,
    CMD_BALANCE = 8,
    CMD_QUIT = 9
};

// Target positions for different states
// Go2W: 12 leg motors (0-11) + 4 wheel motors (12-15)
const float pos_stand[12] = {
    0.0, 0.67, -1.3,    // FR: hip, thigh, calf
    0.0, 0.67, -1.3,    // FL
    0.0, 0.67, -1.3,    // RR
    0.0, 0.67, -1.3     // RL
};

const float pos_down[12] = {
    0.0, 1.36, -2.65,   // FR
    0.0, 1.36, -2.65,   // FL
    -0.2, 1.36, -2.65,  // RR
    0.2, 1.36, -2.65    // RL
};

class SportControl {
public:
    SportControl() : current_cmd(CMD_NONE), running(true) {}

    void Init();
    void Start();
    void Stop() { running = false; }
    void SetCommand(Command cmd) { current_cmd = cmd; }

private:
    void InitLowCmd();
    void LowStateHandler(const void* message);
    void ControlLoop();
    void MoveToPosition(const float* target, float duration_ms);
    uint32_t Crc32(uint32_t* ptr, uint32_t len);

    float Kp = 70.0;
    float Kd = 5.0;
    float wheel_speed = 0.0;
    float turn_diff = 0.0;

    std::atomic<Command> current_cmd;
    std::atomic<bool> running;

    float current_pos[12];
    float target_pos[12];
    float start_pos[12];
    float transition_progress = 1.0;
    float transition_duration = 500.0;  // ms
    int motion_time = 0;
    bool first_run = true;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> cmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> state_subscriber;
    ThreadPtr control_thread;
};

uint32_t SportControl::Crc32(uint32_t* ptr, uint32_t len) {
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

void SportControl::Init() {
    InitLowCmd();

    cmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    cmd_publisher->InitChannel();

    state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    state_subscriber->InitChannel(std::bind(&SportControl::LowStateHandler, this, std::placeholders::_1), 1);

    // Initialize target to standing position
    for (int i = 0; i < 12; i++) {
        target_pos[i] = pos_stand[i];
    }
}

void SportControl::InitLowCmd() {
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for (int i = 0; i < 20; i++) {
        low_cmd.motor_cmd()[i].mode() = 0x01;
        low_cmd.motor_cmd()[i].q() = PosStopF;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].dq() = VelStopF;
        low_cmd.motor_cmd()[i].kd() = 0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }
}

void SportControl::LowStateHandler(const void* message) {
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void SportControl::Start() {
    control_thread = CreateRecurrentThreadEx("control_loop", UT_CPU_ID_NONE, 2000, &SportControl::ControlLoop, this);
}

void SportControl::ControlLoop() {
    if (!running) return;

    motion_time++;

    // Wait a bit for state data
    if (motion_time < 250) return;

    // First run - capture current position
    if (first_run) {
        for (int i = 0; i < 12; i++) {
            current_pos[i] = low_state.motor_state()[i].q();
            start_pos[i] = current_pos[i];
            target_pos[i] = current_pos[i];
        }
        first_run = false;
        std::cout << "Ready. Type 'list' for commands." << std::endl;
    }

    // Process command changes
    Command cmd = current_cmd.exchange(CMD_NONE);

    switch (cmd) {
        case CMD_STAND_UP:
            for (int i = 0; i < 12; i++) {
                start_pos[i] = low_state.motor_state()[i].q();
                target_pos[i] = pos_stand[i];
            }
            transition_progress = 0;
            transition_duration = 500;
            wheel_speed = 0;
            turn_diff = 0;
            std::cout << "Standing up..." << std::endl;
            break;

        case CMD_STAND_DOWN:
            for (int i = 0; i < 12; i++) {
                start_pos[i] = low_state.motor_state()[i].q();
                target_pos[i] = pos_down[i];
            }
            transition_progress = 0;
            transition_duration = 500;
            wheel_speed = 0;
            turn_diff = 0;
            std::cout << "Lying down..." << std::endl;
            break;

        case CMD_MOVE_FORWARD:
            wheel_speed = 5.0;
            turn_diff = 0;
            std::cout << "Moving forward..." << std::endl;
            break;

        case CMD_MOVE_BACKWARD:
            wheel_speed = -5.0;
            turn_diff = 0;
            std::cout << "Moving backward..." << std::endl;
            break;

        case CMD_TURN_LEFT:
            wheel_speed = 0;
            turn_diff = 3.0;
            std::cout << "Turning left..." << std::endl;
            break;

        case CMD_TURN_RIGHT:
            wheel_speed = 0;
            turn_diff = -3.0;
            std::cout << "Turning right..." << std::endl;
            break;

        case CMD_STOP:
            wheel_speed = 0;
            turn_diff = 0;
            std::cout << "Stopping..." << std::endl;
            break;

        case CMD_BALANCE:
            for (int i = 0; i < 12; i++) {
                start_pos[i] = low_state.motor_state()[i].q();
                target_pos[i] = pos_stand[i];
            }
            transition_progress = 0;
            transition_duration = 300;
            wheel_speed = 0;
            turn_diff = 0;
            std::cout << "Balance stand..." << std::endl;
            break;

        case CMD_DAMP:
            Kp = 0;
            Kd = 5.0;
            wheel_speed = 0;
            turn_diff = 0;
            std::cout << "Damp mode (motors relaxed)..." << std::endl;
            break;

        default:
            break;
    }

    // Update transition
    if (transition_progress < 1.0) {
        transition_progress += 1.0 / transition_duration;
        if (transition_progress > 1.0) transition_progress = 1.0;
    }

    // Set leg motor commands
    for (int i = 0; i < 12; i++) {
        float pos = (1.0 - transition_progress) * start_pos[i] + transition_progress * target_pos[i];
        low_cmd.motor_cmd()[i].q() = pos;
        low_cmd.motor_cmd()[i].dq() = 0;
        low_cmd.motor_cmd()[i].kp() = Kp;
        low_cmd.motor_cmd()[i].kd() = Kd;
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    // Set wheel motor commands (velocity control)
    // FR wheel (12), FL wheel (13), RR wheel (14), RL wheel (15)
    // Note: In Go2W model, all wheels use same dq sign for forward motion
    float right_speed = wheel_speed - turn_diff;
    float left_speed = wheel_speed + turn_diff;

    // All wheels: same dq = forward (model has axes set up this way)
    for (int i = 12; i < 16; i++) {
        low_cmd.motor_cmd()[i].q() = 0;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = 3.0;
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    // Right wheels (12=FR, 14=RR)
    low_cmd.motor_cmd()[12].dq() = right_speed;
    low_cmd.motor_cmd()[14].dq() = right_speed;

    // Left wheels (13=FL, 15=RL)
    low_cmd.motor_cmd()[13].dq() = left_speed;
    low_cmd.motor_cmd()[15].dq() = left_speed;

    // Calculate CRC and publish
    low_cmd.crc() = Crc32((uint32_t*)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    cmd_publisher->Write(low_cmd);
}

void PrintHelp() {
    std::cout << "\nAvailable commands:" << std::endl;
    std::cout << "  list        - Show this help" << std::endl;
    std::cout << "  stand_up    - Stand up" << std::endl;
    std::cout << "  stand_down  - Lie down" << std::endl;
    std::cout << "  forward     - Move forward (wheels)" << std::endl;
    std::cout << "  backward    - Move backward (wheels)" << std::endl;
    std::cout << "  left        - Turn left" << std::endl;
    std::cout << "  right       - Turn right" << std::endl;
    std::cout << "  stop        - Stop moving" << std::endl;
    std::cout << "  balance     - Balance stand" << std::endl;
    std::cout << "  damp        - Relax motors (damp mode)" << std::endl;
    std::cout << "  quit        - Exit program" << std::endl;
    std::cout << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "Sport Control for Go2W (MuJoCo)" << std::endl;
    std::cout << "================================" << std::endl;

    ChannelFactory::Instance()->Init(0, "lo");

    SportControl controller;
    controller.Init();
    controller.Start();

    PrintHelp();

    std::string input;
    while (true) {
        std::cout << "> ";
        std::getline(std::cin, input);

        if (input == "list" || input == "help") {
            PrintHelp();
        } else if (input == "stand_up" || input == "1") {
            controller.SetCommand(CMD_STAND_UP);
        } else if (input == "stand_down" || input == "2") {
            controller.SetCommand(CMD_STAND_DOWN);
        } else if (input == "forward" || input == "f" || input == "3") {
            controller.SetCommand(CMD_MOVE_FORWARD);
        } else if (input == "backward" || input == "b" || input == "4") {
            controller.SetCommand(CMD_MOVE_BACKWARD);
        } else if (input == "left" || input == "l" || input == "5") {
            controller.SetCommand(CMD_TURN_LEFT);
        } else if (input == "right" || input == "r" || input == "6") {
            controller.SetCommand(CMD_TURN_RIGHT);
        } else if (input == "stop" || input == "s" || input == "7") {
            controller.SetCommand(CMD_STOP);
        } else if (input == "balance" || input == "8") {
            controller.SetCommand(CMD_BALANCE);
        } else if (input == "damp" || input == "0") {
            controller.SetCommand(CMD_DAMP);
        } else if (input == "quit" || input == "q" || input == "exit") {
            std::cout << "Exiting..." << std::endl;
            controller.Stop();
            break;
        } else if (!input.empty()) {
            std::cout << "Unknown command. Type 'list' for help." << std::endl;
        }
    }

    return 0;
}
