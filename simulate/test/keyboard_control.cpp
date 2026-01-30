// Keyboard Controller for Go2W (wheeled quadruped)
// Drive Mako around with keyboard
//
// Controls:
//   W/S     : Drive forward/backward
//   A/D     : Turn left/right
//   Q/E     : Strafe left/right (hip movement)
//   R/F     : Raise/lower body
//   SPACE   : Stand up
//   C       : Sit down (crouch)
//   ESC     : Quit

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
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

// Motor indices for Go2W (16 motors total):
// Legs:
//   0: FR_hip    1: FR_thigh    2: FR_calf
//   3: FL_hip    4: FL_thigh    5: FL_calf
//   6: RR_hip    7: RR_thigh    8: RR_calf
//   9: RL_hip   10: RL_thigh   11: RL_calf
// Wheels:
//  12: FR_wheel  13: FL_wheel  14: RR_wheel  15: RL_wheel

// Terminal handling for non-blocking keyboard input
struct termios orig_termios;

void disableRawMode()
{
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

void enableRawMode()
{
    tcgetattr(STDIN_FILENO, &orig_termios);
    atexit(disableRawMode);

    struct termios raw = orig_termios;
    raw.c_lflag &= ~(ECHO | ICANON);  // Disable echo and canonical mode
    raw.c_cc[VMIN] = 0;   // Non-blocking
    raw.c_cc[VTIME] = 0;  // No timeout
    tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

int kbhit()
{
    struct timeval tv = {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
}

class KeyboardControl
{
public:
    KeyboardControl(){};
    ~KeyboardControl(){};
    void Init();
    void ProcessKeyboard();

private:
    void InitLowCmd();
    void LowStateMessageHandler(const void *msg);
    void LowCmdWrite();

private:
    // Sitting position (crouched low)
    double sit_pos[12] = {
        0.0, 1.1, -2.2,      // FR
       -0.0, 1.1, -2.2,      // FL
        0.0, 1.1, -2.2,      // RR
       -0.0, 1.1, -2.2       // RL
    };

    // Standing position (level body, all wheels on ground)
    // Slightly lower stance for stability while driving
    double stand_pos[12] = {
        0.0,  0.7, -1.4,     // FR
       -0.0,  0.7, -1.4,     // FL
        0.0,  0.7, -1.4,     // RR (same as front for level stance)
       -0.0,  0.7, -1.4      // RL
    };

    // High stance
    double high_pos[12] = {
        0.0,  0.4, -0.9,     // FR
       -0.0,  0.4, -0.9,     // FL
        0.0,  0.4, -0.9,     // RR
       -0.0,  0.4, -0.9      // RL
    };

    // Control inputs
    double forward_cmd = 0.0;   // W/S
    double turn_cmd = 0.0;      // A/D
    double strafe_cmd = 0.0;    // Q/E
    double height_cmd = 0.0;    // R/F

    // Robot state
    bool standing = true;
    double stand_phase = 0.0;
    bool running = true;

    // Parameters
    double max_wheel_speed = 8.0;   // Reduced for stability
    double max_turn_rate = 5.0;
    double wheel_kd = 5.0;          // Stiffer wheel response
    double input_decay = 0.995;  // Slow decay when key released (smoother stopping)

    double dt = 0.002;
    double running_time = 0.0;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;
    ThreadPtr lowCmdWriteThreadPtr;

public:
    bool isRunning() { return running; }
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

void KeyboardControl::Init()
{
    InitLowCmd();

    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&KeyboardControl::LowStateMessageHandler, this, std::placeholders::_1), 1);

    lowCmdWriteThreadPtr = CreateRecurrentThreadEx("keyboard_ctrl", UT_CPU_ID_NONE, int(dt * 1000000), &KeyboardControl::LowCmdWrite, this);
}

void KeyboardControl::InitLowCmd()
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

void KeyboardControl::LowStateMessageHandler(const void *msg)
{
    low_state = *(unitree_go::msg::dds_::LowState_ *)msg;
}

void KeyboardControl::ProcessKeyboard()
{
    // Track if movement keys were pressed this cycle
    bool forward_pressed = false;
    bool turn_pressed = false;
    bool strafe_pressed = false;

    // Process all available keypresses
    while (kbhit())
    {
        char c = getchar();

        switch (c)
        {
        // Movement
        case 'w':
        case 'W':
            forward_cmd = 1.0;
            forward_pressed = true;
            break;
        case 's':
        case 'S':
            forward_cmd = -1.0;
            forward_pressed = true;
            break;
        case 'a':
        case 'A':
            turn_cmd = -1.0;
            turn_pressed = true;
            break;
        case 'd':
        case 'D':
            turn_cmd = 1.0;
            turn_pressed = true;
            break;
        case 'q':
        case 'Q':
            strafe_cmd = -0.3;
            strafe_pressed = true;
            break;
        case 'e':
        case 'E':
            strafe_cmd = 0.3;
            strafe_pressed = true;
            break;

        // Height
        case 'r':
        case 'R':
            height_cmd += 0.05;
            if (height_cmd > 1.0) height_cmd = 1.0;
            break;
        case 'f':
        case 'F':
            height_cmd -= 0.05;
            if (height_cmd < 0.0) height_cmd = 0.0;
            break;

        // Stand/Sit
        case ' ':
            standing = true;
            break;
        case 'c':
        case 'C':
            standing = false;
            break;

        // Quit
        case 27:  // ESC
        case 'x':
        case 'X':
            running = false;
            break;
        }
    }

    // Only decay commands if no key was pressed (handles key repeat delay)
    if (!forward_pressed) forward_cmd *= input_decay;
    if (!turn_pressed) turn_cmd *= input_decay;
    if (!strafe_pressed) strafe_cmd *= input_decay;

    // Snap small values to zero
    if (fabs(forward_cmd) < 0.05) forward_cmd = 0;
    if (fabs(turn_cmd) < 0.05) turn_cmd = 0;
    if (fabs(strafe_cmd) < 0.05) strafe_cmd = 0;
}

void KeyboardControl::LowCmdWrite()
{
    running_time += dt;

    // Stand/sit transition
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

    // Calculate leg positions
    double target_pos[12];
    double kp = 120.0;  // High stiffness for stable driving
    double kd = 6.0;

    for (int i = 0; i < 12; i++)
    {
        double base_pos = stand_phase * stand_pos[i] + (1 - stand_phase) * sit_pos[i];

        // Apply height adjustment
        if (stand_phase > 0.5 && height_cmd > 0)
        {
            base_pos = base_pos + height_cmd * (high_pos[i] - stand_pos[i]);
        }

        target_pos[i] = base_pos;
    }

    // Apply strafe (hip abduction)
    if (stand_phase > 0.9)
    {
        target_pos[0] += strafe_cmd;   // FR hip
        target_pos[3] += strafe_cmd;   // FL hip
        target_pos[6] += strafe_cmd;   // RR hip
        target_pos[9] += strafe_cmd;   // RL hip
    }

    // Lean into turns
    if (stand_phase > 0.9 && fabs(turn_cmd) > 0.1)
    {
        double lean = turn_cmd * 0.08;
        target_pos[0] -= lean;
        target_pos[3] -= lean;
        target_pos[6] -= lean;
        target_pos[9] -= lean;
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

    // Wheel control
    double wheel_speed = 0;
    double turn_diff = 0;

    if (stand_phase > 0.8)
    {
        wheel_speed = forward_cmd * max_wheel_speed;
        turn_diff = turn_cmd * max_turn_rate;
    }

    // Go2W model: all wheels use same dq sign for forward motion
    // Right wheels (12=FR, 14=RR): wheel_speed - turn_diff
    // Left wheels (13=FL, 15=RL): wheel_speed + turn_diff
    double right_speed = wheel_speed - turn_diff;
    double left_speed = wheel_speed + turn_diff;

    // FR wheel (12)
    low_cmd.motor_cmd()[12].q() = 0;
    low_cmd.motor_cmd()[12].dq() = right_speed;
    low_cmd.motor_cmd()[12].kp() = 0;
    low_cmd.motor_cmd()[12].kd() = wheel_kd;
    low_cmd.motor_cmd()[12].tau() = 0;

    // FL wheel (13)
    low_cmd.motor_cmd()[13].q() = 0;
    low_cmd.motor_cmd()[13].dq() = left_speed;
    low_cmd.motor_cmd()[13].kp() = 0;
    low_cmd.motor_cmd()[13].kd() = wheel_kd;
    low_cmd.motor_cmd()[13].tau() = 0;

    // RR wheel (14)
    low_cmd.motor_cmd()[14].q() = 0;
    low_cmd.motor_cmd()[14].dq() = right_speed;
    low_cmd.motor_cmd()[14].kp() = 0;
    low_cmd.motor_cmd()[14].kd() = wheel_kd;
    low_cmd.motor_cmd()[14].tau() = 0;

    // RL wheel (15)
    low_cmd.motor_cmd()[15].q() = 0;
    low_cmd.motor_cmd()[15].dq() = left_speed;
    low_cmd.motor_cmd()[15].kp() = 0;
    low_cmd.motor_cmd()[15].kd() = wheel_kd;
    low_cmd.motor_cmd()[15].tau() = 0;

    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher->Write(low_cmd);

    // Status output
    static double last_print = 0;
    static int cmd_count = 0;
    cmd_count++;

    if (running_time - last_print > 0.5)
    {
        printf("\r[%s] Fwd:%.1f Turn:%.1f Height:%.1f Phase:%.2f Cmds:%d   ",
               standing ? "STAND" : "SIT  ",
               forward_cmd, turn_cmd, height_cmd, stand_phase, cmd_count);
        fflush(stdout);
        last_print = running_time;
    }
}

int main(int argc, const char **argv)
{
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(0, "lo");
    }
    else
    {
        ChannelFactory::Instance()->Init(0, argv[1]);
    }

    std::cout << "=== Go2W Keyboard Controller ===" << std::endl;
    std::cout << std::endl;
    std::cout << "Controls:" << std::endl;
    std::cout << "  W/S       : Drive forward/backward" << std::endl;
    std::cout << "  A/D       : Turn left/right" << std::endl;
    std::cout << "  Q/E       : Strafe left/right" << std::endl;
    std::cout << "  R/F       : Raise/lower body" << std::endl;
    std::cout << "  SPACE     : Stand up" << std::endl;
    std::cout << "  C         : Sit down" << std::endl;
    std::cout << "  ESC / X   : Quit" << std::endl;
    std::cout << std::endl;
    std::cout << "Press Enter to start...";
    std::cin.get();

    enableRawMode();

    KeyboardControl controller;
    controller.Init();

    std::cout << "\nKeyboard control active!" << std::endl;

    while (controller.isRunning())
    {
        controller.ProcessKeyboard();
        usleep(10000);  // 10ms
    }

    std::cout << "\nShutting down..." << std::endl;

    return 0;
}
