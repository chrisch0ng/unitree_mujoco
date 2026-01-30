// Gait Controller for Go2W in MuJoCo
// Implements trot, walk, pronk, bound gaits with inverse kinematics
// Based on Unitree's gait generation system

// ============================================================================
// ====================== TUNABLE PARAMETERS ==================================
// ============================================================================
// All adjustable parameters are defined here for easy tuning.
// Modify these values to change robot behavior.

// ---------- BODY & STANCE PARAMETERS ----------
// These control the robot's default posture

constexpr double DEFAULT_BODY_HEIGHT = 0.30;  // meters - How high the body is from ground
                                               // Range: 0.15 to 0.40
                                               // Lower = more stable, Higher = longer reach

constexpr double DEFAULT_GAIT_HEIGHT = 0.08;  // meters - How high feet lift during swing
                                               // Range: 0.03 to 0.15
                                               // Higher = clears obstacles better, but slower

// ---------- MOTOR CONTROL GAINS ----------
// PD controller gains for joint position control

constexpr double MOTOR_KP = 80.0;   // Position gain (stiffness)
                                     // Higher = stiffer joints, faster response
                                     // Too high = vibration/instability

constexpr double MOTOR_KD = 5.0;    // Velocity gain (damping)
                                     // Higher = more damping, less oscillation
                                     // Too high = sluggish response

// ---------- VELOCITY LIMITS ----------
// Maximum speeds the robot can achieve

constexpr double MAX_VX = 0.5;       // m/s - Max forward/backward speed
constexpr double MAX_VY = 0.3;       // m/s - Max strafe (sideways) speed
constexpr double MAX_VYAW = 0.5;     // rad/s - Max turning speed

// ---------- KEYBOARD VELOCITY COMMANDS ----------
// Speed when pressing movement keys

constexpr double KEY_VX = 0.3;       // m/s - Forward/backward (W/S keys)
constexpr double KEY_VY = 0.2;       // m/s - Strafe left/right (Q/E keys)
constexpr double KEY_VYAW = 0.3;     // rad/s - Turn left/right (A/D keys)
constexpr double VELOCITY_DECAY = 0.95;  // How fast velocity decays when key released
                                          // 0.9 = fast stop, 0.99 = coast longer

// ---------- SWING COMPENSATION ----------
// Compensates for back leg joint limits during foot lift
// Back legs have restricted hip range, need more backward offset when lifting

constexpr double COMP_TROT_WALK_BACK = 1.5;   // Back leg compensation for trot/walk
                                               // Higher = feet swing more backward

constexpr double COMP_PRONK_BACK = 1.2;       // Back leg compensation for pronk/bound
constexpr double COMP_PRONK_FRONT = 0.5;      // Front leg compensation for pronk/bound

// ---------- PRONK (JUMP) PARAMETERS ----------
// Controls the crouch-push-flight cycle for jumping

constexpr double PRONK_CROUCH_DEPTH = 0.06;   // meters - How much to crouch down before jump
                                               // Deeper crouch = more power, but slower

constexpr double PRONK_EXTEND_HEIGHT = 0.04;  // meters - How much to extend above nominal
                                               // Higher = more explosive push

constexpr double PRONK_TUCK_HEIGHT = 0.04;    // meters - How much to tuck feet during flight
                                               // Higher = feet pulled up more in air

// ---------- WHEEL CONTROL ----------
// For Go2W wheeled variant

constexpr double WHEEL_SPEED_SCALE = 8.0;     // Multiplier from vx to wheel speed
constexpr double WHEEL_TURN_SCALE = 3.0;      // Differential speed for turning
constexpr double WHEEL_KD = 5.0;              // Wheel damping gain

// ---------- GAIT TIMING PARAMETERS ----------
// Defined in GAITS[] array below. Format:
//   {period, stanceRatio, {FR, FL, RR, RL phase biases}, "name"}
//
// period:      Total gait cycle time in seconds (0.3-1.0 typical)
//              Shorter = faster cadence, Longer = slower, more deliberate
//
// stanceRatio: Fraction of cycle foot is on ground (0.1-0.95)
//              Lower = more time in air (bouncy), Higher = more time on ground (stable)
//              For jumping (pronk): use 0.2-0.4 for more air time
//
// phase bias:  When each leg starts its cycle (0.0-1.0)
//              0.0 = starts at beginning, 0.5 = starts halfway through
//              FR=Front Right, FL=Front Left, RR=Rear Right, RL=Rear Left
//
//              Common patterns:
//              - Trot: diagonal pairs together {0, 0.5, 0.5, 0}
//              - Walk: sequential {0, 0.25, 0.5, 0.75}
//              - Pronk: all together {0, 0, 0, 0}
//              - Bound: front/back pairs {0, 0, 0.5, 0.5}

// ============================================================================
// ====================== END TUNABLE PARAMETERS ==============================
// ============================================================================

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string>
#include <atomic>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
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

// ============ Math Types ============
struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
    Vec3 operator+(const Vec3& v) const { return Vec3(x+v.x, y+v.y, z+v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x-v.x, y-v.y, z-v.z); }
    Vec3 operator*(double s) const { return Vec3(x*s, y*s, z*s); }
};

// ============ Go2 Robot Parameters ============
const double ABAD_LINK = 0.0955;   // Hip abduction link length (adjusted for Go2W)
const double HIP_LINK = 0.213;     // Thigh link length
const double KNEE_LINK = 0.2264;   // Calf to wheel center (Go2W)

// Hip positions relative to body center (x, y, z)
const Vec3 HIP_POS[4] = {
    Vec3( 0.1934, -0.0465, 0),  // FR (leg 0)
    Vec3( 0.1934,  0.0465, 0),  // FL (leg 1)
    Vec3(-0.1934, -0.0465, 0),  // RR (leg 2)
    Vec3(-0.1934,  0.0465, 0)   // RL (leg 3)
};

// Side sign for each leg (determines hip abduction direction)
const int SIDE_SIGN[4] = {-1, 1, -1, 1};  // FR, FL, RR, RL

// ============ Gait Types ============
enum GaitType {
    GAIT_TROT = 0,
    GAIT_WALK = 1,
    GAIT_PRONK = 2,
    GAIT_BOUND = 3,
    GAIT_STAND = 4,
    GAIT_CUSTOM = 5
};

struct GaitParams {
    double period;        // Gait cycle period (seconds)
    double stanceRatio;   // Fraction of cycle in stance phase
    double bias[4];       // Phase offset for each leg (0-1): FR, FL, RR, RL
    const char* name;
};

// Gait definitions (from mako_ctrl.cpp)
// {period, stanceRatio, {FR, FL, RR, RL phase biases}, name}
GaitParams GAITS[] = {
    {0.5,  0.35, {0.0, 0.5, 0.5, 0.0}, "Trot"},     // Running Trot (mako default)
    {0.40, 0.60, {0.0, 0.5, 0.50, 0.0}, "Walk"},   // Walk (from mako) - walking trot 
    {0.5,  0.30, {0.0, 0.0, 0.0, 0.0}, "Pronk"},    // Pronk (from mako)
    {0.45, 0.50, {0.0, 0.0, 0.5, 0.5}, "Bound"},    // Bound (from mako)
    {1.0,  1.0,  {0.0, 0.0, 0.0, 0.0}, "Stand"},    // All stance (no swing)
    {0.5,  0.5,  {0.0, 0.0, 0.0, 0.0}, "Custom"}    // User-defined
};

// Function to set custom gait parameters
void setCustomGait(double period, double stanceRatio, double biasFR, double biasFL, double biasRR, double biasRL) {
    GAITS[GAIT_CUSTOM].period = period;
    GAITS[GAIT_CUSTOM].stanceRatio = fmax(0.1, fmin(0.95, stanceRatio));
    GAITS[GAIT_CUSTOM].bias[0] = fmax(0.0, fmin(1.0, biasFR));
    GAITS[GAIT_CUSTOM].bias[1] = fmax(0.0, fmin(1.0, biasFL));
    GAITS[GAIT_CUSTOM].bias[2] = fmax(0.0, fmin(1.0, biasRR));
    GAITS[GAIT_CUSTOM].bias[3] = fmax(0.0, fmin(1.0, biasRL));
}

// ============ Joint Limits (from go2w.xml) ============
const double ABAD_MIN = -1.0472;
const double ABAD_MAX = 1.0472;
const double FRONT_HIP_MIN = -1.5708;
const double FRONT_HIP_MAX = 3.4907;
const double BACK_HIP_MIN = -0.5236;
const double BACK_HIP_MAX = 4.5379;
const double KNEE_MIN = -2.7227;
const double KNEE_MAX = -0.83776;

// ============ Inverse Kinematics ============
class LegIK {
public:
    // Clamp joint angles to valid limits
    static void clampJoints(int legID, double& q0, double& q1, double& q2) {
        q0 = fmax(ABAD_MIN, fmin(ABAD_MAX, q0));

        // Back legs (RR=2, RL=3) have different hip limits
        if (legID >= 2) {
            q1 = fmax(BACK_HIP_MIN, fmin(BACK_HIP_MAX, q1));
        } else {
            q1 = fmax(FRONT_HIP_MIN, fmin(FRONT_HIP_MAX, q1));
        }

        q2 = fmax(KNEE_MIN, fmin(KNEE_MAX, q2));
    }

    // Calculate joint angles from foot position relative to hip
    static void calcQ(int legID, const Vec3& footPos, double& q0, double& q1, double& q2) {
        int sideSign = SIDE_SIGN[legID];

        double px = footPos.x;
        double py = footPos.y;
        double pz = footPos.z;

        double l1 = ABAD_LINK * sideSign;
        double l2 = HIP_LINK;
        double l3 = KNEE_LINK;

        // Distance calculations
        double c = sqrt(px*px + py*py + pz*pz);  // Total distance
        double a = fabs(l1);
        double b = sqrt(c*c - a*a);  // Distance from shoulder to foot
        if (b < 0.001) b = 0.001;  // Prevent divide by zero

        // Q1: Hip abduction angle
        double L = sqrt(py*py + pz*pz - l1*l1);
        if (L < 0.001) L = 0.001;
        q0 = atan2(pz*l1 + py*L, py*l1 - pz*L);

        // Q3: Knee angle
        double temp = (l2*l2 + l3*l3 - b*b) / (2*l2*l3);
        temp = fmax(-1.0, fmin(1.0, temp));  // Clamp to [-1, 1]
        q2 = -(M_PI - acos(temp));  // Knee flexion (negative)

        // Q2: Hip angle
        double a1 = py*sin(q0) - pz*cos(q0);
        double a2 = px;
        double m1 = -l3*sin(q2);
        double m2 = -l2 - l3*cos(q2);
        q1 = atan2(m1*a1 + m2*a2, m1*a2 - m2*a1);
    }

    // Forward kinematics: joint angles to foot position (relative to hip)
    static Vec3 calcFootPos(int legID, double q0, double q1, double q2) {
        int sideSign = SIDE_SIGN[legID];

        double l1 = ABAD_LINK * sideSign;
        double l2 = -HIP_LINK;
        double l3 = -KNEE_LINK;

        double s1 = sin(q0), c1 = cos(q0);
        double s2 = sin(q1), c2 = cos(q1);
        double s3 = sin(q2), c3 = cos(q2);
        double c23 = c2*c3 - s2*s3;
        double s23 = s2*c3 + c2*s3;

        Vec3 pos;
        pos.x = l3*s23 + l2*s2;
        pos.y = -l3*s1*c23 + l1*c1 - l2*c2*s1;
        pos.z = l3*c1*c23 + l1*s1 + l2*c1*c2;

        return pos;
    }
};

// ============ Wave Generator ============
class WaveGenerator {
public:
    GaitType currentGait;
    double startTime;

    WaveGenerator() : currentGait(GAIT_STAND), startTime(0) {
        startTime = getTimeSeconds();
    }

    void setGait(GaitType gait) {
        if (gait != currentGait) {
            currentGait = gait;
            startTime = getTimeSeconds();  // Reset phase
        }
    }

    // Calculate contact state and phase for each leg
    void calcContactPhase(int contact[4], double phase[4]) {
        const GaitParams& gait = GAITS[currentGait];
        double passedTime = getTimeSeconds() - startTime;

        for (int i = 0; i < 4; i++) {
            // Normalized time with phase offset
            double normalT = fmod(passedTime + gait.period - gait.period * gait.bias[i], gait.period) / gait.period;

            if (normalT < gait.stanceRatio) {
                // Stance phase: foot on ground
                contact[i] = 1;
                phase[i] = normalT / gait.stanceRatio;
            } else {
                // Swing phase: foot in air
                contact[i] = 0;
                phase[i] = (normalT - gait.stanceRatio) / (1.0 - gait.stanceRatio);
            }
        }
    }

    double getTswing() { return GAITS[currentGait].period * (1.0 - GAITS[currentGait].stanceRatio); }
    double getTstance() { return GAITS[currentGait].period * GAITS[currentGait].stanceRatio; }

private:
    double getTimeSeconds() {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec + tv.tv_usec * 1e-6;
    }
};

// ============ Cycloid Trajectory ============
class Trajectory {
public:
    // Cycloid X/Y position
    static double cycloidXY(double start, double end, double phase) {
        double phasePI = 2 * M_PI * phase;
        return (end - start) * (phasePI - sin(phasePI)) / (2 * M_PI) + start;
    }

    // Cycloid Z position (foot height)
    static double cycloidZ(double start, double height, double phase) {
        double phasePI = 2 * M_PI * phase;
        return height * (1 - cos(phasePI)) / 2 + start;
    }
};

// ============ Gait Controller ============
class GaitController {
public:
    GaitController() : running(true), vx(0), vy(0), vyaw(0), gaitHeight(DEFAULT_GAIT_HEIGHT), bodyHeight(DEFAULT_BODY_HEIGHT) {
        wave = new WaveGenerator();

        // Initialize foot positions to standing pose
        for (int i = 0; i < 4; i++) {
            // Default standing position relative to hip
            standingFeet[i] = Vec3(0, SIDE_SIGN[i] * ABAD_LINK, -bodyHeight);
            swingStart[i] = standingFeet[i];
            swingEnd[i] = standingFeet[i];
        }
    }

    ~GaitController() { delete wave; }

    void Init();
    void Start();
    void Stop() { running = false; }

    void setVelocity(double _vx, double _vy, double _vyaw) {
        vx = fmax(-MAX_VX, fmin(MAX_VX, _vx));
        vy = fmax(-MAX_VY, fmin(MAX_VY, _vy));
        vyaw = fmax(-MAX_VYAW, fmin(MAX_VYAW, _vyaw));
    }

    void setGait(GaitType gait) { wave->setGait(gait); }
    GaitType getGait() { return wave->currentGait; }

    void setBodyHeight(double h) { bodyHeight = fmax(0.15, fmin(0.40, h)); }
    void setGaitHeight(double h) { gaitHeight = fmax(0.03, fmin(0.15, h)); }

private:
    void InitLowCmd();
    void LowStateHandler(const void* message);
    void ControlLoop();
    uint32_t Crc32(uint32_t* ptr, uint32_t len);

    WaveGenerator* wave;

    double vx, vy, vyaw;         // Velocity commands
    double gaitHeight;            // Swing foot lift height
    double bodyHeight;            // Body height above ground

    Vec3 standingFeet[4];         // Default standing foot positions
    Vec3 swingStart[4];           // Start position of swing
    Vec3 swingEnd[4];             // End position of swing
    int lastContact[4] = {1,1,1,1};

    std::atomic<bool> running;

    double Kp = MOTOR_KP;
    double Kd = MOTOR_KD;

    unitree_go::msg::dds_::LowCmd_ low_cmd{};
    unitree_go::msg::dds_::LowState_ low_state{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> cmd_publisher;
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> state_subscriber;
    ThreadPtr control_thread;
};

uint32_t GaitController::Crc32(uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0, data = 0;
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
            if (data & xbit) CRC32 ^= polynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

void GaitController::Init() {
    InitLowCmd();

    cmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    cmd_publisher->InitChannel();

    state_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    state_subscriber->InitChannel(std::bind(&GaitController::LowStateHandler, this, std::placeholders::_1), 1);
}

void GaitController::InitLowCmd() {
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

void GaitController::LowStateHandler(const void* message) {
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void GaitController::Start() {
    control_thread = CreateRecurrentThreadEx("gait_control", UT_CPU_ID_NONE, 2000, &GaitController::ControlLoop, this);
}

void GaitController::ControlLoop() {
    if (!running) return;

    // static int debugCounter = 0;
    // bool debugPrint = (debugCounter++ % 250 == 0);  // Print every 250 iterations (~0.5 sec)

    // Get gait phase and contact state
    int contact[4];
    double phase[4];
    wave->calcContactPhase(contact, phase);

    // if (debugPrint) {
    //     printf("\n=== DEBUG: %s Gait ===\n", GAITS[wave->currentGait].name);
    //     printf("Leg:     FR(0)   FL(1)   RR(2)   RL(3)\n");
    //     printf("Contact: %d       %d       %d       %d\n", contact[0], contact[1], contact[2], contact[3]);
    //     printf("Phase:   %.2f    %.2f    %.2f    %.2f\n", phase[0], phase[1], phase[2], phase[3]);
    // }

    // Calculate foot positions for each leg
    Vec3 footPos[4];
    double effectiveBodyHeight = bodyHeight;  // Declare before goto

    // Special case: Stand mode - just hold standing position
    if (wave->currentGait == GAIT_STAND) {
        for (int leg = 0; leg < 4; leg++) {
            footPos[leg] = Vec3(0, SIDE_SIGN[leg] * ABAD_LINK, -bodyHeight);
        }
        goto apply_ik;  // Skip gait logic
    }

    // For pronk, modulate effective body height to create crouch-push-flight cycle
    if (wave->currentGait == GAIT_PRONK) {
        // All legs have same phase in pronk (all biases are 0)
        // Use leg 0's phase as reference
        double stancePhase = phase[0];
        bool inStance = (contact[0] == 1);

        if (inStance) {
            // Stance phase: crouch then extend
            // stancePhase goes 0->1 during stance
            // 0-0.5: crouch (lower body)
            // 0.5-1.0: extend (raise body to push off)
            double crouchDepth = PRONK_CROUCH_DEPTH;
            double extendHeight = PRONK_EXTEND_HEIGHT;

            if (stancePhase < 0.5) {
                // Crouch phase: lower body
                double crouchProgress = stancePhase / 0.5;  // 0->1
                effectiveBodyHeight = bodyHeight + crouchDepth * (1 - cos(M_PI * crouchProgress)) / 2;
            } else {
                // Extend phase: push up
                double extendProgress = (stancePhase - 0.5) / 0.5;  // 0->1
                // Go from crouched (+crouchDepth) to extended (-extendHeight)
                double heightOffset = crouchDepth - (crouchDepth + extendHeight) * (1 - cos(M_PI * extendProgress)) / 2;
                effectiveBodyHeight = bodyHeight + heightOffset;
            }
        } else {
            // Swing/flight phase: tuck feet (effectively shorter body height command)
            // This makes legs retract during flight
            effectiveBodyHeight = bodyHeight - PRONK_TUCK_HEIGHT;
        }
    }

    // Initialize foot positions to standing pose
    for (int leg = 0; leg < 4; leg++) {
        standingFeet[leg].z = -effectiveBodyHeight;
        footPos[leg] = standingFeet[leg];
    }

    for (int leg = 0; leg < 4; leg++) {
        if (contact[leg] == 1) {
            // Stance phase: foot stays on ground
            if (lastContact[leg] == 0) {
                // Just landed - use the end of swing as new stance position
                swingStart[leg] = swingEnd[leg];
                // printf("Leg %d LANDED (swing->stance)\n", leg);
            }
            // Keep foot at stance position (with effective height for pronk)
            footPos[leg] = swingStart[leg];
            footPos[leg].z = -effectiveBodyHeight;  // Apply height modulation
        } else {
            // Swing phase: foot moves to next position
            if (lastContact[leg] == 1) {
                // Just lifted off - record current stance position as swing start
                // swingStart stays at its current value (set from last landing)

                // Calculate next footstep location based on velocity
                double Tswing = wave->getTswing();
                double Tstance = wave->getTstance();

                // Raibert heuristic: step to where we need to be
                double stepX = vx * Tstance / 2 + vx * (1 - phase[leg]) * Tswing;
                double stepY = vy * Tstance / 2 + vy * (1 - phase[leg]) * Tswing;

                // Calculate swing end position
                swingEnd[leg].x = stepX;
                swingEnd[leg].y = SIDE_SIGN[leg] * ABAD_LINK + stepY;
                swingEnd[leg].z = -effectiveBodyHeight;
            }

            // Cycloid trajectory during swing
            double swingX = Trajectory::cycloidXY(swingStart[leg].x, swingEnd[leg].x, phase[leg]);
            double swingZ = Trajectory::cycloidZ(-effectiveBodyHeight, gaitHeight, phase[leg]);
            double liftHeight = swingZ - (-effectiveBodyHeight);

            // Compensation for joint limits during swing
            // Back legs have restricted hip range - need backward offset to lift properly
            GaitType currentGait = wave->currentGait;

            if (fabs(vx) > 0.05) {
                // Moving: directional compensation
                double compensationDir = (vx > 0) ? -1.0 : 1.0;

                if (currentGait == GAIT_TROT || currentGait == GAIT_WALK || currentGait == GAIT_BOUND) {
                    if (leg >= 2) {
                        swingX += compensationDir * liftHeight * COMP_TROT_WALK_BACK;
                    }
                } else if (currentGait == GAIT_PRONK) {
                    if (leg >= 2) {
                        swingX += compensationDir * liftHeight * COMP_PRONK_BACK;
                    } else {
                        swingX += compensationDir * liftHeight * COMP_PRONK_FRONT;
                    }
                }
            } else {
                // Standing still: backward offset for back legs to allow lifting
                if (leg >= 2) {
                    swingX -= liftHeight * 1.0;
                }
            }

            footPos[leg].x = swingX;
            footPos[leg].y = Trajectory::cycloidXY(swingStart[leg].y, swingEnd[leg].y, phase[leg]);
            footPos[leg].z = swingZ;
        }

        lastContact[leg] = contact[leg];
    }

    // if (debugPrint) {
    //     printf("FootPos (x,y,z):\n");
    //     for (int i = 0; i < 4; i++) {
    //         printf("  Leg%d: (%.3f, %.3f, %.3f)\n", i, footPos[i].x, footPos[i].y, footPos[i].z);
    //     }
    // }

apply_ik:
    // Convert foot positions to joint angles using inverse kinematics
    // Motor mapping: FR(0-2), FL(3-5), RR(6-8), RL(9-11), Wheels(12-15)
    int legToMotor[4] = {0, 3, 6, 9};  // Starting motor index for each leg

    // double allQ[4][3];  // Store joint angles for debug

    for (int leg = 0; leg < 4; leg++) {
        double q0, q1, q2;
        LegIK::calcQ(leg, footPos[leg], q0, q1, q2);
        LegIK::clampJoints(leg, q0, q1, q2);
        // allQ[leg][0] = q0; allQ[leg][1] = q1; allQ[leg][2] = q2;

        int baseIdx = legToMotor[leg];

        low_cmd.motor_cmd()[baseIdx + 0].q() = q0;  // Hip
        low_cmd.motor_cmd()[baseIdx + 0].dq() = 0;
        low_cmd.motor_cmd()[baseIdx + 0].kp() = Kp;
        low_cmd.motor_cmd()[baseIdx + 0].kd() = Kd;
        low_cmd.motor_cmd()[baseIdx + 0].tau() = 0;

        low_cmd.motor_cmd()[baseIdx + 1].q() = q1;  // Thigh
        low_cmd.motor_cmd()[baseIdx + 1].dq() = 0;
        low_cmd.motor_cmd()[baseIdx + 1].kp() = Kp;
        low_cmd.motor_cmd()[baseIdx + 1].kd() = Kd;
        low_cmd.motor_cmd()[baseIdx + 1].tau() = 0;

        low_cmd.motor_cmd()[baseIdx + 2].q() = q2;  // Calf
        low_cmd.motor_cmd()[baseIdx + 2].dq() = 0;
        low_cmd.motor_cmd()[baseIdx + 2].kp() = Kp;
        low_cmd.motor_cmd()[baseIdx + 2].kd() = Kd;
        low_cmd.motor_cmd()[baseIdx + 2].tau() = 0;
    }

    // if (debugPrint) {
    //     printf("Joint angles (q0, q1, q2) in rad:\n");
    //     // Joint limits from MuJoCo model:
    //     // abduction: -1.0472 to 1.0472
    //     // front_hip: -1.5708 to 3.4907
    //     // back_hip:  -0.5236 to 4.5379
    //     // knee:      -2.7227 to -0.83776
    //     for (int i = 0; i < 4; i++) {
    //         const char* warn = "";
    //         double hipMin = (i < 2) ? -1.5708 : -0.5236;  // front vs back
    //         double hipMax = (i < 2) ? 3.4907 : 4.5379;
    //         if (allQ[i][1] < hipMin || allQ[i][1] > hipMax) warn = " [HIP OUT OF RANGE!]";
    //         if (allQ[i][2] < -2.7227 || allQ[i][2] > -0.83776) warn = " [KNEE OUT OF RANGE!]";
    //         printf("  Leg%d: (%.3f, %.3f, %.3f) -> motors %d-%d%s\n",
    //                i, allQ[i][0], allQ[i][1], allQ[i][2], legToMotor[i], legToMotor[i]+2, warn);
    //     }
    //     printf("===========================\n");
    //     fflush(stdout);
    // }

    // Wheel control for Go2W (motors 12-15)
    // Use wheels for forward/backward motion when standing or slow walk
    double wheelSpeed = vx * WHEEL_SPEED_SCALE;
    double turnDiff = vyaw * WHEEL_TURN_SCALE;

    for (int i = 12; i < 16; i++) {
        low_cmd.motor_cmd()[i].q() = 0;
        low_cmd.motor_cmd()[i].kp() = 0;
        low_cmd.motor_cmd()[i].kd() = WHEEL_KD;
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    // Right wheels (12, 14)
    low_cmd.motor_cmd()[12].dq() = wheelSpeed - turnDiff;
    low_cmd.motor_cmd()[14].dq() = wheelSpeed - turnDiff;
    // Left wheels (13, 15)
    low_cmd.motor_cmd()[13].dq() = wheelSpeed + turnDiff;
    low_cmd.motor_cmd()[15].dq() = wheelSpeed + turnDiff;

    // Publish command
    low_cmd.crc() = Crc32((uint32_t*)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    cmd_publisher->Write(low_cmd);
}

// ============ Keyboard Input ============
int kbhit() {
    struct termios oldt, newt;
    int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

void printHelp() {
    std::cout << "\n=== Gait Controller for Go2W ===" << std::endl;
    std::cout << "\nMovement:" << std::endl;
    std::cout << "  W/S     : Forward/Backward" << std::endl;
    std::cout << "  A/D     : Turn left/right" << std::endl;
    std::cout << "  Q/E     : Strafe left/right" << std::endl;
    std::cout << "\nGaits:" << std::endl;
    std::cout << "  1       : Trot (diagonal)" << std::endl;
    std::cout << "  2       : Walk (sequential)" << std::endl;
    std::cout << "  3       : Pronk (jump)" << std::endl;
    std::cout << "  4       : Bound (gallop)" << std::endl;
    std::cout << "  5       : Stand" << std::endl;
    std::cout << "  0/C     : Custom WaveGenerator (enter your own values)" << std::endl;
    std::cout << "  P       : Print current gait parameters" << std::endl;
    std::cout << "\nHeight:" << std::endl;
    std::cout << "  R/F     : Body height up/down" << std::endl;
    std::cout << "  T/G     : Gait height up/down" << std::endl;
    std::cout << "\n  H       : Show this help" << std::endl;
    std::cout << "  ESC/X   : Quit" << std::endl;
    std::cout << std::endl;
}

void printGaitParams(GaitType gait) {
    const GaitParams& g = GAITS[gait];
    std::cout << "\n=== Current Gait: " << g.name << " ===" << std::endl;
    std::cout << "WaveGenerator(" << g.period << ", " << g.stanceRatio
              << ", Vec4(" << g.bias[0] << ", " << g.bias[1] << ", "
              << g.bias[2] << ", " << g.bias[3] << "))" << std::endl;
    std::cout << "  Period:       " << g.period << " seconds" << std::endl;
    std::cout << "  Stance Ratio: " << g.stanceRatio << " (0-1)" << std::endl;
    std::cout << "  Phase Bias:   FR=" << g.bias[0] << ", FL=" << g.bias[1]
              << ", RR=" << g.bias[2] << ", RL=" << g.bias[3] << std::endl;
    std::cout << std::endl;
}

// Restore terminal to normal mode for input
void enableLineInput() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag |= (ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

// Disable line buffering for real-time key detection
void disableLineInput() {
    struct termios t;
    tcgetattr(STDIN_FILENO, &t);
    t.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &t);
}

bool inputCustomGait() {
    enableLineInput();  // Enable normal terminal input

    std::cout << "\n=== Custom WaveGenerator ===" << std::endl;
    std::cout << "Enter gait parameters (like mako_ctrl.cpp WaveGenerator):" << std::endl;
    std::cout << "  WaveGenerator(period, stancePhaseRatio, Vec4(FR, FL, RR, RL))" << std::endl;
    std::cout << "\nPreset examples:" << std::endl;
    std::cout << "  Trot:  0.5, 0.35, 0, 0.5, 0.5, 0" << std::endl;
    std::cout << "  Walk:  0.5, 0.70, 0, 0.5, 0.6, 0.25" << std::endl;
    std::cout << "  Pronk: 0.4, 0.70, 0, 0, 0, 0" << std::endl;
    std::cout << "  Bound: 0.45, 0.5, 0, 0, 0.5, 0.5" << std::endl;
    std::cout << "\nEnter period (seconds, e.g. 0.5): ";

    double period, stanceRatio, biasFR, biasFL, biasRR, biasRL;

    if (!(std::cin >> period)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input. Cancelled." << std::endl;
        disableLineInput();
        return false;
    }

    std::cout << "Enter stance ratio (0.1-0.95, e.g. 0.5): ";
    if (!(std::cin >> stanceRatio)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input. Cancelled." << std::endl;
        disableLineInput();
        return false;
    }

    std::cout << "Enter phase bias FR (0-1, e.g. 0): ";
    if (!(std::cin >> biasFR)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input. Cancelled." << std::endl;
        disableLineInput();
        return false;
    }

    std::cout << "Enter phase bias FL (0-1, e.g. 0.5): ";
    if (!(std::cin >> biasFL)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input. Cancelled." << std::endl;
        disableLineInput();
        return false;
    }

    std::cout << "Enter phase bias RR (0-1, e.g. 0.5): ";
    if (!(std::cin >> biasRR)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input. Cancelled." << std::endl;
        disableLineInput();
        return false;
    }

    std::cout << "Enter phase bias RL (0-1, e.g. 0): ";
    if (!(std::cin >> biasRL)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Invalid input. Cancelled." << std::endl;
        disableLineInput();
        return false;
    }

    std::cin.ignore(10000, '\n');  // Clear remaining input

    // Apply custom gait
    setCustomGait(period, stanceRatio, biasFR, biasFL, biasRR, biasRL);

    std::cout << "\nCustom gait set!" << std::endl;
    printGaitParams(GAIT_CUSTOM);

    disableLineInput();  // Return to real-time key detection
    return true;
}

int main(int argc, char** argv) {
    std::cout << "Gait Controller for Go2W (MuJoCo)" << std::endl;
    std::cout << "=================================" << std::endl;

    ChannelFactory::Instance()->Init(0, "lo");

    GaitController controller;
    controller.Init();
    controller.Start();

    printHelp();

    double vx = 0, vy = 0, vyaw = 0;
    double bodyHeight = DEFAULT_BODY_HEIGHT;
    double gaitHeight = DEFAULT_GAIT_HEIGHT;
    const double decay = VELOCITY_DECAY;

    bool running = true;
    while (running) {
        // Process keyboard
        while (kbhit()) {
            char c = getchar();
            switch (c) {
                // Movement
                case 'w': case 'W': vx = KEY_VX; break;
                case 's': case 'S': vx = -KEY_VX; break;
                case 'a': case 'A': vyaw = KEY_VYAW; break;
                case 'd': case 'D': vyaw = -KEY_VYAW; break;
                case 'q': case 'Q': vy = KEY_VY; break;
                case 'e': case 'E': vy = -KEY_VY; break;

                // Gaits
                case '1': controller.setGait(GAIT_TROT); std::cout << "Gait: Trot" << std::endl; break;
                case '2': controller.setGait(GAIT_WALK); std::cout << "Gait: Walk" << std::endl; break;
                case '3': controller.setGait(GAIT_PRONK); std::cout << "Gait: Pronk" << std::endl; break;
                case '4': controller.setGait(GAIT_BOUND); std::cout << "Gait: Bound" << std::endl; break;
                case '5': controller.setGait(GAIT_STAND); std::cout << "Gait: Stand" << std::endl; break;
                case '0': case 'c': case 'C':
                    if (inputCustomGait()) {
                        controller.setGait(GAIT_CUSTOM);
                        std::cout << "Gait: Custom" << std::endl;
                    }
                    break;
                case 'p': case 'P':
                    printGaitParams(controller.getGait());
                    break;

                // Height
                case 'r': case 'R':
                    bodyHeight += 0.02;
                    controller.setBodyHeight(bodyHeight);
                    std::cout << "Body height: " << bodyHeight << std::endl;
                    break;
                case 'f': case 'F':
                    bodyHeight -= 0.02;
                    controller.setBodyHeight(bodyHeight);
                    std::cout << "Body height: " << bodyHeight << std::endl;
                    break;
                case 't': case 'T':
                    gaitHeight += 0.01;
                    controller.setGaitHeight(gaitHeight);
                    std::cout << "Gait height: " << gaitHeight << std::endl;
                    break;
                case 'g': case 'G':
                    gaitHeight -= 0.01;
                    controller.setGaitHeight(gaitHeight);
                    std::cout << "Gait height: " << gaitHeight << std::endl;
                    break;

                // Quit
                case 27: case 'x': case 'X':
                    running = false;
                    break;

                case 'h': case 'H':
                    printHelp();
                    break;
            }
        }

        // Decay velocities
        vx *= decay;
        vy *= decay;
        vyaw *= decay;

        // Apply small threshold
        if (fabs(vx) < 0.01) vx = 0;
        if (fabs(vy) < 0.01) vy = 0;
        if (fabs(vyaw) < 0.01) vyaw = 0;

        controller.setVelocity(vx, vy, vyaw);

        usleep(10000);  // 100 Hz main loop
    }

    controller.Stop();
    std::cout << "Exiting..." << std::endl;

    return 0;
}
