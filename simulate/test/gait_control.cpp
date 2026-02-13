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

constexpr double MOTOR_KP = 100.0;  // Position gain (stiffness)
                                     // Higher = stiffer joints, faster response
                                     // Too high = vibration/instability

constexpr double MOTOR_KD = 6.0;    // Velocity gain (damping)
                                     // Higher = more damping, less oscillation
                                     // Too high = sluggish response

constexpr double MOTOR_KP_WHEEL = 0;    // Wheels use velocity control
constexpr double MOTOR_KD_WHEEL = 5.0;  // Wheel damping

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
constexpr double WHEEL_TURN_SCALE = 4.0;      // Differential speed for turning
constexpr double WHEEL_KD = 5.0;              // Wheel damping gain

// ---------- STANDUP SEQUENCE ----------
constexpr double STANDUP_DURATION = 2.0;      // Seconds to stand up
constexpr double INITIAL_BODY_HEIGHT = 0.15;  // Starting crouch height

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
// THESE VALUES COME FROM go2w.xml - NEVER CHANGE
const double ABAD_LINK = 0.0955;   // Hip abduction offset (from XML: thigh pos Y relative to hip)
const double HIP_LINK = 0.213;     // Thigh length (from XML: calf pos Z relative to thigh)
const double KNEE_LINK = 0.2264;   // Calf to wheel center (from XML: wheel pos Z relative to calf)

// Hip positions relative to body center (from XML: hip body pos attribute)
// NEVER CHANGE - These define robot geometry in world frame
const Vec3 HIP_POS[4] = {
    Vec3( 0.1934, -0.0465, 0),  // FR(0) - NEVER CHANGE (from XML FR_hip pos)
    Vec3( 0.1934,  0.0465, 0),  // FL(1) - NEVER CHANGE (from XML FL_hip pos)
    Vec3(-0.1934, -0.0465, 0),  // RR(2) - NEVER CHANGE (from XML RR_hip pos)
    Vec3(-0.1934,  0.0465, 0)   // RL(3) - NEVER CHANGE (from XML RL_hip pos)
};

// Side sign for each leg (determines thigh offset direction from XML)
// From XML: left legs(FL,RL) have thigh pos Y=+0.0955, right legs(FR,RR) have Y=-0.0955
// NEVER CHANGE
const int SIDE_SIGN[4] = {-1, 1, -1, 1};  // FR(-), FL(+), RR(-), RL(+)

// ============ Gait Types ============
enum GaitType {
    GAIT_TROT = 0,        // 1: Trot (backward)
    GAIT_WALK = 1,        // 2: Walk (backward)
    GAIT_TROT_FWD = 2,    // 3: Trot (forward)
    GAIT_WALK_FWD = 3,    // 4: Walk (forward)
    GAIT_PRONK = 4,       // 5: Pronk (jump)
    GAIT_BOUND = 5,       // 6: Bound (gallop)
    GAIT_STAND = 6,       // 0: Stand
    GAIT_CUSTOM = 7
};

struct GaitParams {
    double period;        // Gait cycle period (seconds)
    double stanceRatio;   // Fraction of cycle in stance phase
    double bias[4];       // Phase offset for each leg (0-1): FR, FL, RR, RL
    const char* name;
};

// Gait definitions
// {period, stanceRatio, {FR, FL, RR, RL phase biases}, name}
GaitParams GAITS[] = {
    // Trot (backward): diagonal pairs move together
    {0.5,  0.35, {0.0, 0.5, 0.5, 0.0}, "Trot"},
    // Walk (backward): sequential
    {0.60, 0.75, {0.0, 0.25, 0.5, 0.75}, "Walk"},
    // Trot (forward): same phases but inverted step direction
    {0.5,  0.35, {0.0, 0.5, 0.5, 0.0}, "TrotF"},
    // Walk (forward): same phases but inverted step direction  
    {0.60, 0.75, {0.0, 0.25, 0.5, 0.75}, "WalkF"},
    // Pronk: all legs together (jump)
    {0.5,  0.30, {0.0, 0.0, 0.0, 0.0}, "Pronk"},
    // Bound: front pair, then back pair
    {0.45, 0.50, {0.0, 0.0, 0.5, 0.5}, "Bound"},
    // Stand: stationary
    {1.0,  1.0,  {0.0, 0.0, 0.0, 0.0}, "Stand"},
    // Custom: user-defined
    {0.5,  0.5,  {0.0, 0.0, 0.0, 0.0}, "Custom"}
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
// NEVER CHANGE - These match the <joint range="..."> attributes in XML
const double ABAD_MIN = -1.0472;      // NEVER CHANGE (XML: abduction joint range)
const double ABAD_MAX = 1.0472;       // NEVER CHANGE (XML: abduction joint range)
const double FRONT_HIP_MIN = -1.5708; // NEVER CHANGE (XML: front_hip joint range)
const double FRONT_HIP_MAX = 3.4907;  // NEVER CHANGE (XML: front_hip joint range)
const double BACK_HIP_MIN = -0.5236;  // NEVER CHANGE (XML: back_hip joint range)
const double BACK_HIP_MAX = 4.5379;   // NEVER CHANGE (XML: back_hip joint range)
const double KNEE_MIN = -2.7227;      // NEVER CHANGE (XML: knee joint range)
const double KNEE_MAX = -0.83776;     // NEVER CHANGE (XML: knee joint range)

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

        double l1 = ABAD_LINK;
        double l2 = HIP_LINK;
        double l3 = KNEE_LINK;

        // Q0: Hip abduction angle
        // Project foot position onto YZ plane and solve for abduction
        double L_xy = sqrt(py*py + pz*pz);
        if (L_xy < 0.001) L_xy = 0.001;
        
        // Account for abduction link offset
        double y_offset = sideSign * l1;
        double py_eff = py - y_offset;
        
        // Solve for abduction angle
        double c_abad = sqrt(py_eff*py_eff + pz*pz);
        if (c_abad < 0.001) c_abad = 0.001;
        
        // Standard 2R IK in the abducted plane
        q0 = sideSign * atan2(py_eff, -pz);
        q0 = fmax(-1.0472, fmin(1.0472, q0));  // Clamp to joint limits
        
        // Transform to the thigh/knee plane after abduction
        double py_rot = py * cos(q0) - pz * sin(q0);
        double pz_rot = py * sin(q0) + pz * cos(q0);
        
        // Effective Y after abduction (should be close to y_offset)
        double y_eff = py_rot;
        double z_eff = -pz_rot;
        
        // Distance from hip joint to foot in sagittal plane
        double r = sqrt(px*px + z_eff*z_eff);
        if (r < 0.001) r = 0.001;
        
        // Knee angle using law of cosines
        double cos_knee = (l2*l2 + l3*l3 - r*r) / (2*l2*l3);
        cos_knee = fmax(-1.0, fmin(1.0, cos_knee));
        q2 = -(M_PI - acos(cos_knee));  // Knee flexion (negative)
        
        // Hip angle
        double alpha = atan2(px, z_eff);
        double cos_beta = (l2*l2 + r*r - l3*l3) / (2*l2*r);
        cos_beta = fmax(-1.0, fmin(1.0, cos_beta));
        double beta = acos(cos_beta);
        q1 = alpha + beta;
    }

    // Forward kinematics: joint angles to foot position (relative to hip)
    static Vec3 calcFootPos(int legID, double q0, double q1, double q2) {
        int sideSign = SIDE_SIGN[legID];

        double l1 = ABAD_LINK;  // Abduction link (always positive)
        double l2 = HIP_LINK;   // Thigh link
        double l3 = KNEE_LINK;  // Calf link

        double s0 = sin(q0), c0 = cos(q0);
        double s1 = sin(q1), c1 = cos(q1);
        double s2 = sin(q2), c2 = cos(q2);
        
        // Rotation matrix for abduction (around X axis)
        // After abduction, thigh rotates around Y axis
        double s12 = sin(q1 + q2), c12 = cos(q1 + q2);
        
        Vec3 pos;
        // X position: forward/back from thigh and calf
        pos.x = l2 * s1 + l3 * s12;
        
        // Y position: sideways from abduction
        double y_local = l2 * c1 + l3 * c12;
        pos.y = sideSign * l1 + y_local * s0;
        
        // Z position: vertical
        pos.z = -y_local * c0;

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
            // Normalized time with phase offset - FIX: handle negative time properly
            double phaseOffset = gait.period * gait.bias[i];
            double normalT = fmod(passedTime - phaseOffset, gait.period);
            if (normalT < 0) normalT += gait.period;  // Ensure positive
            normalT /= gait.period;

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

    // Public time utility function
    static double getTimeSeconds() {
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
    GaitController() : running(true), vx(0), vy(0), vyaw(0), gaitHeight(DEFAULT_GAIT_HEIGHT), 
                       bodyHeight(DEFAULT_BODY_HEIGHT), standupPhase(0.0), isStandingUp(true) {
        wave = new WaveGenerator();
        startTime = WaveGenerator::getTimeSeconds();

        // Initialize foot positions to standing pose
        // FIX: Y position must account for abduction link offset to have q0=0 (straight legs)
        // When q0=0, foot Y = SIDE_SIGN * ABAD_LINK (from FK: pos.y = sideSign*l1 + ...)
        for (int i = 0; i < 4; i++) {
            // Default standing position relative to hip
            // Y offset ensures legs hang straight down (q0=0) in neutral stance
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

    void setGait(GaitType gait) { 
        GaitType oldGait = wave->currentGait;
        wave->setGait(gait); 
        // Reset swing positions when changing to/from walking gaits
        if (oldGait != gait) {
            for (int i = 0; i < 4; i++) {
                swingStart[i] = standingFeet[i];
                swingEnd[i] = standingFeet[i];
                lastContact[i] = 1;  // Start in stance
            }
        }
    }
    GaitType getGait() { return wave->currentGait; }

    void setBodyHeight(double h) { bodyHeight = fmax(0.15, fmin(0.40, h)); }
    void setGaitHeight(double h) { gaitHeight = fmax(0.03, fmin(0.15, h)); }

private:
    void InitLowCmd();
    void LowStateHandler(const void* message);
    void ControlLoop();
    uint32_t Crc32(uint32_t* ptr, uint32_t len);

    WaveGenerator* wave;

    double vx, vy, vyaw;          // Velocity commands
    double gaitHeight;            // Swing foot lift height
    double bodyHeight;            // Body height above ground
    
    double standupPhase;          // 0.0 to 1.0 standup progress
    bool isStandingUp;            // True during standup sequence
    double startTime;             // Controller start time

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

    // Handle standup sequence
    double currentBodyHeight;
    if (isStandingUp) {
        standupPhase += 0.005;  // Increment at ~500Hz
        if (standupPhase >= 1.0) {
            standupPhase = 1.0;
            isStandingUp = false;
        }
        // Smooth interpolation from crouch to stand
        double smoothPhase = tanh(standupPhase * 3.0);  // Faster rise
        currentBodyHeight = INITIAL_BODY_HEIGHT + (bodyHeight - INITIAL_BODY_HEIGHT) * smoothPhase;
    } else {
        currentBodyHeight = bodyHeight;
    }

    // Get gait phase and contact state
    int contact[4];
    double phase[4];
    wave->calcContactPhase(contact, phase);

    static int debugCounter = 0;
    bool debugPrint = (debugCounter++ % 100 == 0);  // Print every 100 iterations (~1 sec)
    
    if (debugPrint && vx != 0) {
        double fwdMult = 1.0;
        if (wave->currentGait == GAIT_TROT || wave->currentGait == GAIT_WALK) {
            fwdMult = -1.0;
        } else if (wave->currentGait == GAIT_TROT_FWD || wave->currentGait == GAIT_WALK_FWD) {
            fwdMult = 1.0;
        }
        printf("\n=== %s === vx=%.2f fwdMult=%+.1f ===\n", GAITS[wave->currentGait].name, vx, fwdMult);
        printf("Leg:     FR(0)   FL(1)   RR(2)   RL(3)\n");
        printf("Contact: %d       %d       %d       %d\n", contact[0], contact[1], contact[2], contact[3]);
    }

    // Calculate foot positions for each leg
    Vec3 footPos[4];
    double effectiveBodyHeight = bodyHeight;  // Declare before goto
    
    // Check if we should use wheels only (Go2W wheeled mode)
    // Only in STAND gait or very low speeds - walking gaits should use legs
    double v_linear = sqrt(vx*vx + vy*vy);
    bool useWheelsOnly = (wave->currentGait == GAIT_STAND && v_linear < 0.8);

    // Special case: Stand mode or wheels-only driving
    if (useWheelsOnly) {
        for (int leg = 0; leg < 4; leg++) {
            // FIX: Use same Y offset as initialization for straight legs
            footPos[leg] = Vec3(0, SIDE_SIGN[leg] * ABAD_LINK, -currentBodyHeight);
        }
        goto apply_ik;  // Skip gait logic - wheels handle movement
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
        standingFeet[leg].z = -currentBodyHeight;
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
                
                // Forward/backward multiplier: -1 for backward gaits, +1 for forward gaits
                double fwdMult = 1.0;
                if (wave->currentGait == GAIT_TROT || wave->currentGait == GAIT_WALK) {
                    fwdMult = -1.0;  // Original gaits walk backward
                } else if (wave->currentGait == GAIT_TROT_FWD || wave->currentGait == GAIT_WALK_FWD) {
                    fwdMult = 1.0;   // New gaits walk forward
                }

                // Raibert heuristic: step to where we need to be
                double stepX = fwdMult * vx * Tstance / 2 + fwdMult * vx * (1 - phase[leg]) * Tswing;
                double stepY = vy * Tstance / 2 + vy * (1 - phase[leg]) * Tswing;
                
                if (debugPrint && leg == 0) {
                    printf("  stepX=%+.3f (fwdMult=%+.1f * vx=%.2f)\n", stepX, fwdMult, vx);
                }

                // Calculate swing end position
                swingEnd[leg].x = stepX;
                swingEnd[leg].y = SIDE_SIGN[leg] * ABAD_LINK + stepY;
                swingEnd[leg].z = -effectiveBodyHeight;
            }

            // Cycloid trajectory during swing
            double swingX = Trajectory::cycloidXY(swingStart[leg].x, swingEnd[leg].x, phase[leg]);
            double swingZ = Trajectory::cycloidZ(-effectiveBodyHeight, gaitHeight, phase[leg]);
            double liftHeight = swingZ - (-effectiveBodyHeight);

            // Compensation DISABLED - was causing backward walking issues

            footPos[leg].x = swingX;
            footPos[leg].y = Trajectory::cycloidXY(swingStart[leg].y, swingEnd[leg].y, phase[leg]);
            footPos[leg].z = swingZ;
            
            if (debugPrint) {
                const char* names[4] = {"FR", "FL", "RR", "RL"};
                printf("  %s: footPos=(%+.2f, %+.2f, %+.2f) %s\n", 
                       names[leg], footPos[leg].x, footPos[leg].y, footPos[leg].z,
                       contact[leg] ? "STANCE" : "SWING");
            }
        }

        lastContact[leg] = contact[leg];
    }
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
        low_cmd.motor_cmd()[i].q() = 0;  // Position target (not used in vel mode)
        low_cmd.motor_cmd()[i].kp() = MOTOR_KP_WHEEL;
        low_cmd.motor_cmd()[i].kd() = MOTOR_KD_WHEEL;
        low_cmd.motor_cmd()[i].tau() = 0;
    }

    // Wheel direction conventions (matches keyboard_control.cpp)
    // Right wheels (12=FR, 14=RR): wheelSpeed - turnDiff
    // Left wheels (13=FL, 15=RL): wheelSpeed + turnDiff
    double rightSpeed = wheelSpeed - turnDiff;
    double leftSpeed = wheelSpeed + turnDiff;
    
    low_cmd.motor_cmd()[12].dq() = rightSpeed;
    low_cmd.motor_cmd()[14].dq() = rightSpeed;
    low_cmd.motor_cmd()[13].dq() = leftSpeed;
    low_cmd.motor_cmd()[15].dq() = leftSpeed;

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
    std::cout << "  1       : Trot (backward)" << std::endl;
    std::cout << "  2       : Walk (backward)" << std::endl;
    std::cout << "  3       : Trot (forward)" << std::endl;
    std::cout << "  4       : Walk (forward)" << std::endl;
    std::cout << "  5       : Pronk (jump)" << std::endl;
    std::cout << "  6       : Bound (gallop)" << std::endl;
    std::cout << "  0       : Stand" << std::endl;
    std::cout << "  C       : Custom WaveGenerator (enter your own values)" << std::endl;
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

    // Key state tracking for simultaneous input
    bool key_w = false, key_s = false, key_a = false, key_d = false, key_q = false, key_e = false;

    bool running = true;
    while (running) {
        // Process all pending keyboard input
        while (kbhit()) {
            char c = getchar();
            switch (c) {
                // Movement - set key states
                case 'w': case 'W': key_w = true; break;
                case 's': case 'S': key_s = true; break;
                case 'a': case 'A': key_a = true; break;
                case 'd': case 'D': key_d = true; break;
                case 'q': case 'Q': key_q = true; break;
                case 'e': case 'E': key_e = true; break;

                // Gaits
                case '1': controller.setGait(GAIT_TROT); std::cout << "Gait: Trot (backward)" << std::endl; break;
                case '2': controller.setGait(GAIT_WALK); std::cout << "Gait: Walk (backward)" << std::endl; break;
                case '3': controller.setGait(GAIT_TROT_FWD); std::cout << "Gait: Trot (forward)" << std::endl; break;
                case '4': controller.setGait(GAIT_WALK_FWD); std::cout << "Gait: Walk (forward)" << std::endl; break;
                case '5': controller.setGait(GAIT_PRONK); std::cout << "Gait: Pronk" << std::endl; break;
                case '6': controller.setGait(GAIT_BOUND); std::cout << "Gait: Bound" << std::endl; break;
                case '0': controller.setGait(GAIT_STAND); std::cout << "Gait: Stand" << std::endl; break;
                case 'c': case 'C':
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

        // Compute velocities from accumulated key states
        // Keys accumulate during the while(kbhit()) loop above
        double target_vx = 0, target_vy = 0, target_vyaw = 0;
        if (key_w) target_vx += KEY_VX;
        if (key_s) target_vx -= KEY_VX;
        if (key_a) target_vyaw += KEY_VYAW;
        if (key_d) target_vyaw -= KEY_VYAW;
        if (key_q) target_vy += KEY_VY;
        if (key_e) target_vy -= KEY_VY;

        // INSTANT full speed when key pressed, decay when released
        // Use direct assignment for max responsiveness
        if (target_vx != 0) vx = target_vx;
        else vx *= decay;
        
        if (target_vy != 0) vy = target_vy;
        else vy *= decay;
        
        if (target_vyaw != 0) vyaw = target_vyaw;
        else vyaw *= decay;

        // Apply threshold
        if (fabs(vx) < 0.001) vx = 0;
        if (fabs(vy) < 0.001) vy = 0;
        if (fabs(vyaw) < 0.001) vyaw = 0;

        // Reset key states - they will be set again next frame if keys are still held
        // Terminal auto-repeat sends repeated keypresses when held
        bool any_key_this_frame = key_w || key_s || key_a || key_d || key_q || key_e;
        key_w = key_s = key_a = key_d = key_q = key_e = false;

        controller.setVelocity(vx, vy, vyaw);

        usleep(10000);  // 100 Hz main loop
    }

    controller.Stop();
    std::cout << "Exiting..." << std::endl;

    return 0;
}
