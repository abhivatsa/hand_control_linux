#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

// Include enumerations for DriveCommand, DriveStatus, ControllerID, etc.
#include "merai/Enums.h"

namespace merai
{

    //====================================================
    // Constants
    //====================================================

    // Maximum number of servo drives supported by the system
    constexpr int MAX_SERVO_DRIVES = 12;

    constexpr std::uint32_t RT_MEMORY_MAGIC   = 0x52544D4D; // 'RTMM'
    constexpr std::uint32_t RT_MEMORY_VERSION = 2;          // bumped from 1 -> 2

    // For trajectory planning / control (7-DOF arm, modest limits).
    constexpr int TRAJ_MAX_JOINTS     = 7;    // matches your RobotModel::NUM_JOINTS
    constexpr int TRAJ_MAX_WAYPOINTS  = 64;   // GUI → planner waypoints
    constexpr int TRAJ_MAX_POINTS     = 512;  // planner → control dense samples

    //====================================================
    // Servo Fieldbus Data Structures
    //====================================================

    // ----------- Servo Rx (Commands -> from Controller to Servo) -----------

    // Control-related fields (e.g. control word, mode of operation)
    struct ServoRxControl
    {
        std::uint16_t controlWord = 0; // Command control word
    };

    // Motion-related fields (e.g. target torque, position, velocity)
    struct ServoRxMotion
    {
        std::uint8_t  modeOfOperation = 8; // Operating mode indicator
        std::int16_t  targetTorque    = 0; // Desired torque value (0x6071)
        std::int32_t  targetPosition  = 0; // Desired position value (0x607A)
        std::uint16_t maxTorque       = 0; // Max torque (0x6072)
    };

    // IO-related fields (digital/analog IO, if any)
    struct ServoRxIO
    {
        std::uint32_t digitalOutputs = 0; // 0x60FE:01
    };

    // Consolidated Rx PDO that groups the above sub-structs
    struct ServoRxPdo
    {
        ServoRxControl ctrl;
        ServoRxMotion  motion;
        ServoRxIO      io;
    };

    // ----------- Servo Tx (Feedback -> from Servo to Controller) -----------

    struct ServoTxControl
    {
        std::uint16_t statusWord = 0; // Feedback status word
    };

    struct ServoTxMotion
    {
        std::int32_t positionActual = 0; // Actual position feedback
        std::int32_t velocityActual = 0; // Actual velocity feedback
        std::int16_t torqueActual   = 0; // Actual torque feedback (0x6077)
    };

    struct ServoTxIO
    {
        std::uint32_t digitalInputs = 0; // 0x60FD
        std::uint16_t analogInput   = 0; // 0x2401
        std::uint16_t error_code    = 0; // 0x603F
    };

    struct ServoTxPdo
    {
        ServoTxControl ctrl;
        ServoTxMotion  motion;
        ServoTxIO      io;
    };

    //====================================================
    // Joint Command / Feedback Structures
    //====================================================

    struct JointControlCommand
    {
        std::uint16_t controlWord = 0;
    };

    struct JointMotionCommand
    {
        double        targetPosition   = 0.0;
        double        targetTorque     = 0.0;
        std::uint8_t  modeOfOperation  = 8; // tightly coupled to servo mode
    };

    struct JointCommandIO
    {
        // currently empty; extend as needed
    };

    struct JointCommandData
    {
        JointControlCommand control;
        JointMotionCommand  motion;
        JointCommandIO      io;
    };

    struct JointControlFeedback
    {
        std::uint16_t statusWord = 0;
    };

    struct JointMotionFeedback
    {
        double positionActual = 0.0;
        double velocityActual = 0.0;
        double torqueActual   = 0.0;
    };

    struct JointFeedbackIO
    {
        bool   digitalInputClutch = false;
        bool   digitalInputThumb  = false;
        double analogInputPinch   = 0.0;
    };

    struct JointFeedbackData
    {
        JointControlFeedback control;
        JointMotionFeedback  motion;
        JointFeedbackIO      io;
    };

    //====================================================
    // Drive Data Structures
    //====================================================

    struct DriveCommandData
    {
        std::array<merai::DriveCommand, MAX_SERVO_DRIVES> commands{};
    };

    struct DriveFeedbackData
    {
        std::array<merai::DriveStatus, MAX_SERVO_DRIVES> status{};
    };

    //====================================================
    // Controller Data Structures
    //====================================================

    struct ControllerCommand
    {
        bool                 requestSwitch = false;
        merai::ControllerID  controllerId  = merai::ControllerID::NONE;
    };

    struct ControllerFeedback
    {
        merai::ControllerFeedbackState feedbackState      = merai::ControllerFeedbackState::IDLE;
        merai::ControllerID            activeControllerId = merai::ControllerID::NONE;
        merai::ControllerSwitchResult  switchResult       = merai::ControllerSwitchResult::IDLE;
        std::uint64_t                  loopOverrunCount   = 0;
        std::uint64_t                  halErrorCount      = 0;
        bool                           loopOverrun        = false;
        bool                           controllerCommandFresh = true;
        bool                           driveCommandFresh       = true;
    };

    //====================================================
    // User Command and Feedback Structures
    //====================================================

    struct UserCommands
    {
        bool             eStop          = false;
        bool             resetFault     = false;
        bool             shutdownRequest = false;
        merai::UserMode  desiredMode    = merai::UserMode::HOMING;
    };

    struct UserFeedback
    {
        merai::AppState currentState = merai::AppState::INIT;
    };

    //====================================================
    // Planner / Trajectory Structures
    //====================================================

    struct JointTrajectoryWaypoint
    {
        double q  [TRAJ_MAX_JOINTS]{};  // position
        double qd [TRAJ_MAX_JOINTS]{};  // velocity
        double qdd[TRAJ_MAX_JOINTS]{};  // acceleration
    };

    /// Input payload for a joint trajectory planning job (logic → planner).
    struct JointTrajectoryRequestPayload
    {
        std::uint32_t dof          = TRAJ_MAX_JOINTS; // currently 7
        std::uint32_t num_waypoints = 0;              // <= TRAJ_MAX_WAYPOINTS
        JointTrajectoryWaypoint waypoints[TRAJ_MAX_WAYPOINTS]{};

        // Per-joint limits for this request
        double maxVel[TRAJ_MAX_JOINTS]{}; // [rad/s]
        double maxAcc[TRAJ_MAX_JOINTS]{}; // [rad/s^2]
    };

    /// Request from logic → planner.
    struct PlannerRequest
    {
        std::uint32_t       job_id  = 0;
        merai::PlannerJobType   type   = merai::PlannerJobType::NONE;
        merai::PlannerJobStatus status = merai::PlannerJobStatus::IDLE;

        JointTrajectoryRequestPayload jointTraj;
    };

    /// Result from planner → logic.
    struct PlannerResult
    {
        std::uint32_t       job_id  = 0;
        merai::PlannerJobStatus status = merai::PlannerJobStatus::IDLE;
    };

    /// Single point in a time-parameterized trajectory.
    struct JointTrajectoryPoint
    {
        double t = 0.0; // time from start [s]

        double q  [TRAJ_MAX_JOINTS]{}; // position
        double qd [TRAJ_MAX_JOINTS]{}; // velocity
        double qdd[TRAJ_MAX_JOINTS]{}; // acceleration
    };

    /// Full planned trajectory, written by planner, consumed by control.
    struct JointTrajectoryPlan
    {
        std::uint32_t job_id     = 0;
        std::uint32_t dof        = TRAJ_MAX_JOINTS;
        std::uint32_t num_points = 0;   // <= TRAJ_MAX_POINTS

        JointTrajectoryPoint points[TRAJ_MAX_POINTS]{};
    };

    /// Runtime execution status for a trajectory, written by control.
    struct JointTrajectoryStatus
    {
        std::uint32_t job_id        = 0;
        std::uint32_t active        = 0;    // 0 = inactive, 1 = active
        std::uint32_t current_index = 0;    // current sample index
        double        current_time  = 0.0;  // [s] since start

        bool finished = false;             // reached end of plan
        bool aborted  = false;             // stopped early / error
    };

    //====================================================
    // Double Buffer Template and Real-Time Memory Layout
    //====================================================

    template <typename T>
    struct alignas(64) DoubleBuffer
    {
        T                 buffer[2];                     // two buffers for double buffering
        std::atomic<int>  activeIndex{0};               // index of the currently published buffer (0 or 1)
    };

    struct alignas(64) RTMemoryLayout
    {
        std::uint32_t magic   = RT_MEMORY_MAGIC;
        std::uint32_t version = RT_MEMORY_VERSION;

        // Servo fieldbus data
        DoubleBuffer<std::array<ServoTxPdo, MAX_SERVO_DRIVES>> servoTxBuffer;
        DoubleBuffer<std::array<ServoRxPdo, MAX_SERVO_DRIVES>> servoRxBuffer;

        // High-level joint data
        DoubleBuffer<std::array<JointCommandData,  MAX_SERVO_DRIVES>> jointCommandBuffer;
        DoubleBuffer<std::array<JointFeedbackData, MAX_SERVO_DRIVES>> jointFeedbackBuffer;

        // Drive-level command and feedback
        DoubleBuffer<DriveCommandData>  driveCommandBuffer;
        DoubleBuffer<DriveFeedbackData> driveFeedbackBuffer;

        // Controller-level command and feedback
        DoubleBuffer<ControllerCommand>  controllerCommandBuffer;
        DoubleBuffer<ControllerFeedback> controllerFeedbackBuffer;

        // User commands and feedback
        DoubleBuffer<UserCommands> userCommandsBuffer;
        DoubleBuffer<UserFeedback> userFeedbackBuffer;

        // ---------- New: planner / trajectory buffers ----------

        // Logic → planner
        DoubleBuffer<PlannerRequest> plannerRequestBuffer;
        // Planner → logic
        DoubleBuffer<PlannerResult>  plannerResultBuffer;

        // Planner → control: time-parameterized trajectory
        DoubleBuffer<JointTrajectoryPlan>   jointTrajectoryPlanBuffer;

        // Control → logic/network: execution status
        DoubleBuffer<JointTrajectoryStatus> jointTrajectoryStatusBuffer;
    };

    // SHM contract: layout must stay trivially copyable/POD-friendly.
    static_assert(std::is_trivially_copyable<RTMemoryLayout>::value,
                  "RTMemoryLayout must be trivially copyable");

} // namespace merai
