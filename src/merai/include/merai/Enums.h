#pragma once

#include <cstdint>

namespace merai
{
    // ===========================================
    // AppState (high-level robot state)
    // ===========================================
    enum class AppState : int
    {
        INIT = 0,   ///< Starting up: SHM OK, drives/controllers not ready
        READY,      ///< Drives OK, no motion running (gravity comp / jog allowed)
        ACTIVE,     ///< Some motion is running (trajectory or jog)
        FAULT,      ///< Fault latched (drive or controller)
        ESTOP       ///< Hardware or software e-stop asserted
    };

    // ===========================================
    // UserMode (GUI / user intent, optional)
    // ===========================================
    enum class UserMode : int
    {
        HOMING = 0,
        TELEOP,
        FAULT_RESET
        // Extend / rename as needed when we finalize GUI commands…
    };

    // ===========================================
    // Controller-Related Enums
    // ===========================================
    enum class ControllerID : int
    {
        NONE = 0,
        GRAVITY_COMP,
        E_STOP,
        JOINT_TRAJECTORY,  ///< Plays a pre-planned joint trajectory
        JOINT_JOG          ///< Joint jog controller
    };

    // ===========================================
    // Drive Command (one at a time)
    // ===========================================
    enum class DriveCommand : std::uint8_t
    {
        NONE = 0,
        FAULT_RESET,
        ALLOW_OPERATION,
        FORCE_DISABLE,
        SWITCH_ON
    };

    // ===========================================
    // Drive Status (one exclusive state)
    // ===========================================
    enum class DriveStatus : std::uint8_t
    {
        NOT_READY_TO_SWITCH_ON = 0,
        READY_TO_SWITCH_ON,
        SWITCHED_ON,
        OPERATION_ENABLED,
        FAULT,
        QUICK_STOP,
        SWITCH_ON_DISABLED
    };

    // ===========================================
    // Controller Feedback State
    // ===========================================
    enum class ControllerFeedbackState : std::uint8_t
    {
        IDLE = 0,
        SWITCH_IN_PROGRESS,
        SWITCH_COMPLETED,
        SWITCH_FAILED
    };

    enum class ControllerSwitchResult : std::uint8_t
    {
        IDLE = 0,
        IN_PROGRESS,
        SUCCEEDED,
        FAILED
    };

    // ===========================================
    // Drive / EtherCAT config enums
    // ===========================================
    enum class DriveType : std::uint8_t
    {
        Unknown = 0,
        Servo,
        Io
    };

    enum class SyncType : std::uint8_t
    {
        Unknown = 0,
        RxPdo,
        TxPdo
    };

    enum class PdoDataType : std::uint8_t
    {
        Unknown = 0,
        Int8,
        Int16,
        Int32,
        UInt16,
        UInt32
    };

    // ===========================================
    // Planner-related enums
    // ===========================================

    /// What kind of job the planner should run.
    enum class PlannerJobType : std::uint32_t
    {
        NONE = 0,
        JOINT_TRAJECTORY = 1,
        // CARTESIAN_TRAJECTORY = 2, // future
        // IK_SOLVE            = 3, // future
    };

    /// Lifecycle of a planner job.
    enum class PlannerJobStatus : std::uint32_t
    {
        IDLE = 0,
        PENDING = 1,
        RUNNING = 2,
        DONE = 3,
        FAILED = 4,
        REJECTED = 5,
    };

} // namespace merai
