#pragma once

#include <cstdint>

namespace merai
{
    // ===========================================
    // AppState
    // ===========================================
    enum class AppState : int
    {
        INIT = 0, ///< System is initializing
        HOMING,   ///< System is homing or calibrating
        ACTIVE,   ///< System is fully active / in run mode
        FAULT     ///< System is in an unrecoverable/severe fault
    };

    // ===========================================
    // UserMode
    // ===========================================
    enum class UserMode : int
    {
        HOMING = 0,
        TELEOP,
        FAULT_RESET
        // Add more if needed...
    };

    // ===========================================
    // Controller-Related Enums
    // ===========================================
    enum class ControllerID : int
    {
        NONE = 0,
        HOMING,
        GRAVITY_COMP,
        E_STOP
        // Extend as needed...
    };

    // ===========================================
    // Drive Command (one at a time)
    // ===========================================
    enum class DriveCommand : uint8_t
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
    enum class DriveStatus : uint8_t
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
    enum class ControllerFeedbackState : uint8_t
    {
        IDLE = 0,
        SWITCH_IN_PROGRESS,
        SWITCH_COMPLETED,
        SWITCH_FAILED
    };

    enum class ControllerSwitchResult : uint8_t
    {
        IDLE = 0,
        IN_PROGRESS,
        SUCCEEDED,
        FAILED
    };

    enum class DriveType : uint8_t
    {
        Unknown = 0,
        Servo,
        Io
    };

    enum class SyncType : uint8_t
    {
        Unknown = 0,
        RxPdo,
        TxPdo
    };

    enum class PdoDataType : uint8_t
    {
        Unknown = 0,
        Int8,
        Int16,
        Int32,
        UInt16,
        UInt32
    };

} // namespace merai
