#pragma once

#include <cstdint>

namespace merai
{
    // ===========================================
    // AppState
    // ===========================================
    // int32 is explicit but maps cleanly to 'int' on x86.
    enum class AppState : std::int32_t
    {
        INIT = 0,  ///< System is initializing
        HOMING,    ///< System is homing or calibrating
        ACTIVE,    ///< System is fully active / in run mode
        FAULT      ///< System is in an unrecoverable/severe fault
    };

    // ===========================================
    // UserMode
    // ===========================================
    enum class UserMode : std::int32_t
    {
        HOMING = 0,
        TELEOP,
        FAULT_RESET
    };

    // ===========================================
    // Controller-Related Enums
    // ===========================================
    enum class ControllerID : std::int32_t
    {
        NONE = 0,
        HOMING,
        GRAVITY_COMP,
        E_STOP
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

} // namespace merai
