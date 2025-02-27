#pragma once

#include <cstdint>

namespace hand_control
{
    namespace fieldbus
    {
        enum class FieldbusState
        {
            POWER_ON,
            STARTUP,
            SAFETY_CHECK,
            PRE_OPERATION,
            OPERATIONAL,
            STOP_0,       // Immediate power removal (E-stop)
            STOP_1,       // Controlled stop, then power removal
            STOP_2,       // Controlled stop with power retained
            FAULT,
            SHUTDOWN
        };

        enum class StopCause
        {
            NONE,
            ESTOP_TRIGGERED,
            SAFETY_CIRCUIT_FAIL,
            DRIVE_FAULT,
            USER_REQUESTED_STOP,
            UNKNOWN
        };
    } // namespace fieldbus

    namespace control
    {
        enum class EControlMode
        {
            Teleop,     // Teleoperation controller
            Gravity,    // Gravity compensation controller
            Trajectory  // Joint trajectory controller
            // Add more as needed...
        };
    } // namespace control

    namespace logic
    {
        enum class LogicLimitCheck
        {
            OK,
            NOT_OK,
            NOT_EVAL
        };

        enum class UserInputState
        {
            INIT,
            START,
            SWITCH_ON,
            IDLE,
            HANDCONTROLLER,
            HOMING,
            OPERATIONAL
        };

        enum class LogicState
        {
            INIT,
            START,
            IDLE,
            HANDCONTROLLER,
            HOMING,
            OPERATIONAL
        };
    } // namespace logic
} // namespace hand_control
