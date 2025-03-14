#pragma once

#include <array>
#include <atomic>
#include <cstdint>

namespace hand_control
{
    namespace merai
    {
        // ===========================================
        // Drive-Related Enums
        // ===========================================
        enum class DriveCommand : int
        {
            NONE = 0,
            ENABLE_ALL,
            DISABLE_ALL,
            QUICK_STOP,
            FAULT_RESET,
            // Add others as needed
        };

        // ===========================================
        // Updated AppState (replaces OrchestratorState)
        // ===========================================
        enum class AppState : int
        {
            INIT = 0,   ///< System is initializing
            HOMING,     ///< System is homing or calibrating
            IDLE,       ///< System is powered but inactive
            ACTIVE,     ///< System is fully active / in run mode
            FAULT       ///< System is in an unrecoverable/severe fault
        };

        /**
         * @brief SystemMode
         *  - Represents the user's requested mode. The StateMachine decides
         *    how to map this mode to an actual AppState, depending on faults, etc.
         */
        enum class SystemMode : int
        {
            HOMING,
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
            E_STOP,
            // Extend as needed...
        };

    } // namespace merai
} // namespace hand_control
