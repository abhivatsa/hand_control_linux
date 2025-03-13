#pragma once

namespace hand_control
{
    namespace merai
    {
        // ===========================================
        // Drive-Related Enums
        // ===========================================
        /**
         * @brief Defines high-level drive commands
         * that the Logic layer can issue and the Control layer can interpret.
         *
         * For example: NONE=0, ENABLE_ALL=1, etc.
         */
        enum class DriveCommand : int
        {
            NONE = 0,
            ENABLE_ALL = 1,
            DISABLE_ALL = 2,
            QUICK_STOP = 3,
            FAULT_RESET = 4,
            // Add others as needed
        };

        enum class OrchestratorState : int
        {
            IDLE = 0, ///< System is powered but inactive
            HOMING,   ///< System is homing or calibrating
            TELEOP,   ///< System is under teleoperation
            FAULT     ///< System is in fault condition
        };

        /**
         * @brief SystemMode
         *  - Represents the user's requested mode. The orchestrator decides
         *    how to map this mode to an actual OrchestratorState, depending
         *    on fault or other conditions.
         */
        enum class SystemMode : int
        {
            IDLE = 0, ///< User requests system idle
            HOMING,   ///< User requests homing
            TELEOP    ///< User requests teleoperation
            // Add more if needed (e.g. FREE_DRIVE, STOPPED, etc.)
        };

        // ===========================================
        // Controller-Related Enums
        // ===========================================
        /**
         * @brief Identifiers for each controller you want to run.
         *
         * Instead of passing a string like "HomingController" or "GravityCompController",
         * you can use these IDs in the real-time code.
         */
        enum class ControllerID : int
        {
            NONE = 0,
            HOMING = 1,
            GRAVITY_COMP = 2,
            E_STOP = 3,
            // Extend as needed...
        };

        // Add more enum groups as needed, e.g. for error codes or logging levels
        // ===========================================
        // Error Codes
        // enum class ErrorCode : int { ... };
        // ===========================================
    } // namespace merai
} // namespace hand_control
