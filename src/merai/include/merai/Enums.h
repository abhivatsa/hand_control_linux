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
            NONE         = 0,
            ENABLE_ALL   = 1,
            DISABLE_ALL  = 2,
            QUICK_STOP   = 3,
            FAULT_RESET  = 4,
            // Add others as needed
        };

        // ===========================================
        // Orchestrator-Related Enums
        // ===========================================
        /**
         * @brief High-level states in the system orchestration
         */
        enum class OrchestratorState : int
        {
            INIT,
            HOMING,
            IDLE,
            ACTIVE,
            RECOVERY,
            FAULT
            // Add others if needed
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
            NONE            = 0,
            HOMING          = 1,
            GRAVITY_COMP    = 2,
            E_STOP          = 3,
            // Extend as needed...
        };

        // Add more enum groups as needed, e.g. for error codes or logging levels
        // ===========================================
        // Error Codes
        // enum class ErrorCode : int { ... };
        // ===========================================
    } // namespace merai
} // namespace hand_control
