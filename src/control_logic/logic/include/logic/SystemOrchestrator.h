#pragma once

#include <string>

namespace hand_control
{
    namespace logic
    {
        // The possible orchestrator states
        enum class OrchestratorState
        {
            INIT,
            HOMING,
            IDLE,
            ACTIVE,
            RECOVERY,
            FAULT
        };

        // A simple enum for drive commands
        enum class DriveCommand : int
        {
            NONE        = 0,
            ENABLE_ALL  = 1,
            DISABLE_ALL = 2,
            QUICK_STOP  = 3,
            FAULT_RESET = 4,
            // add more as needed
        };

        class SystemOrchestrator
        {
        public:
            bool init();

            /**
             * @brief update the orchestrator:
             *  - Check if there's a fault (faultActive, severity)
             *  - Check user requests (start, stop, or user requests controller switch)
             *  - Decide a single DriveCommand (enable, disable, etc.)
             *  - Possibly switch states
             */
            void update(bool faultActive, int faultSeverity,
                        bool userRequestedActive,
                        bool userRequestedControllerSwitch,
                        const std::string& controllerName);

            /**
             * @brief getDriveCommand
             *  The single enumerated command the logic should write to shared memory
             */
            DriveCommand getDriveCommand() const;

            /**
             * @brief wantsControllerSwitch
             *  Returns true if the orchestrator decided we should switch controllers
             */
            bool wantsControllerSwitch() const;

            /**
             * @brief desiredControllerName
             *  The new controller name if a switch is requested
             */
            const std::string& desiredControllerName() const;

            // Force a fault or recovery if needed
            void forceFault();
            void forceRecovery();

        private:
            OrchestratorState currentState_ = OrchestratorState::INIT;

            // Internal drive command for this cycle
            DriveCommand currentDriveCmd_ = DriveCommand::NONE;

            // If we want a new controller switch
            bool controllerSwitchWanted_ = false;
            std::string targetControllerName_;

            // Possibly user event checks
            bool systemReset() const;
            // e.g. might read aggregator or do logic
        };
    }
}
