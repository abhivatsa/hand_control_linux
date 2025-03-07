#pragma once

#include <string>

// Pull in the enums from merai/Enums.h
#include "merai/Enums.h"

namespace hand_control
{
    namespace logic
    {
        /**
         * @brief SystemOrchestrator
         *  - Manages high-level state transitions (INIT, HOMING, IDLE, etc.)
         *  - Chooses a single drive command (ENABLE_ALL, DISABLE_ALL, etc.)
         *  - Optionally sets a controller switch request
         */
        class SystemOrchestrator
        {
        public:
            bool init();

            /**
             * @brief update the orchestrator:
             *  - Check if there's a fault (faultActive, severity)
             *  - Check user requests (start, stop, or user requests controller switch)
             *  - Decide a single drive command (enable, disable, etc.)
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
            hand_control::merai::DriveCommand getDriveCommand() const;

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
            // We now use the OrchestratorState from merai::Enums.h
            hand_control::merai::OrchestratorState currentState_ 
                = hand_control::merai::OrchestratorState::INIT;

            // Single enumerated drive command
            hand_control::merai::DriveCommand currentDriveCmd_ 
                = hand_control::merai::DriveCommand::NONE;

            // If we want a new controller switch
            bool controllerSwitchWanted_ = false;
            std::string targetControllerName_;

            // Possibly user event checks
            bool systemReset() const;
        };
    } // namespace logic
} // namespace hand_control
