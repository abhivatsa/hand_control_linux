#pragma once

#include "merai/Enums.h"

namespace hand_control
{
    namespace logic
    {
        /**
         * @brief SystemOrchestrator
         *  - Manages high-level state transitions (INIT, HOMING, IDLE, etc.)
         *  - Possibly sets individual drive signals (if your design calls for it)
         *  - Optionally sets a controller switch request using ControllerID
         */
        class SystemOrchestrator
        {
        public:
            bool init();

            /**
             * @brief update the orchestrator:
             *  - Check if there's a fault (faultActive, severity)
             *  - Check user requests (start, stop, or user requests controller switch)
             *  - Possibly switch states
             *  - Possibly set per-drive signals or an all-drive command
             */
            void update(bool faultActive, 
                        int faultSeverity,
                        bool userRequestedActive,
                        bool userRequestedControllerSwitch,
                        hand_control::merai::ControllerID desiredControllerId);

            /**
             * @brief wantsControllerSwitch
             *  Returns true if the orchestrator decided we should switch controllers
             */
            bool wantsControllerSwitch() const;

            /**
             * @brief desiredControllerId
             *  The new controller ID if a switch is requested
             */
            hand_control::merai::ControllerID desiredControllerId() const;

            // Force a fault or recovery if needed
            void forceFault();
            void forceRecovery();

        private:
            bool systemReset() const;

            // High-level system state
            hand_control::merai::OrchestratorState currentState_
                = hand_control::merai::OrchestratorState::INIT;

            // If we want a new controller switch
            bool controllerSwitchWanted_ = false;

            // The ID of the next desired controller if a switch is requested
            hand_control::merai::ControllerID targetControllerId_
                = hand_control::merai::ControllerID::NONE;
        };
    } // namespace logic
} // namespace hand_control
