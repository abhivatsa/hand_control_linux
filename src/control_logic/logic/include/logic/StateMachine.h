#pragma once

#include "merai/Enums.h"

namespace hand_control
{
    namespace logic
    {
        /**
         * @brief StateMachine (formerly SystemOrchestrator)
         *  - Manages high-level state transitions (INIT, HOMING, IDLE, ACTIVE, RECOVERY, FAULT)
         *  - Optionally sets drive signals or controller switches
         *  - Replaces references to "SystemOrchestrator"
         */
        class StateMachine
        {
        public:
            /**
             * @brief init 
             *  - Initializes internal flags, sets default state to INIT, etc.
             */
            bool init();

            /**
             * @brief update the StateMachine:
             *  - Check if there's a fault (faultActive, severity)
             *  - Check user requests (start, stop, or controller switch)
             *  - Perform state transitions
             *  - Possibly set per-drive signals or an all-drive command (if you choose)
             */
            void update(bool faultActive,
                        int faultSeverity,
                        bool userRequestedActive,
                        bool userRequestedControllerSwitch,
                        hand_control::merai::ControllerID desiredControllerId);

            /**
             * @brief wantsControllerSwitch
             *  Returns true if the user has requested a switch (set inside update).
             */
            bool wantsControllerSwitch() const;

            /**
             * @brief desiredControllerId
             *  The ID of the next desired controller if a switch is requested.
             */
            hand_control::merai::ControllerID desiredControllerId() const;

            /**
             * @brief Force the StateMachine into FAULT or RECOVERY directly.
             */
            void forceFault();
            void forceRecovery();

        private:
            bool systemReset() const;

            // The current state of the system
            hand_control::merai::AppState currentState_ = hand_control::merai::AppState::INIT;

            // If a user wants a new controller switch
            bool controllerSwitchWanted_ = false;

            // The ID of the next desired controller if a switch is requested
            hand_control::merai::ControllerID targetControllerId_ = hand_control::merai::ControllerID::NONE;
        };
    } // namespace logic
} // namespace hand_control
