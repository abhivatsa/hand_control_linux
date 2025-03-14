#include "logic/StateMachine.h"
#include <iostream>

namespace hand_control
{
    namespace logic
    {
        bool StateMachine::init()
        {
            currentState_ = merai::AppState::INIT;
            controllerSwitchWanted_ = false;
            targetControllerId_ = merai::ControllerID::NONE;

            // If you have arrays for drive signals, e.g.:
            // for (size_t i = 0; i < MAX_SERVO_DRIVES; ++i) {
            //     enableDrive_[i] = false;
            //     forceDisable_[i] = false;
            // }

            return true;
        }

        void StateMachine::update(bool faultActive, bool isHomingCompleted, hand_control::merai::UserCommands userCmds)
        {
            // 1) Handle faults first
            if (faultActive)
            {
                currentState_ = merai::AppState::FAULT;
                std::cout << "[StateMachine] Entering FAULT (unrecoverable)\n";
            }

            // 2) State transitions
            switch (currentState_)
            {
            case merai::AppState::INIT:

                if (!faultActive)
                {
                    currentState_ = merai::AppState::HOMING;
                }
                else
                {
                    currentState_ = merai::AppState::FAULT;
                }
                break;

            case merai::AppState::HOMING:
                if (!faultActive)
                {
                    if (isHomingCompleted)
                    {
                        currentState_ = merai::AppState::ACTIVE;
                    }
                }
                else
                {
                    currentState_ = merai::AppState::FAULT;
                }
                break;

            case merai::AppState::ACTIVE:
                if (faultActive)
                {
                    currentState_ = merai::AppState::FAULT;
                }
                break;

            case merai::AppState::FAULT:
                // remain in FAULT until user resets
                if (userCmds.resetFault)
                {
                    currentState_ = merai::AppState::INIT;
                }
                break;
            }
        }

    } // namespace logic
} // namespace hand_control
