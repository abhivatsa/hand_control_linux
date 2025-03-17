#include "logic/StateMachine.h"
#include <iostream>

namespace hand_control
{
    namespace logic
    {
        bool StateMachine::init()
        {
            currentState_ = merai::AppState::INIT;
            return true;
        }

        StateManagerOutput StateMachine::update(bool faultActive, bool isHomingCompleted, hand_control::merai::UserCommands userCmds)
        {
            StateManagerOutput output;

            // 1) Handle faults first
            if (faultActive)
            {
                currentState_ = merai::AppState::FAULT;
                std::cout << "[StateMachine] Entering FAULT (unrecoverable)\n";
            }
            else
            {
                // 2) State transitions based on current state and homing status
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
                    // Remain in FAULT until the user resets the fault.
                    if (userCmds.resetFault)
                    {
                        currentState_ = merai::AppState::INIT;
                    }
                    break;

                default:
                    break;
                }
            }

            // 3) Set the output app state
            output.appState = currentState_;

            // 4) Compute drive and controller commands.
            //    Replace the default construction with your actual command logic as needed.
            output.driveCmd = hand_control::merai::DriveCommand{};
            output.ctrlCmd  = hand_control::merai::ControllerCommand{};

            return output;
        }
    } // namespace logic
} // namespace hand_control
