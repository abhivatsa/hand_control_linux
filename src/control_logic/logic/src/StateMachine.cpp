#include "StateMachine.h"
#include <iostream>

namespace hand_control
{
    namespace logic
    {
        bool StateMachine::init()
        {
            currentState_           = merai::AppState::INIT;
            controllerSwitchWanted_ = false;
            targetControllerId_     = merai::ControllerID::NONE;

            // If you have arrays for drive signals, e.g.:
            // for (size_t i = 0; i < MAX_SERVO_DRIVES; ++i) {
            //     enableDrive_[i] = false;
            //     forceDisable_[i] = false;
            // }

            return true;
        }

        void StateMachine::update(bool faultActive,
                                  int faultSeverity,
                                  bool userRequestedActive,
                                  bool userRequestedControllerSwitch,
                                  merai::ControllerID desiredControllerId)
        {
            // 1) Handle faults first
            if (faultActive)
            {
                if (faultSeverity == 1)
                {
                    currentState_ = merai::AppState::RECOVERY;
                    std::cout << "[StateMachine] Entering RECOVERY (recoverable)\n";
                }
                else if (faultSeverity >= 2)
                {
                    currentState_ = merai::AppState::FAULT;
                    std::cout << "[StateMachine] Entering FAULT (unrecoverable)\n";
                }
            }

            // 2) State transitions
            switch (currentState_)
            {
            case merai::AppState::INIT:
                // Possibly do homing or go IDLE. 
                // For demonstration, let's go straight to ACTIVE:
                currentState_ = merai::AppState::ACTIVE;
                std::cout << "[StateMachine] INIT => ACTIVE.\n";
                break;

            case merai::AppState::HOMING:
                // If homing is complete, go IDLE or ACTIVE
                // Example logic here...
                break;

            case merai::AppState::IDLE:
                if (userRequestedActive)
                {
                    currentState_ = merai::AppState::ACTIVE;
                    std::cout << "[StateMachine] user requested ACTIVE => enabling drives.\n";
                }
                break;

            case merai::AppState::ACTIVE:
                // if user stops => IDLE
                if (!userRequestedActive)
                {
                    currentState_ = merai::AppState::IDLE;
                    std::cout << "[StateMachine] user stopped => IDLE.\n";
                }

                // if user wants a controller switch => record that
                if (userRequestedControllerSwitch)
                {
                    controllerSwitchWanted_ = true;
                    targetControllerId_     = desiredControllerId;
                    std::cout << "[StateMachine] user requested controller switch => ID = "
                              << static_cast<int>(desiredControllerId) << "\n";
                }
                break;

            case merai::AppState::RECOVERY:
                // Wait for user/system fix. If systemReset() => back to IDLE
                if (systemReset())
                {
                    std::cout << "[StateMachine] Recovery done => IDLE.\n";
                    currentState_ = merai::AppState::IDLE;
                }
                else
                {
                    // Possibly do partial fault resets for certain drives
                }
                break;

            case merai::AppState::FAULT:
                // remain in FAULT until user resets
                if (systemReset())
                {
                    std::cout << "[StateMachine] System reset => INIT.\n";
                    currentState_ = merai::AppState::INIT;
                }
                else
                {
                    // Possibly keep drives disabled
                }
                break;
            }
        }

        bool StateMachine::wantsControllerSwitch() const
        {
            return controllerSwitchWanted_;
        }

        merai::ControllerID StateMachine::desiredControllerId() const
        {
            return targetControllerId_;
        }

        void StateMachine::forceFault()
        {
            currentState_ = merai::AppState::FAULT;
            // e.g. set all drives disabled
            std::cout << "[StateMachine] Forcing FAULT => disabling drives.\n";
        }

        void StateMachine::forceRecovery()
        {
            currentState_ = merai::AppState::RECOVERY;
            std::cout << "[StateMachine] Forcing RECOVERY => attempt fault reset.\n";
        }

        bool StateMachine::systemReset() const
        {
            // Possibly check an aggregator or UI command for "reset"
            return false;
        }
    } // namespace logic
} // namespace hand_control
