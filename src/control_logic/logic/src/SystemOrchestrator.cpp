#include "SystemOrchestrator.h"
#include <iostream>
#include "merai/Enums.h" // for OrchestratorState, ControllerID, etc.

namespace hand_control
{
    namespace logic
    {
        bool SystemOrchestrator::init()
        {
            currentState_           = merai::OrchestratorState::INIT;
            controllerSwitchWanted_ = false;
            targetControllerId_     = merai::ControllerID::NONE;

            // If you have arrays for drive signals, e.g.:
            // for (size_t i = 0; i < MAX_SERVO_DRIVES; ++i) {
            //     enableDrive_[i] = false;
            //     forceDisable_[i] = false;
            // }

            return true;
        }

        void SystemOrchestrator::update(bool faultActive,
                                        int faultSeverity,
                                        bool userRequestedActive,
                                        bool userRequestedControllerSwitch,
                                        merai::ControllerID desiredControllerId)
        {
            // 1) Handle faults
            if (faultActive)
            {
                if (faultSeverity == 1)
                {
                    currentState_ = merai::OrchestratorState::RECOVERY;
                    std::cout << "[Orchestrator] Entering RECOVERY (recoverable)\n";
                }
                else if (faultSeverity >= 2)
                {
                    currentState_ = merai::OrchestratorState::FAULT;
                    std::cout << "[Orchestrator] Entering FAULT (unrecoverable)\n";
                }
            }

            // 2) State machine
            switch (currentState_)
            {
            case merai::OrchestratorState::INIT:
                // Possibly do homing or go IDLE. For demonstration, let's just go active:
                // e.g. set some internal enableDrive_[i] = true
                currentState_ = merai::OrchestratorState::ACTIVE;
                std::cout << "[Orchestrator] After INIT => set drives active.\n";
                break;

            case merai::OrchestratorState::HOMING:
                // If homing is complete, go IDLE or ACTIVE
                break;

            case merai::OrchestratorState::IDLE:
                if (userRequestedActive)
                {
                    // e.g. set enableDrive_[i] = true
                    currentState_ = merai::OrchestratorState::ACTIVE;
                    std::cout << "[Orchestrator] user requested ACTIVE => enabling drives.\n";
                }
                break;

            case merai::OrchestratorState::ACTIVE:
                // if user stops => IDLE
                if (!userRequestedActive)
                {
                    // e.g. set enableDrive_[i] = false
                    currentState_ = merai::OrchestratorState::IDLE;
                    std::cout << "[Orchestrator] user stopped => IDLE, disabling drives.\n";
                }

                // if user wants a controller switch => record that
                if (userRequestedControllerSwitch)
                {
                    controllerSwitchWanted_ = true;
                    targetControllerId_     = desiredControllerId;
                    std::cout << "[Orchestrator] user requested controller switch => ID = "
                              << static_cast<int>(desiredControllerId) << "\n";
                }
                break;

            case merai::OrchestratorState::RECOVERY:
                // Wait for user/system fix. If systemReset() => back to IDLE
                if (systemReset())
                {
                    std::cout << "[Orchestrator] Recovery done => IDLE.\n";
                    currentState_ = merai::OrchestratorState::IDLE;
                }
                else
                {
                    // Possibly do partial fault resets for certain drives
                }
                break;

            case merai::OrchestratorState::FAULT:
                // remain in FAULT until user resets
                if (systemReset())
                {
                    std::cout << "[Orchestrator] System reset => back to INIT.\n";
                    currentState_ = merai::OrchestratorState::INIT;
                }
                else
                {
                    // Possibly keep drives disabled
                }
                break;
            }
        }

        bool SystemOrchestrator::wantsControllerSwitch() const
        {
            return controllerSwitchWanted_;
        }

        merai::ControllerID SystemOrchestrator::desiredControllerId() const
        {
            return targetControllerId_;
        }

        void SystemOrchestrator::forceFault()
        {
            currentState_        = merai::OrchestratorState::FAULT;
            // e.g. set all drives disabled
            std::cout << "[Orchestrator] Forcing FAULT => disabling drives.\n";
        }

        void SystemOrchestrator::forceRecovery()
        {
            currentState_        = merai::OrchestratorState::RECOVERY;
            std::cout << "[Orchestrator] Forcing RECOVERY => attempt fault reset.\n";
        }

        bool SystemOrchestrator::systemReset() const
        {
            // Possibly check an aggregator or UI command for "reset"
            return false;
        }

    } // namespace logic
} // namespace hand_control
