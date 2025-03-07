#include "SystemOrchestrator.h"
#include <iostream>

// If needed for clarity:
#include "merai/Enums.h"

namespace hand_control
{
    namespace logic
    {
        bool SystemOrchestrator::init()
        {
            currentState_        = merai::OrchestratorState::INIT;
            currentDriveCmd_     = merai::DriveCommand::NONE;
            controllerSwitchWanted_ = false;
            targetControllerName_.clear();
            return true;
        }

        void SystemOrchestrator::update(bool faultActive, int faultSeverity,
                                        bool userRequestedActive,
                                        bool userRequestedControllerSwitch,
                                        const std::string& controllerName)
        {
            // Reset the drive command each cycle to NONE by default
            currentDriveCmd_        = merai::DriveCommand::NONE;
            controllerSwitchWanted_ = false;
            targetControllerName_.clear();

            // 1) If fault is active, decide if it's recoverable or not
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

            // 2) Switch on current state
            switch (currentState_)
            {
            case merai::OrchestratorState::INIT:
                // Possibly we want to go to HOMING or IDLE
                // for demonstration, let's enable all drives after INIT
                currentDriveCmd_ = merai::DriveCommand::ENABLE_ALL;
                // if homing needed, we might set currentState_ = HOMING
                break;

            case merai::OrchestratorState::HOMING:
                // Once homing complete => IDLE
                break;

            case merai::OrchestratorState::IDLE:
                if (userRequestedActive)
                {
                    currentDriveCmd_ = merai::DriveCommand::ENABLE_ALL;
                    currentState_    = merai::OrchestratorState::ACTIVE;
                    std::cout << "[Orchestrator] user requested ACTIVE => enabling drives.\n";
                }
                break;

            case merai::OrchestratorState::ACTIVE:
                // If user stops => IDLE
                if (!userRequestedActive)
                {
                    currentDriveCmd_ = merai::DriveCommand::DISABLE_ALL;
                    currentState_    = merai::OrchestratorState::IDLE;
                    std::cout << "[Orchestrator] user stopped => IDLE, disabling drives.\n";
                }

                // If user wants a controller switch => set that
                if (userRequestedControllerSwitch)
                {
                    controllerSwitchWanted_ = true;
                    targetControllerName_   = controllerName;
                    std::cout << "[Orchestrator] user requested controller switch to: "
                              << controllerName << "\n";
                }
                break;

            case merai::OrchestratorState::RECOVERY:
                // Wait for user or system to fix the issue
                if (systemReset())
                {
                    std::cout << "[Orchestrator] Recovery done => returning to IDLE.\n";
                    currentState_ = merai::OrchestratorState::IDLE;
                }
                else
                {
                    currentDriveCmd_ = merai::DriveCommand::FAULT_RESET;
                }
                break;

            case merai::OrchestratorState::FAULT:
                // remain in FAULT until user reset
                if (systemReset())
                {
                    std::cout << "[Orchestrator] System reset => back to INIT.\n";
                    currentState_ = merai::OrchestratorState::INIT;
                }
                else
                {
                    currentDriveCmd_ = merai::DriveCommand::DISABLE_ALL;
                }
                break;
            }
        }

        merai::DriveCommand SystemOrchestrator::getDriveCommand() const
        {
            return currentDriveCmd_;
        }

        bool SystemOrchestrator::wantsControllerSwitch() const
        {
            return controllerSwitchWanted_;
        }

        const std::string& SystemOrchestrator::desiredControllerName() const
        {
            return targetControllerName_;
        }

        void SystemOrchestrator::forceFault()
        {
            currentState_    = merai::OrchestratorState::FAULT;
            currentDriveCmd_ = merai::DriveCommand::DISABLE_ALL;
            std::cout << "[Orchestrator] Forcing FAULT => disabling drives.\n";
        }

        void SystemOrchestrator::forceRecovery()
        {
            currentState_    = merai::OrchestratorState::RECOVERY;
            currentDriveCmd_ = merai::DriveCommand::FAULT_RESET;
            std::cout << "[Orchestrator] Forcing RECOVERY => attempt fault reset.\n";
        }

        bool SystemOrchestrator::systemReset() const
        {
            // Example check if aggregator or user input says "reset"
            return false;
        }

    } // namespace logic
} // namespace hand_control
