#include "SystemOrchestrator.h"
#include <iostream>

namespace hand_control
{
    namespace logic
    {
        bool SystemOrchestrator::init()
        {
            currentState_ = OrchestratorState::INIT;
            currentDriveCmd_ = DriveCommand::NONE;
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
            currentDriveCmd_ = DriveCommand::NONE;
            controllerSwitchWanted_ = false;
            targetControllerName_.clear();

            // 1) If fault is active, decide if it's recoverable or not
            if (faultActive)
            {
                if (faultSeverity == 1)
                {
                    currentState_ = OrchestratorState::RECOVERY;
                    std::cout << "[Orchestrator] Entering RECOVERY (recoverable)\n";
                }
                else if (faultSeverity >= 2)
                {
                    currentState_ = OrchestratorState::FAULT;
                    std::cout << "[Orchestrator] Entering FAULT (unrecoverable)\n";
                }
            }

            // 2) Switch on current state
            switch (currentState_)
            {
            case OrchestratorState::INIT:
                // Possibly we want to go to HOMING or IDLE
                // for demonstration, let's enable all drives after INIT
                // so we can do homing or so
                currentDriveCmd_ = DriveCommand::ENABLE_ALL;
                // if homing needed, we might set currentState_ = HOMING
                break;

            case OrchestratorState::HOMING:
                // Once homing complete => IDLE
                // for demonstration, we do nothing
                break;

            case OrchestratorState::IDLE:
                if (userRequestedActive)
                {
                    // Move to ACTIVE => enable drives
                    currentDriveCmd_ = DriveCommand::ENABLE_ALL;
                    currentState_ = OrchestratorState::ACTIVE;
                    std::cout << "[Orchestrator] user requested ACTIVE => enabling drives.\n";
                }
                break;

            case OrchestratorState::ACTIVE:
                // If user stops => IDLE, for demonstration do a disable
                // Or if user wants a controller switch => set wanted
                if (!userRequestedActive)
                {
                    // user no longer wants active => go IDLE
                    currentDriveCmd_ = DriveCommand::DISABLE_ALL;
                    currentState_ = OrchestratorState::IDLE;
                    std::cout << "[Orchestrator] user stopped => IDLE, disabling drives.\n";
                }

                if (userRequestedControllerSwitch)
                {
                    controllerSwitchWanted_ = true;
                    targetControllerName_ = controllerName;
                    // we do not necessarily exit ACTIVE state,
                    // we just set that we want a new controller
                    std::cout << "[Orchestrator] user requested controller switch to: "
                              << controllerName << "\n";
                }
                break;

            case OrchestratorState::RECOVERY:
                // Wait for user or system to fix the issue
                // if user says done => might go back to IDLE or INIT
                if (systemReset())
                {
                    std::cout << "[Orchestrator] Recovery done => returning to IDLE.\n";
                    currentState_ = OrchestratorState::IDLE;
                }
                else
                {
                    // maybe we do a fault reset
                    currentDriveCmd_ = DriveCommand::FAULT_RESET;
                }
                break;

            case OrchestratorState::FAULT:
                // remain in FAULT until user reset
                if (systemReset())
                {
                    std::cout << "[Orchestrator] System reset => back to INIT.\n";
                    currentState_ = OrchestratorState::INIT;
                }
                else
                {
                    // possibly we do a DISABLE_ALL or keep motors off
                    currentDriveCmd_ = DriveCommand::DISABLE_ALL;
                }
                break;
            }
        }

        DriveCommand SystemOrchestrator::getDriveCommand() const
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
            currentState_ = OrchestratorState::FAULT;
            currentDriveCmd_ = DriveCommand::DISABLE_ALL;
            std::cout << "[Orchestrator] Forcing FAULT => disabling drives.\n";
        }

        void SystemOrchestrator::forceRecovery()
        {
            currentState_ = OrchestratorState::RECOVERY;
            currentDriveCmd_ = DriveCommand::FAULT_RESET;
            std::cout << "[Orchestrator] Forcing RECOVERY => attempt fault reset.\n";
        }

        bool SystemOrchestrator::systemReset() const
        {
            // check aggregator or user input if "reset" is pressed
            return false;
        }
    }
}
