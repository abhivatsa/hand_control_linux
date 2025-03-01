#include "logic/SystemOrchestrator.h"
#include <iostream>

namespace hand_control
{
    namespace logic
    {
        bool SystemOrchestrator::init(const merai::ParameterServer* paramServer,
                                      merai::RTMemoryLayout* rtLayout,
                                      ErrorManager* errorMgr)
        {
            paramServer_ = paramServer;
            rtLayout_    = rtLayout;
            errorMgr_    = errorMgr;

            currentState_ = OrchestratorState::INIT;
            return true;
        }

        void SystemOrchestrator::update()
        {
            switch (currentState_)
            {
            case OrchestratorState::INIT:
                if (needHoming())
                {
                    currentState_ = OrchestratorState::HOMING;
                    std::cout << "[Orchestrator] Transition to HOMING.\n";
                }
                else
                {
                    currentState_ = OrchestratorState::IDLE;
                    std::cout << "[Orchestrator] Transition to IDLE.\n";
                }
                break;

            case OrchestratorState::HOMING:
                if (homingComplete())
                {
                    currentState_ = OrchestratorState::IDLE;
                    std::cout << "[Orchestrator] Homing done, now IDLE.\n";
                }
                // If homing fails, we can report an error:
                // errorMgr_->reportError(101, "Homing failure on joint X");
                break;

            case OrchestratorState::IDLE:
                if (userRequestedActive())
                {
                    currentState_ = OrchestratorState::ACTIVE;
                    std::cout << "[Orchestrator] Going ACTIVE.\n";
                }
                break;

            case OrchestratorState::ACTIVE:
                if (userStops())
                {
                    currentState_ = OrchestratorState::IDLE;
                    std::cout << "[Orchestrator] User stopped, back to IDLE.\n";
                }
                break;

            case OrchestratorState::FAULT:
                // Remain in FAULT until systemReset() is true
                if (systemReset())
                {
                    currentState_ = OrchestratorState::INIT;
                    std::cout << "[Orchestrator] System reset, back to INIT.\n";
                }
                break;
            }
        }

        void SystemOrchestrator::forceEmergencyStop()
        {
            currentState_ = OrchestratorState::FAULT;
            std::cout << "[Orchestrator] Forcing E-STOP => FAULT state.\n";
            if (errorMgr_)
            {
                errorMgr_->reportError(999, "Emergency Stop triggered by orchestrator");
            }
        }

        // Stub condition checks
        bool SystemOrchestrator::needHoming() const
        {
            return paramServer_ && paramServer_->startup.requireHoming;
        }
        bool SystemOrchestrator::homingComplete() const
        {
            return false;
        }
        bool SystemOrchestrator::userRequestedActive() const
        {
            return false;
        }
        bool SystemOrchestrator::userStops() const
        {
            return false;
        }
        bool SystemOrchestrator::systemReset() const
        {
            return false;
        }
    }
}
