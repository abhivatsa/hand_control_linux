#include <cstdio>
#include "logic/SystemOrchestrator.h"

namespace hand_control
{
    namespace logic
    {
        SystemOrchestrator::SystemOrchestrator(ErrorManager& errMgr)
            : errorManager_(errMgr),
              currentState_(OrchestratorState::INIT),
              wantEnable_(false),
              wantRun_(false)
        {
        }

        void SystemOrchestrator::update()
        {
            // If we detect any errors and aren't already in ERROR state, go there
            if (errorManager_.hasActiveErrors() &&
                currentState_ != OrchestratorState::ERROR)
            {
                std::printf("[SystemOrchestrator] Transition to ERROR (due to active errors)\n");
                currentState_ = OrchestratorState::ERROR;
            }

            switch (currentState_)
            {
            case OrchestratorState::INIT:
                handleStateINIT();
                break;

            case OrchestratorState::READY:
                handleStateREADY();
                break;

            case OrchestratorState::RUNNING:
                handleStateRUNNING();
                break;

            case OrchestratorState::ERROR:
                handleStateERROR();
                break;

            case OrchestratorState::SHUTDOWN:
                // Possibly do nothing or final steps
                break;
            }
        }

        void SystemOrchestrator::handleStateINIT()
        {
            // Example: if user wants enable & no errors, move to READY
            if (wantEnable_ && !errorManager_.hasActiveErrors())
            {
                std::printf("[SystemOrchestrator] Transition INIT->READY\n");
                currentState_ = OrchestratorState::READY;
            }
        }

        void SystemOrchestrator::handleStateREADY()
        {
            // If user wants to run & no errors
            if (wantRun_ && !errorManager_.hasActiveErrors())
            {
                std::printf("[SystemOrchestrator] Transition READY->RUNNING\n");
                currentState_ = OrchestratorState::RUNNING;
            }
        }

        void SystemOrchestrator::handleStateRUNNING()
        {
            // If user no longer wants run or an error occurs, we leave RUNNING
            if (!wantRun_)
            {
                std::printf("[SystemOrchestrator] Transition RUNNING->READY\n");
                currentState_ = OrchestratorState::READY;
            }
        }

        void SystemOrchestrator::handleStateERROR()
        {
            // Typically wait until user calls resetErrors() or system is re-initialized
            // No automatic transition out of ERROR unless we confirm errors cleared
        }

        void SystemOrchestrator::requestEnable()
        {
            wantEnable_ = true;
        }

        void SystemOrchestrator::requestRun()
        {
            wantRun_ = true;
        }

        void SystemOrchestrator::requestDisable()
        {
            // For example, if we want to fully shut down
            wantEnable_ = false;
            wantRun_    = false;
            currentState_ = OrchestratorState::SHUTDOWN;
        }

        void SystemOrchestrator::resetErrors()
        {
            // Clear all errors
            errorManager_.clearAll();

            // Optionally return to INIT
            if (!errorManager_.hasActiveErrors())
            {
                std::printf("[SystemOrchestrator] Errors cleared, returning to INIT.\n");
                currentState_ = OrchestratorState::INIT;
            }
        }
    } // namespace logic
} // namespace hand_control
