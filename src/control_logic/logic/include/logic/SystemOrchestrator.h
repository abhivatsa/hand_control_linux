#pragma once

#include "ErrorManager.h"

namespace hand_control
{
    namespace logic
    {
        /**
         * @brief Possible states for the orchestrator’s top-level system mode.
         */
        enum class OrchestratorState
        {
            INIT = 0,
            READY,
            RUNNING,
            ERROR,
            SHUTDOWN
        };

        /**
         * @brief SystemOrchestrator manages high-level transitions (e.g., from INIT → READY → RUNNING),
         *        handling user requests and monitoring error states.
         */
        class SystemOrchestrator
        {
        public:
            explicit SystemOrchestrator(hand_control::logic::ErrorManager& errMgr);

            /**
             * @brief Update call at ~100 Hz (or any chosen rate) to process commands and handle state transitions.
             */
            void update();

            /**
             * @brief Requests from user or external modules
             */
            void requestEnable();
            void requestRun();
            void requestDisable();
            void resetErrors();

            /**
             * @brief Gets the current orchestrator state.
             * @return OrchestratorState enum value (INIT, READY, RUNNING, etc.).
             */
            OrchestratorState getCurrentState() const
            {
                return currentState_;
            }

        private:
            void handleStateINIT();
            void handleStateREADY();
            void handleStateRUNNING();
            void handleStateERROR();

            hand_control::logic::ErrorManager& errorManager_;
            OrchestratorState currentState_;

            // Flags (like user-intent commands)
            bool wantEnable_ = false;
            bool wantRun_    = false;
        };
    } // namespace logic
} // namespace hand_control
