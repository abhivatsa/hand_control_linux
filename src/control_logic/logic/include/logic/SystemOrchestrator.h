#pragma once

#include <string>
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "logic/ErrorManager.h"

namespace hand_control
{
    namespace logic
    {
        enum class OrchestratorState
        {
            INIT,
            HOMING,
            IDLE,
            ACTIVE,
            FAULT
        };

        class SystemOrchestrator
        {
        public:
            bool init(const hand_control::merai::ParameterServer* paramServer,
                      hand_control::merai::RTMemoryLayout* rtLayout,
                      ErrorManager* errorMgr);

            void update();

            void forceEmergencyStop();

        private:
            // Helper conditions
            bool needHoming() const;
            bool homingComplete() const;
            bool userRequestedActive() const;
            bool userStops() const;
            bool systemReset() const;

            const hand_control::merai::ParameterServer* paramServer_ = nullptr;
            hand_control::merai::RTMemoryLayout*        rtLayout_    = nullptr;
            ErrorManager*                               errorMgr_    = nullptr;

            OrchestratorState currentState_ = OrchestratorState::INIT;
        };
    }
}
