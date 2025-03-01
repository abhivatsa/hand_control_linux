#pragma once

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "logic/ErrorManager.h"

namespace hand_control
{
    namespace logic
    {
        class SafetyManager
        {
        public:
            bool init(const merai::ParameterServer* paramServer,
                      merai::RTMemoryLayout* rtLayout,
                      ErrorManager* errorMgr);

            void update();

            bool isFaulted() const;
            void clearFault();

        private:
            const merai::ParameterServer* paramServer_ = nullptr;
            merai::RTMemoryLayout*        rtLayout_    = nullptr;
            ErrorManager*                 errorMgr_    = nullptr;

            bool faulted_{false};
        };
    }
}
