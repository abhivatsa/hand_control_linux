#include "logic/SafetyManager.h"
#include <iostream>

namespace hand_control
{
    namespace logic
    {
        bool SafetyManager::init(const merai::ParameterServer* paramServer,
                                 merai::RTMemoryLayout* rtLayout,
                                 ErrorManager* errorMgr)
        {
            paramServer_ = paramServer;
            rtLayout_    = rtLayout;
            errorMgr_    = errorMgr;
            faulted_     = false;
            return true;
        }

        void SafetyManager::update()
        {
            // Example: read some flags from rtLayout_
            // e.g., auto &sf = rtLayout_->safetyFeedback;
            // if (sf.eStop || sf.overTemp || sf.limitExceeded)
            // {
            //     faulted_ = true;
            //     if (errorMgr_)
            //     {
            //         errorMgr_->reportError(200, "Safety fault: eStop or limit exceeded");
            //     }
            // }

            // Another approach: check paramServer_ for some dynamic threshold
        }

        bool SafetyManager::isFaulted() const
        {
            return faulted_;
        }

        void SafetyManager::clearFault()
        {
            faulted_ = false;
        }
    }
}
