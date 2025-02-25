#pragma once

#include "control/controllers/BaseController.h"

namespace motion_control
{
    namespace control
    {
        class BaseBridgingController : public BaseController
        {
        public:
            virtual ~BaseBridgingController() = default;

            /**
             * @brief Indicates whether bridging is complete.
             * @return True if bridging is done, false otherwise.
             */
            virtual bool isDone() const = 0;

            // More bridging-specific APIs can go here...
        };
    } // namespace control
} // namespace motion_control
