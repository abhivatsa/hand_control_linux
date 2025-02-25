#pragma once

#include <string>
#include "merai/RTMemoryLayout.h" // for JointState, JointCommand

namespace motion_control
{
    namespace control
    {
        enum class ControllerState
        {
            UNINIT,
            INIT,
            RUNNING,
            STOPPED
        };

        /**
         * @brief BaseController is an interface for all controllers, providing
         *        lifecycle methods (init, start, stop, etc.) and an update step.
         *
         * If JointState / JointCommand are defined in, say, motion_control::merai,
         * we reference them fully. Adjust as needed if they live elsewhere.
         */
        class BaseController
        {
        public:
            virtual ~BaseController() = default;

            virtual bool init(const std::string& controllerName) = 0;
            virtual void start() = 0;

            // Use fully qualified references if JointState / JointCommand
            // come from motion_control::merai namespace. Adjust as needed.
            virtual void update(const motion_control::merai::JointState* states,
                                motion_control::merai::JointCommand* commands,
                                int numJoints,
                                double dt) = 0;

            virtual void stop() = 0;
            virtual void teardown() = 0;

            ControllerState getState() const
            {
                return state_;
            }

            std::string name() const
            {
                return name_;
            }

        protected:
            std::string name_;
            ControllerState state_{ControllerState::UNINIT};
        };
    } // namespace control
} // namespace motion_control
