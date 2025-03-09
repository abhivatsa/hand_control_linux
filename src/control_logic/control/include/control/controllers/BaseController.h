#pragma once

#include "merai/RTMemoryLayout.h" // for JointState, JointCommand

namespace hand_control
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
         * If JointState / JointCommand are defined in, say, hand_control::merai,
         * we reference them fully. Adjust as needed if they live elsewhere.
         *
         * This version removes all std::string usage to avoid dynamic allocations.
         */
        class BaseController
        {
        public:
            virtual ~BaseController() = default;

            /**
             * @brief init
             *  A parameterless init for real-time safe code (no string usage).
             */
            virtual bool init() = 0;

            /**
             * @brief start
             *  Called when transitioning from another controller to this one.
             */
            virtual void start() = 0;

            /**
             * @brief update
             *  Main loop function, typically called each real-time cycle (e.g., 1ms).
             *
             * @param states    Pointer to array of JointState
             * @param commands  Pointer to array of JointCommand
             * @param numJoints Number of joints
             * @param dt        Timestep in seconds
             */
            virtual void update(const hand_control::merai::JointState* states,
                                hand_control::merai::JointCommand* commands,
                                int numJoints,
                                double dt) = 0;

            /**
             * @brief stop
             *  Called when switching away from this controller to a new one.
             */
            virtual void stop() = 0;

            /**
             * @brief teardown
             *  Called if the controller is being permanently removed or for cleanup.
             */
            virtual void teardown() = 0;

            /**
             * @brief getState
             *  Returns the current internal state (UNINIT, INIT, RUNNING, STOPPED).
             */
            ControllerState getState() const
            {
                return state_;
            }

        protected:
            ControllerState state_{ControllerState::UNINIT};
        };

    } // namespace control
} // namespace hand_control
