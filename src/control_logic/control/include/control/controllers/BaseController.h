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
         * @brief BaseController is an abstract interface for controllers,
         *        providing lifecycle methods (init, start, stop, teardown)
         *        and a simplified update(dt) call.
         *
         * Derived classes that use "Approach B" will store pointers to
         * JointState, JointCommand, etc. in their constructor or init().
         */
        class BaseController
        {
        public:
            virtual ~BaseController() = default;

            /**
             * @brief init
             *  Called once before the controller is used. Typically used to
             *  validate pointers, load parameters, etc.
             *  Must return true if initialization succeeds, false otherwise.
             */
            virtual bool init() = 0;

            /**
             * @brief start
             *  Called when transitioning from another controller to this one.
             *  Typically sets internal state_ to RUNNING if in INIT or STOPPED.
             */
            virtual void start() = 0;

            /**
             * @brief update
             *  Main loop function, typically called each real-time cycle (e.g., 1ms).
             *  The derived class is expected to have references/pointers to the
             *  needed joint data from the constructor or init() stage, so we only
             *  need dt here.
             *
             * @param dt Timestep in seconds (e.g. 0.001 for 1 kHz).
             */
            virtual void update(double dt) = 0;

            /**
             * @brief stop
             *  Called when switching away from this controller to a new one.
             *  Typically sets internal state_ to STOPPED if running.
             */
            virtual void stop() = 0;

            /**
             * @brief teardown
             *  Called if the controller is being permanently removed or for final cleanup.
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
