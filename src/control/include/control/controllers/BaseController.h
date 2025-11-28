#pragma once

#include <span>
#include "merai/RTMemoryLayout.h"

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
     *        and a span-based update call.
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
         *  Provides access to the current feedback/command spans to allow
         *  controllers to set initial modes or targets.
         */
        virtual bool start(std::span<const merai::JointMotionFeedback> motionFbk,
                           std::span<merai::JointMotionCommand> motionCmd) = 0;

        /**
         * @brief update
         *  Main loop function, typically called each real-time cycle (e.g., 1ms).
         *
         * @param motionFbk  Span over current joint motion feedback.
         * @param motionCmd  Span over joint motion command outputs to fill.
         * @param dt         Timestep in seconds (e.g. 0.001 for 1 kHz).
         */
        virtual void update(std::span<const merai::JointMotionFeedback> motionFbk,
                            std::span<merai::JointMotionCommand> motionCmd,
                            double dt) = 0;

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
