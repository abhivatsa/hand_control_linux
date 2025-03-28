#pragma once

#include <atomic>
#include <memory>
#include <array>
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/Enums.h"
#include "control/controllers/BaseController.h"

namespace hand_control
{
    namespace control
    {
        static constexpr int MAX_CONTROLLERS = 10;

        enum class SwitchState
        {
            IDLE,
            STOP_OLD,
            BRIDGING,
            START_NEW,
            RUNNING
        };

        /**
         * @brief Manages multiple controllers, allowing for real-time switching.
         *        Uses a mapping from ControllerID -> specific controller pointer.
         *
         * In the current approach:
         *   - Each controller stores its own joint pointers (if needed).
         *   - We only call `controller->update(dt)` each cycle if it's active.
         *   - Manager keeps a pointer to the joint data for fallback logic (no active controller).
         */
        class ControllerManager
        {
        public:
            /**
             * @brief Constructor
             *
             * @param paramServerPtr    Pointer to ParameterServer for config (must not be null).
             * @param motionFeedbackPtr Pointer to array of JointMotionFeedback (from HAL).
             * @param motionCommandPtr  Pointer to array of JointMotionCommand (from HAL).
             * @param jointCount        Number of joints.
             */
            ControllerManager(
                const hand_control::merai::ParameterServer* paramServerPtr,
                hand_control::merai::JointMotionFeedback*    motionFeedbackPtr,
                hand_control::merai::JointMotionCommand*     motionCommandPtr,
                std::size_t                                  jointCount
            );

            ~ControllerManager();

            /**
             * @brief Register a controller object for a specific ID.
             * @return True on success, false if invalid ID or a controller is
             *         already registered with that ID.
             */
            bool registerController(hand_control::merai::ControllerID id,
                                    std::shared_ptr<BaseController> controller);

            /**
             * @brief Initialize all registered controllers.
             * @return True if all controllers initialized OK, false otherwise.
             */
            bool init();

            /**
             * @brief The main update function called in the real-time loop.
             *        Checks if the user requested a controller switch, processes bridging,
             *        then runs the currently active controller with `update(dt)`.
             *        If no controller is active, runs fallback logic.
             *
             * @param cmd      A single ControllerCommand struct (requestSwitch, controllerId).
             * @param feedback A single ControllerFeedback struct for bridging/switch info.
             * @param dt       The time step in seconds (e.g., 0.001 for 1kHz).
             */
            void update(const hand_control::merai::ControllerCommand &cmd,
                        hand_control::merai::ControllerFeedback &feedback,
                        double dt);

        private:
            void processControllerSwitch();
            bool requiresBridging(BaseController* oldCtrl, BaseController* newCtrl);
            std::shared_ptr<BaseController> findControllerById(hand_control::merai::ControllerID id);

        private:
            const hand_control::merai::ParameterServer* paramServerPtr_ = nullptr;

            // Pointers to the motion-level data from HAL
            hand_control::merai::JointMotionFeedback* motionFeedbackPtr_ = nullptr;
            hand_control::merai::JointMotionCommand*  motionCommandPtr_  = nullptr;
            std::size_t jointCount_ = 0;

            // Mapping from ControllerID to controller
            std::array<std::shared_ptr<BaseController>,
                       static_cast<int>(hand_control::merai::ControllerID::E_STOP) + 1> idToController_ {};

            // The currently active controller
            std::shared_ptr<BaseController> active_controller_{nullptr};

            // Switch state machine
            SwitchState switchState_{SwitchState::IDLE};
            bool bridgingNeeded_{false};

            // The old and new controllers used during a switch
            std::shared_ptr<BaseController> oldController_{nullptr};
            std::shared_ptr<BaseController> newController_{nullptr};

            // Whether a switch was requested
            std::atomic<bool> switch_pending_{false};

            // The ID of the next desired controller
            hand_control::merai::ControllerID target_controller_id_ = hand_control::merai::ControllerID::NONE;
        };

    } // namespace control
} // namespace hand_control
