#pragma once

#include <algorithm>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/Enums.h"
#include "control/controllers/BaseController.h"

namespace seven_axis_robot
{
    namespace control
    {
        static constexpr int MAX_CONTROLLERS = 10;

        enum class SwitchState
        {
            IDLE,
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
                const seven_axis_robot::merai::ParameterServer* paramServerPtr,
                seven_axis_robot::merai::JointMotionFeedback*    motionFeedbackPtr,
                seven_axis_robot::merai::JointMotionCommand*     motionCommandPtr,
                std::size_t                                  jointCount,
                seven_axis_robot::merai::multi_ring_logger_memory* loggerMem
            );

            ~ControllerManager();

            struct FallbackPolicy
            {
                enum class Behavior
                {
                    HoldPosition,
                    ZeroTorque
                };

                int modeOfOperation = 8;
                Behavior behavior = Behavior::HoldPosition;
                double torqueLimit = 0.0; // optional clamp in torque mode
            };

            struct ModeCompatibilityPolicy
            {
                std::vector<int> allowedModes;

                bool isCompatible(int candidateMode) const
                {
                    if (allowedModes.empty())
                    {
                        return true;
                    }
                    return std::find(allowedModes.begin(), allowedModes.end(), candidateMode) != allowedModes.end();
                }
            };

            struct ControllerRegistration
            {
                std::shared_ptr<BaseController> controller;
                int modeHint = 0;
            };

            /**
             * @brief Register a controller object for a specific ID.
             * @return True on success, false if invalid ID or a controller is
             *         already registered with that ID.
             */
            bool registerController(seven_axis_robot::merai::ControllerID id,
                                    std::shared_ptr<BaseController> controller,
                                    int modeHint);

            void setFallbackPolicy(const FallbackPolicy& policy);
            void setModeCompatibilityPolicy(const ModeCompatibilityPolicy& policy);

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
            void update(const seven_axis_robot::merai::ControllerCommand &cmd,
                        seven_axis_robot::merai::ControllerFeedback &feedback,
                        double dt);

        private:
            ControllerRegistration* findControllerById(seven_axis_robot::merai::ControllerID id);
            void applySafeFallback();
            bool isModeCompatible(int candidateMode) const;
            void populateFeedback(seven_axis_robot::merai::ControllerFeedback& feedback) const;
            seven_axis_robot::merai::ControllerFeedbackState toLegacyFeedbackState(
                seven_axis_robot::merai::ControllerSwitchResult result) const;

        private:
            const seven_axis_robot::merai::ParameterServer* paramServerPtr_ = nullptr;

            // Pointers to the motion-level data from HAL
            seven_axis_robot::merai::JointMotionFeedback* motionFeedbackPtr_ = nullptr;
            seven_axis_robot::merai::JointMotionCommand*  motionCommandPtr_  = nullptr;
            std::size_t jointCount_ = 0;
            seven_axis_robot::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            struct ControllerIdHash
            {
                size_t operator()(seven_axis_robot::merai::ControllerID id) const noexcept
                {
                    return static_cast<size_t>(id);
                }
            };

            // Mapping from ControllerID to controller
            std::unordered_map<seven_axis_robot::merai::ControllerID,
                               ControllerRegistration,
                               ControllerIdHash> registry_{};

            // The currently active controller
            std::shared_ptr<BaseController> active_controller_{nullptr};
            seven_axis_robot::merai::ControllerID activeControllerId_ = seven_axis_robot::merai::ControllerID::NONE;

            FallbackPolicy fallbackPolicy_{};
            ModeCompatibilityPolicy modePolicy_{};
            seven_axis_robot::merai::ControllerSwitchResult switchResult_{seven_axis_robot::merai::ControllerSwitchResult::IDLE};

            // Whether a switch was requested
            std::atomic<bool> switch_pending_{false};

            // The ID of the next desired controller
            seven_axis_robot::merai::ControllerID target_controller_id_ = seven_axis_robot::merai::ControllerID::NONE;
        };

    } // namespace control
} // namespace seven_axis_robot
