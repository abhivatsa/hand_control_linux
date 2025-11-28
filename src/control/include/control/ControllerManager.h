#pragma once

#include <algorithm>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <span>
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/Enums.h"
#include "control/controllers/BaseController.h"
#include "control/ControlData.h"

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
         * @param jointCount        Number of joints.
         */
        ControllerManager(
            const merai::ParameterServer *paramServerPtr,
            std::size_t jointCount,
            merai::multi_ring_logger_memory *loggerMem,
            double loopPeriodSec);

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
        bool registerController(merai::ControllerID id,
                                std::shared_ptr<BaseController> controller,
                                int modeHint);

        void setFallbackPolicy(const FallbackPolicy &policy);
        void setModeCompatibilityPolicy(const ModeCompatibilityPolicy &policy);

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
        void update(const ControlCycleInputs &in,
                    ControlCycleOutputs &out);

    private:
        ControllerRegistration *findControllerById(merai::ControllerID id);
        void applySafeFallback(std::span<const merai::JointMotionFeedback> motionFbk,
                               std::span<merai::JointMotionCommand> motionCmd);
        bool isModeCompatible(int candidateMode) const;
        void populateFeedback(merai::ControllerFeedback &feedback) const;
        merai::ControllerFeedbackState toLegacyFeedbackState(
            merai::ControllerSwitchResult result) const;

    private:
        const merai::ParameterServer *paramServerPtr_ = nullptr;

        std::size_t jointCount_ = 0;
        merai::multi_ring_logger_memory *loggerMem_ = nullptr;
        double defaultDtSec_{0.0};
        std::chrono::steady_clock::time_point lastTimestamp_{};
        bool hasLastTimestamp_{false};

        struct ControllerIdHash
        {
            size_t operator()(merai::ControllerID id) const noexcept
            {
                return static_cast<size_t>(id);
            }
        };

        // Mapping from ControllerID to controller
        std::unordered_map<merai::ControllerID,
                           ControllerRegistration,
                           ControllerIdHash>
            registry_{};

        // The currently active controller
        std::shared_ptr<BaseController> active_controller_{nullptr};
        merai::ControllerID activeControllerId_ = merai::ControllerID::NONE;

        FallbackPolicy fallbackPolicy_{};
        ModeCompatibilityPolicy modePolicy_{};
        merai::ControllerSwitchResult switchResult_{merai::ControllerSwitchResult::IDLE};

        // Whether a switch was requested
        std::atomic<bool> switch_pending_{false};

        // The ID of the next desired controller
        merai::ControllerID target_controller_id_ = merai::ControllerID::NONE;
    };

} // namespace control
