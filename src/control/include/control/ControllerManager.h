#pragma once

#include <memory>
#include <unordered_map>
#include <span>

#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/Enums.h"

#include "control/controllers/BaseController.h"
#include "control/ControlData.h"

namespace control
{
    static constexpr int MAX_CONTROLLERS = 10;

    /**
     * @brief Manages the set of controllers and handles safe switching between them.
     *
     * - Keeps a registry of controllers keyed by merai::ControllerID.
     * - Tracks a single active controller.
     * - On switch request: stop old controller, start new, and report status.
     * - When inactive or in error, applies a safe fallback policy.
     */
    class ControllerManager
    {
    public:
        ControllerManager(std::size_t jointCount,
                          merai::multi_ring_logger_memory *loggerMem);

        ~ControllerManager();

        struct FallbackPolicy
        {
            enum class Behavior
            {
                HoldPosition,
                ZeroTorque
            };

            int     modeOfOperation = 8;
            Behavior behavior        = Behavior::HoldPosition;
            double  torqueLimit      = 0.0; // optional clamp in torque mode
        };

        struct ControllerRegistration
        {
            std::shared_ptr<BaseController> controller;
            [[maybe_unused]] int modeHint = 0;
        };

        bool registerController(merai::ControllerID id,
                                std::shared_ptr<BaseController> controller,
                                int modeHint);

        void setFallbackPolicy(const FallbackPolicy &policy);

        bool init();

        void update(const ControlCycleInputs &in,
                    ControlCycleOutputs       &out);

    private:
        struct ControllerIdHash
        {
            std::size_t operator()(merai::ControllerID id) const noexcept
            {
                return static_cast<std::size_t>(id);
            }
        };

        enum class SwitchPhase
        {
            NONE,
            STOPPING_OLD,
            STARTING_NEW
        };

        ControllerRegistration *findControllerById(merai::ControllerID id);

        void applySafeFallback(std::span<const merai::JointMotionFeedback> motionFbk,
                               std::span<merai::JointMotionCommand> motionCmd);

        void populateFeedback(merai::ControllerFeedback &feedback) const;

        merai::ControllerFeedbackState toLegacyFeedbackState(
            merai::ControllerSwitchResult result) const;

        void handleSwitchCommand(const merai::ControllerCommand &cmd);

        void advanceSwitchStateMachine(const ControlCycleInputs &in,
                                       ControlCycleOutputs       &out);

        void startPendingController(const ControlCycleInputs &in,
                                    ControlCycleOutputs       &out);

        void runActiveController(const ControlCycleInputs &in,
                                 ControlCycleOutputs       &out,
                                 double                    dt);

    private:
        std::size_t                     jointCount_ = 0;
        merai::multi_ring_logger_memory *loggerMem_ = nullptr;

        std::unordered_map<merai::ControllerID,
                           ControllerRegistration,
                           ControllerIdHash> registry_{};

        BaseController        *active_controller_   = nullptr;
        merai::ControllerID    activeControllerId_  = merai::ControllerID::NONE;

        FallbackPolicy         fallbackPolicy_{};

        merai::ControllerSwitchResult switchResult_{merai::ControllerSwitchResult::IDLE};

        SwitchPhase            switchPhase_{SwitchPhase::NONE};
        BaseController        *pendingController_   = nullptr;
        merai::ControllerID    pendingControllerId_ = merai::ControllerID::NONE;
    };

} // namespace control
