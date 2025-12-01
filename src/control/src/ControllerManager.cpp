#include <stdexcept>

#include "control/ControllerManager.h"

namespace control
{
    ControllerManager::ControllerManager(std::size_t jointCount,
                                         merai::multi_ring_logger_memory *loggerMem)
        : jointCount_(jointCount),
          loggerMem_(loggerMem)
    {
        if (jointCount_ == 0)
        {
            throw std::runtime_error("ControllerManager: Invalid jointCount.");
        }

        if (loggerMem_)
        {
            merai::log_info(loggerMem_, "Control", 2100,
                            "[ControllerManager] jointCount initialized");
        }
    }

    ControllerManager::~ControllerManager() = default;

    bool ControllerManager::registerController(merai::ControllerID id,
                                               std::shared_ptr<BaseController> controller,
                                               int modeHint)
    {
        if (!controller)
        {
            return false;
        }

        ControllerRegistration registration{};
        registration.controller = std::move(controller);
        registration.modeHint   = modeHint;

        auto result = registry_.emplace(id, std::move(registration));
        return result.second;
    }

    void ControllerManager::setFallbackPolicy(const FallbackPolicy &policy)
    {
        fallbackPolicy_ = policy;
    }

    bool ControllerManager::init()
    {
        if (jointCount_ == 0)
        {
            return false;
        }

        bool success = true;

        for (auto &entry : registry_)
        {
            if (entry.second.controller && !entry.second.controller->init())
            {
                success = false;
            }
        }

        active_controller_    = nullptr;
        activeControllerId_   = merai::ControllerID::NONE;
        switchResult_         = merai::ControllerSwitchResult::IDLE;
        switchPhase_          = SwitchPhase::NONE;
        pendingController_    = nullptr;
        pendingControllerId_  = merai::ControllerID::NONE;

        return success;
    }

    void ControllerManager::update(const ControlCycleInputs &in,
                                   ControlCycleOutputs       &out)
    {
        constexpr double dt = 1.0 / 1000.0; // fixed 1 kHz

        if (in.ctrlCmd.requestSwitch)
        {
            handleSwitchCommand(in.ctrlCmd);
        }

        advanceSwitchStateMachine(in, out);
        runActiveController(in, out, dt);
        populateFeedback(out.ctrlFbk);
    }

    void ControllerManager::applySafeFallback(std::span<const merai::JointMotionFeedback> motionFbk,
                                              std::span<merai::JointMotionCommand>       motionCmd)
    {
        for (std::size_t i = 0; i < jointCount_; ++i)
        {
            if (i >= motionCmd.size() || i >= motionFbk.size())
            {
                continue;
            }

            auto       &cmd = motionCmd[i];
            const auto &fbk = motionFbk[i];

            cmd.modeOfOperation = static_cast<std::uint8_t>(fallbackPolicy_.modeOfOperation);
            cmd.targetPosition  = fbk.positionActual;
            cmd.targetTorque    = 0.0;

            if (fallbackPolicy_.behavior == FallbackPolicy::Behavior::ZeroTorque)
            {
                cmd.targetTorque = 0.0;
            }

            if (fallbackPolicy_.torqueLimit > 0.0)
            {
                if (cmd.targetTorque > fallbackPolicy_.torqueLimit)
                    cmd.targetTorque = fallbackPolicy_.torqueLimit;
                if (cmd.targetTorque < -fallbackPolicy_.torqueLimit)
                    cmd.targetTorque = -fallbackPolicy_.torqueLimit;
            }
        }
    }

    ControllerManager::ControllerRegistration *
    ControllerManager::findControllerById(merai::ControllerID id)
    {
        auto it = registry_.find(id);
        if (it == registry_.end())
        {
            return nullptr;
        }
        return &it->second;
    }

    void ControllerManager::handleSwitchCommand(const merai::ControllerCommand &cmd)
    {
        if (switchPhase_ != SwitchPhase::NONE)
        {
            return;
        }

        if (cmd.controllerId == activeControllerId_)
        {
            switchResult_ = merai::ControllerSwitchResult::SUCCEEDED;
            return;
        }

        auto registration = findControllerById(cmd.controllerId);
        if (!registration || !registration->controller)
        {
            switchResult_       = merai::ControllerSwitchResult::FAILED;
            pendingController_  = nullptr;
            pendingControllerId_ = merai::ControllerID::NONE;
            return;
        }

        pendingController_   = registration->controller.get();
        pendingControllerId_ = cmd.controllerId;
        switchResult_        = merai::ControllerSwitchResult::IN_PROGRESS;

        if (active_controller_)
        {
            active_controller_->requestStop();
            switchPhase_ = SwitchPhase::STOPPING_OLD;
        }
        else
        {
            switchPhase_ = SwitchPhase::STARTING_NEW;
        }
    }

    void ControllerManager::advanceSwitchStateMachine(const ControlCycleInputs &in,
                                                      ControlCycleOutputs       &out)
    {
        switch (switchPhase_)
        {
        case SwitchPhase::NONE:
            if (switchResult_ == merai::ControllerSwitchResult::SUCCEEDED)
            {
                switchResult_ = merai::ControllerSwitchResult::IDLE;
            }
            return;

        case SwitchPhase::STOPPING_OLD:
            if (!active_controller_)
            {
                switchPhase_ = SwitchPhase::STARTING_NEW;
                break;
            }
            else
            {
                const auto state = active_controller_->state();
                if (state == ControllerState::STOPPED ||
                    state == ControllerState::ERROR)
                {
                    active_controller_  = nullptr;
                    activeControllerId_ = merai::ControllerID::NONE;
                    switchPhase_        = SwitchPhase::STARTING_NEW;
                }
            }
            break;

        case SwitchPhase::STARTING_NEW:
            startPendingController(in, out);

            if (switchPhase_ != SwitchPhase::STARTING_NEW)
            {
                break;
            }

            if (active_controller_)
            {
                const auto state = active_controller_->state();

                if (state == ControllerState::RUNNING)
                {
                    switchPhase_        = SwitchPhase::NONE;
                    switchResult_       = merai::ControllerSwitchResult::SUCCEEDED;
                    pendingController_  = nullptr;
                    pendingControllerId_ = merai::ControllerID::NONE;
                }
                else if (state == ControllerState::ERROR)
                {
                    switchPhase_        = SwitchPhase::NONE;
                    switchResult_       = merai::ControllerSwitchResult::FAILED;

                    pendingController_   = nullptr;
                    pendingControllerId_ = merai::ControllerID::NONE;
                    active_controller_   = nullptr;
                    activeControllerId_  = merai::ControllerID::NONE;

                    applySafeFallback(in.jointMotionFbk, out.jointMotionCmd);
                }
            }
            break;
        }
    }

    void ControllerManager::startPendingController(const ControlCycleInputs &in,
                                                   ControlCycleOutputs       &out)
    {
        if (!pendingController_)
        {
            switchResult_        = merai::ControllerSwitchResult::FAILED;
            switchPhase_         = SwitchPhase::NONE;
            pendingControllerId_ = merai::ControllerID::NONE;
            return;
        }

        if (active_controller_ == pendingController_)
        {
            return;
        }

        if (!pendingController_->start(in.jointMotionFbk, out.jointMotionCmd))
        {
            switchResult_        = merai::ControllerSwitchResult::FAILED;
            switchPhase_         = SwitchPhase::NONE;

            pendingController_   = nullptr;
            pendingControllerId_ = merai::ControllerID::NONE;

            active_controller_   = nullptr;
            activeControllerId_  = merai::ControllerID::NONE;

            applySafeFallback(in.jointMotionFbk, out.jointMotionCmd);
            return;
        }

        active_controller_   = pendingController_;
        activeControllerId_  = pendingControllerId_;
    }

    void ControllerManager::runActiveController(const ControlCycleInputs &in,
                                                ControlCycleOutputs       &out,
                                                double                    dt)
    {
        if (!active_controller_)
        {
            applySafeFallback(in.jointMotionFbk, out.jointMotionCmd);
            activeControllerId_ = merai::ControllerID::NONE;
            return;
        }

        const auto state = active_controller_->state();

        if (state == ControllerState::STOPPED ||
            state == ControllerState::ERROR)
        {
            applySafeFallback(in.jointMotionFbk, out.jointMotionCmd);
            active_controller_   = nullptr;
            activeControllerId_  = merai::ControllerID::NONE;
            return;
        }

        active_controller_->update(in.jointMotionFbk, out.jointMotionCmd, dt);
    }

    void ControllerManager::populateFeedback(merai::ControllerFeedback &feedback) const
    {
        feedback.activeControllerId = activeControllerId_;
        feedback.switchResult       = switchResult_;
        feedback.feedbackState      = toLegacyFeedbackState(switchResult_);
    }

    merai::ControllerFeedbackState
    ControllerManager::toLegacyFeedbackState(merai::ControllerSwitchResult result) const
    {
        using merai::ControllerFeedbackState;
        using merai::ControllerSwitchResult;

        switch (result)
        {
        case ControllerSwitchResult::IN_PROGRESS:
            return ControllerFeedbackState::SWITCH_IN_PROGRESS;
        case ControllerSwitchResult::SUCCEEDED:
            return ControllerFeedbackState::SWITCH_COMPLETED;
        case ControllerSwitchResult::FAILED:
            return ControllerFeedbackState::SWITCH_FAILED;
        case ControllerSwitchResult::IDLE:
        default:
            return ControllerFeedbackState::IDLE;
        }
    }

} // namespace control
