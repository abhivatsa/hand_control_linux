#include <stdexcept> // for std::runtime_error
#include <utility>
#include "control/ControllerManager.h"
#include "control/controllers/BaseController.h"

namespace control
{

    ControllerManager::ControllerManager(
        const merai::ParameterServer *paramServerPtr,
        std::size_t jointCount,
        merai::multi_ring_logger_memory *loggerMem,
        double loopPeriodSec)
        : paramServerPtr_(paramServerPtr),
          jointCount_(jointCount),
          loggerMem_(loggerMem),
          defaultDtSec_(loopPeriodSec)
    {
        if (!paramServerPtr_)
        {
            throw std::runtime_error("ControllerManager: Null paramServer pointer.");
        }

        merai::log_info(loggerMem_, "Control", 2100, "[ControllerManager] jointcount initialized");

        if (jointCount_ == 0)
        {
            throw std::runtime_error("ControllerManager: Invalid jointCount.");
        }

        // Default compatibility and fallback policies
        modePolicy_.allowedModes = {8, 10, -3};
    }

    ControllerManager::~ControllerManager()
    {
        // Cleanup if needed
    }

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
        registration.modeHint = modeHint;

        auto result = registry_.emplace(id, std::move(registration));
        return result.second;
    }

    void ControllerManager::setFallbackPolicy(const FallbackPolicy &policy)
    {
        fallbackPolicy_ = policy;
    }

    void ControllerManager::setModeCompatibilityPolicy(const ModeCompatibilityPolicy &policy)
    {
        modePolicy_ = policy;
    }

    bool ControllerManager::init()
    {
        // Validate pointers
        if (!paramServerPtr_ || jointCount_ == 0)
        {
            return false;
        }

        bool success = true;

        // Initialize all registered controllers
        for (auto &entry : registry_)
        {
            if (entry.second.controller && !entry.second.controller->init())
            {
                success = false;
            }
        }

        // By default, we can stay with no active controller => fallback
        active_controller_ = nullptr;
        activeControllerId_ = merai::ControllerID::NONE;
        switchResult_ = merai::ControllerSwitchResult::IDLE;

        return success;
    }

    void ControllerManager::update(const ControlCycleInputs &in,
                                   ControlCycleOutputs &out)
    {
        double dt = defaultDtSec_;
        if (hasLastTimestamp_)
        {
            dt = std::chrono::duration<double>(in.timestamp - lastTimestamp_).count();
        }
        lastTimestamp_ = in.timestamp;
        hasLastTimestamp_ = true;

        if (in.ctrlCmd.requestSwitch)
        {
            target_controller_id_ = in.ctrlCmd.controllerId;
            switch_pending_.store(true);
        }

        if (switch_pending_.exchange(false))
        {
            switchResult_ = merai::ControllerSwitchResult::IN_PROGRESS;
            auto registration = findControllerById(target_controller_id_);
            if (!registration || !registration->controller)
            {
                switchResult_ = merai::ControllerSwitchResult::FAILED;
            }
            else if (!isModeCompatible(registration->modeHint))
            {
                switchResult_ = merai::ControllerSwitchResult::FAILED;
            }
            else if (registration->controller == active_controller_)
            {
                activeControllerId_ = target_controller_id_;
                switchResult_ = merai::ControllerSwitchResult::SUCCEEDED;
            }
            else
            {
                if (active_controller_)
                {
                    active_controller_->stop();
                }
                applySafeFallback(in.jointMotionFbk, out.jointMotionCmd);

                if (!registration->controller->start(in.jointMotionFbk, out.jointMotionCmd))
                {
                    active_controller_ = nullptr;
                    activeControllerId_ = merai::ControllerID::NONE;
                    switchResult_ = merai::ControllerSwitchResult::FAILED;
                }
                else
                {
                    active_controller_ = registration->controller;
                    activeControllerId_ = target_controller_id_;
                    switchResult_ = merai::ControllerSwitchResult::SUCCEEDED;
                }
            }
        }

        if (active_controller_)
        {
            active_controller_->update(in.jointMotionFbk, out.jointMotionCmd, dt);
        }
        else
        {
            applySafeFallback(in.jointMotionFbk, out.jointMotionCmd);
            activeControllerId_ = merai::ControllerID::NONE;
        }

        if (active_controller_ && switchResult_ == merai::ControllerSwitchResult::IDLE)
        {
            switchResult_ = merai::ControllerSwitchResult::SUCCEEDED;
        }
        if (!active_controller_ && switchResult_ == merai::ControllerSwitchResult::SUCCEEDED)
        {
            switchResult_ = merai::ControllerSwitchResult::IDLE;
        }

        populateFeedback(out.ctrlFbk);
    }

    void ControllerManager::applySafeFallback(std::span<const merai::JointMotionFeedback> motionFbk,
                                              std::span<merai::JointMotionCommand> motionCmd)
    {
        for (std::size_t i = 0; i < jointCount_; ++i)
        {
            if (i >= motionCmd.size() || i >= motionFbk.size())
            {
                continue;
            }
            auto &cmd = motionCmd[i];
            const auto &fbk = motionFbk[i];

            cmd.modeOfOperation = static_cast<uint8_t>(fallbackPolicy_.modeOfOperation);
            cmd.targetPosition = fbk.positionActual;
            cmd.targetTorque = 0.0;

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

    ControllerManager::ControllerRegistration *ControllerManager::findControllerById(merai::ControllerID id)
    {
        auto it = registry_.find(id);
        if (it == registry_.end())
        {
            return nullptr;
        }
        return &it->second;
    }

    bool ControllerManager::isModeCompatible(int candidateMode) const
    {
        return modePolicy_.isCompatible(candidateMode);
    }

    void ControllerManager::populateFeedback(merai::ControllerFeedback &feedback) const
    {
        feedback.activeControllerId = activeControllerId_;
        feedback.switchResult = switchResult_;
        feedback.feedbackState = toLegacyFeedbackState(switchResult_);
    }

    merai::ControllerFeedbackState ControllerManager::toLegacyFeedbackState(
        merai::ControllerSwitchResult result) const
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
