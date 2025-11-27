#include <stdexcept> // for std::runtime_error
#include <utility>
#include "control/ControllerManager.h"
#include "control/controllers/BaseController.h"

namespace seven_axis_robot
{
    namespace control
    {

        ControllerManager::ControllerManager(
            const seven_axis_robot::merai::ParameterServer *paramServerPtr,
            seven_axis_robot::merai::JointMotionFeedback *motionFeedbackPtr,
            seven_axis_robot::merai::JointMotionCommand *motionCommandPtr,
            std::size_t jointCount,
            seven_axis_robot::merai::multi_ring_logger_memory* loggerMem)
            : paramServerPtr_(paramServerPtr),
              motionFeedbackPtr_(motionFeedbackPtr),
              motionCommandPtr_(motionCommandPtr),
              jointCount_(jointCount),
              loggerMem_(loggerMem)
        {
            if (!paramServerPtr_)
            {
                throw std::runtime_error("ControllerManager: Null paramServer pointer.");
            }

            seven_axis_robot::merai::log_info(loggerMem_, "Control", 2100, "[ControllerManager] jointcount initialized");

            if (!motionFeedbackPtr_ || !motionCommandPtr_ || jointCount_ == 0)
            {
                throw std::runtime_error("ControllerManager: Invalid joint pointers or jointCount.");
            }

            // Default compatibility and fallback policies
            modePolicy_.allowedModes = {8, 10, -3};
        }

        ControllerManager::~ControllerManager()
        {
            // Cleanup if needed
        }

        bool ControllerManager::registerController(seven_axis_robot::merai::ControllerID id,
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

        void ControllerManager::setFallbackPolicy(const FallbackPolicy& policy)
        {
            fallbackPolicy_ = policy;
        }

        void ControllerManager::setModeCompatibilityPolicy(const ModeCompatibilityPolicy& policy)
        {
            modePolicy_ = policy;
        }

        bool ControllerManager::init()
        {
            // Validate pointers
            if (!paramServerPtr_ || !motionFeedbackPtr_ || !motionCommandPtr_ || jointCount_ == 0)
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
            activeControllerId_ = seven_axis_robot::merai::ControllerID::NONE;
            switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::IDLE;

            return success;
        }

        void ControllerManager::update(const seven_axis_robot::merai::ControllerCommand &cmd,
                                       seven_axis_robot::merai::ControllerFeedback &feedback,
                                       double dt)
        {
            if (cmd.requestSwitch)
            {
                target_controller_id_ = cmd.controllerId;
                switch_pending_.store(true);
            }

            if (switch_pending_.exchange(false))
            {
                switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::IN_PROGRESS;
                auto registration = findControllerById(target_controller_id_);
                if (!registration || !registration->controller)
                {
                    switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::FAILED;
                }
                else if (!isModeCompatible(registration->modeHint))
                {
                    switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::FAILED;
                }
                else if (registration->controller == active_controller_)
                {
                    activeControllerId_ = target_controller_id_;
                    switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::SUCCEEDED;
                }
                else
                {
                    if (active_controller_)
                    {
                        active_controller_->stop();
                    }
                    applySafeFallback();

                    if (!registration->controller->start())
                    {
                        active_controller_ = nullptr;
                        activeControllerId_ = seven_axis_robot::merai::ControllerID::NONE;
                        switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::FAILED;
                    }
                    else
                    {
                        active_controller_ = registration->controller;
                        activeControllerId_ = target_controller_id_;
                        switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::SUCCEEDED;
                    }
                }
            }

            if (active_controller_)
            {
                active_controller_->update(dt);
            }
            else
            {
                applySafeFallback();
                activeControllerId_ = seven_axis_robot::merai::ControllerID::NONE;
            }

            if (active_controller_ && switchResult_ == seven_axis_robot::merai::ControllerSwitchResult::IDLE)
            {
                switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::SUCCEEDED;
            }
            if (!active_controller_ && switchResult_ == seven_axis_robot::merai::ControllerSwitchResult::SUCCEEDED)
            {
                switchResult_ = seven_axis_robot::merai::ControllerSwitchResult::IDLE;
            }

            populateFeedback(feedback);
        }

        void ControllerManager::applySafeFallback()
        {
            for (std::size_t i = 0; i < jointCount_; ++i)
            {
                auto &cmd = motionCommandPtr_[i];
                const auto &fbk = motionFeedbackPtr_[i];

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

        ControllerManager::ControllerRegistration* ControllerManager::findControllerById(seven_axis_robot::merai::ControllerID id)
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

        void ControllerManager::populateFeedback(seven_axis_robot::merai::ControllerFeedback& feedback) const
        {
            feedback.activeControllerId = activeControllerId_;
            feedback.switchResult = switchResult_;
            feedback.feedbackState = toLegacyFeedbackState(switchResult_);
        }

        seven_axis_robot::merai::ControllerFeedbackState ControllerManager::toLegacyFeedbackState(
            seven_axis_robot::merai::ControllerSwitchResult result) const
        {
            using seven_axis_robot::merai::ControllerFeedbackState;
            using seven_axis_robot::merai::ControllerSwitchResult;

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
} // namespace seven_axis_robot
