#include <stdexcept> // for std::runtime_error
#include "control/ControllerManager.h"

namespace hand_control
{
    namespace control
    {
        ControllerManager::ControllerManager(
            const hand_control::merai::ParameterServer* paramServerPtr,
            hand_control::merai::JointState* jointStatesPtr,
            hand_control::merai::JointCommand* jointCommandsPtr,
            std::size_t jointCount)
            : paramServerPtr_(paramServerPtr),
              jointStatesPtr_(jointStatesPtr),
              jointCommandsPtr_(jointCommandsPtr),
              jointCount_(jointCount)
        {
            if (!paramServerPtr_)
            {
                throw std::runtime_error("ControllerManager: Null paramServer pointer.");
            }
        }

        ControllerManager::~ControllerManager()
        {
            // Cleanup if needed
        }

        bool ControllerManager::registerController(hand_control::merai::ControllerID id,
                                                   std::shared_ptr<BaseController> controller)
        {
            if (!controller)
            {
                return false;
            }

            // Convert the ID to an index for the array
            int idx = static_cast<int>(id);
            if (idx < 0 || idx >= static_cast<int>(idToController_.size()))
            {
                // out of range
                return false;
            }

            if (idToController_[idx] != nullptr)
            {
                // Already assigned a controller to that ID
                return false;
            }

            idToController_[idx] = controller;
            return true;
        }

        bool ControllerManager::init()
        {
            // Validate pointers
            if (!paramServerPtr_ || !jointStatesPtr_ || !jointCommandsPtr_ || jointCount_ == 0)
            {
                return false;
            }

            bool success = true;

            // Initialize all registered controllers
            for (auto& ctrlPtr : idToController_)
            {
                if (ctrlPtr && !ctrlPtr->init())
                {
                    success = false;
                }
            }

            // By default, we can stay with no active controller => fallback
            active_controller_ = nullptr;
            switchState_ = SwitchState::IDLE;

            return success;
        }

        void ControllerManager::update(const hand_control::merai::ControllerCommand &cmd,
                                       hand_control::merai::ControllerFeedback &feedback,
                                       double dt)
        {
            // If user requests a switch
            if (cmd.requestSwitch)
            {
                target_controller_id_ = cmd.controllerId;
                switch_pending_.store(true);
            }

            // (1) Handle bridging or switching logic
            processControllerSwitch();

            // (2) Run the active controller (Approach B => controller->update(dt))
            if (active_controller_)
            {
                active_controller_->update(dt);
            }
            else
            {
                // Fallback: hold position if no controller is active
                for (std::size_t i = 0; i < jointCount_; ++i)
                {
                    jointCommandsPtr_[i].position = jointStatesPtr_[i].position;
                    jointCommandsPtr_[i].velocity = 0.0;
                    jointCommandsPtr_[i].torque   = 0.0;
                }
            }

            // (3) Fill out the feedback struct
            bool bridging   = (switchState_ == SwitchState::BRIDGING);
            bool switching  = (switchState_ != SwitchState::RUNNING);

            feedback.feedbackState = hand_control::merai::ControllerFeedbackState::SWITCH_IN_PROGRESS;

            // feedback.feedbackState.bridgingActive   = bridging;
            // feedback.switchInProgress = switching;
            // feedback.controllerFailed = false; // Set true if you detect an error
        }

        void ControllerManager::processControllerSwitch()
        {
            switch (switchState_)
            {
            case SwitchState::IDLE:
                if (switch_pending_.load())
                {
                    switch_pending_.store(false);
                    oldController_ = active_controller_;
                    newController_ = findControllerById(target_controller_id_);
                    if (!newController_)
                    {
                        // handle error: e.g., invalid ID
                        break;
                    }
                    bridgingNeeded_ = requiresBridging(oldController_.get(), newController_.get());
                    switchState_ = SwitchState::STOP_OLD;
                }
                break;

            case SwitchState::STOP_OLD:
                if (oldController_)
                {
                    oldController_->stop();
                }
                if (bridgingNeeded_)
                {
                    switchState_ = SwitchState::BRIDGING;
                }
                else
                {
                    switchState_ = SwitchState::START_NEW;
                }
                break;

            case SwitchState::BRIDGING:
                // bridging logic if needed
                switchState_ = SwitchState::START_NEW;
                break;

            case SwitchState::START_NEW:
                if (newController_)
                {
                    newController_->start();
                    active_controller_ = newController_;
                }
                switchState_ = SwitchState::RUNNING;
                break;

            case SwitchState::RUNNING:
                if (switch_pending_.load())
                {
                    switch_pending_.store(false);
                    oldController_ = active_controller_;
                    newController_ = findControllerById(target_controller_id_);
                    if (!newController_)
                    {
                        // handle error
                        break;
                    }
                    bridgingNeeded_ = requiresBridging(oldController_.get(), newController_.get());
                    switchState_ = SwitchState::STOP_OLD;
                }
                break;
            }
        }

        bool ControllerManager::requiresBridging(BaseController* oldCtrl, BaseController* newCtrl)
        {
            if (!oldCtrl || !newCtrl)
            {
                return false;
            }
            // e.g., if old is torque-based and new is position-based => bridging needed
            return false;
        }

        std::shared_ptr<BaseController> ControllerManager::findControllerById(hand_control::merai::ControllerID id)
        {
            int idx = static_cast<int>(id);
            if (idx < 0 || idx >= static_cast<int>(idToController_.size()))
            {
                return nullptr;
            }
            return idToController_[idx];
        }

    } // namespace control
} // namespace hand_control
