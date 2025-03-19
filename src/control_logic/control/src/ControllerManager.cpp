#include <stdexcept>    // for std::runtime_error
#include <iostream>     // for std::cerr, std::endl
#include "control/ControllerManager.h"
#include "control/controllers/BaseController.h"

namespace hand_control
{
    namespace control
    {
        using namespace hand_control::merai;

        ControllerManager::ControllerManager(
            const ParameterServer* paramServerPtr,
            JointMotionFeedback*   motionFeedbackPtr,
            JointMotionCommand*    motionCommandPtr,
            std::size_t            jointCount
        )
            : paramServerPtr_(paramServerPtr),
              motionFeedbackPtr_(motionFeedbackPtr),
              motionCommandPtr_(motionCommandPtr),
              jointCount_(jointCount)
        {
            if (!paramServerPtr_)
            {
                throw std::runtime_error("ControllerManager: Null paramServer pointer.");
            }
            if (!motionFeedbackPtr_ || !motionCommandPtr_ || jointCount_ == 0)
            {
                throw std::runtime_error("ControllerManager: Invalid joint pointers or jointCount.");
            }
        }

        ControllerManager::~ControllerManager()
        {
            // Cleanup if needed
        }

        bool ControllerManager::registerController(ControllerID id, std::shared_ptr<BaseController> controller)
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
            if (!paramServerPtr_ || !motionFeedbackPtr_ || !motionCommandPtr_ || jointCount_ == 0)
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
            switchState_        = SwitchState::IDLE;

            return success;
        }

        void ControllerManager::update(const ControllerCommand &cmd,
                                       ControllerFeedback &feedback,
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
                // Fallback: hold current position if no controller is active
                for (std::size_t i = 0; i < jointCount_; ++i)
                {
                    // Copy actual position from the motion feedback into the motion command
                    // so the joint doesn't move
                    motionCommandPtr_[i].targetPosition =
                        motionFeedbackPtr_[i].positionActual;

                    // Zero out torque if you're in position control by default
                    motionCommandPtr_[i].targetTorque = 0.0;
                    // If you want to specify modeOfOperation, set it here:
                    // motionCommandPtr_[i].modeOfOperation = ...
                }
            }

            // (3) Fill out the feedback struct
            bool bridging  = (switchState_ == SwitchState::BRIDGING);
            bool switching = (switchState_ != SwitchState::RUNNING);

            feedback.feedbackState = ControllerFeedbackState::SWITCH_IN_PROGRESS;
            // e.g. you can track bridging / switching states in more detail if you want
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
                    switchState_    = SwitchState::STOP_OLD;
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
                    switchState_    = SwitchState::STOP_OLD;
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

        std::shared_ptr<BaseController> ControllerManager::findControllerById(ControllerID id)
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
