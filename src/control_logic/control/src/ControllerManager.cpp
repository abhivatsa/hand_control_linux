#include <stdexcept> // for std::runtime_error
#include <iostream>  // for std::cerr, std::endl
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
            std::size_t jointCount)
            : paramServerPtr_(paramServerPtr),
              motionFeedbackPtr_(motionFeedbackPtr),
              motionCommandPtr_(motionCommandPtr),
              jointCount_(jointCount)
        {
            if (!paramServerPtr_)
            {
                throw std::runtime_error("ControllerManager: Null paramServer pointer.");
            }

            std::cout<<"jointcount : "<<jointCount_<<std::endl;

            if (!motionFeedbackPtr_ || !motionCommandPtr_ || jointCount_ == 0)
            {
                throw std::runtime_error("ControllerManager: Invalid joint pointers or jointCount.");
            }
        }

        ControllerManager::~ControllerManager()
        {
            // Cleanup if needed
        }

        bool ControllerManager::registerController(seven_axis_robot::merai::ControllerID id, std::shared_ptr<BaseController> controller)
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
            for (auto &ctrlPtr : idToController_)
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

        void ControllerManager::update(const seven_axis_robot::merai::ControllerCommand &cmd,
                                       seven_axis_robot::merai::ControllerFeedback &feedback,
                                       double dt)
        {
            // If user requests a switch
            // std::cout<<"request Switch : "<<cmd.requestSwitch<<std::endl;
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
                std::cout<<"running active controller"<<std::endl;
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
                    motionCommandPtr_[i].modeOfOperation = 8;
                }
            }

            // (3) Fill out the feedback struct
            bool bridging = (switchState_ == SwitchState::BRIDGING);
            bool switching = !((switchState_ == SwitchState::RUNNING) || (switchState_ == SwitchState::IDLE));

            // std::cout<<"bridging : "<<bridging<<", switching : "<<switching<<std::endl;

            if (!switching){
                feedback.feedbackState = seven_axis_robot::merai::ControllerFeedbackState::SWITCH_COMPLETED;
            }
            else{
                feedback.feedbackState = seven_axis_robot::merai::ControllerFeedbackState::SWITCH_IN_PROGRESS;
            }

            
            // e.g. you can track bridging / switching states in more detail if you want
        }

        void ControllerManager::processControllerSwitch()
        {
            switch (switchState_)
            {
            case SwitchState::IDLE:

            // std::cout<<"inside Idle , switch pending status : "<<switch_pending_.load()<<std::endl;
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

            // std::cout<<"stop old "<<std::endl;
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
            // std::cout<<"bridging "<<std::endl;
                // bridging logic if needed
                switchState_ = SwitchState::START_NEW;
                break;

            case SwitchState::START_NEW:
            // std::cout<<"start new"<<std::endl;
                if (newController_)
                {
                    newController_->start();
                    active_controller_ = newController_;
                }
                switchState_ = SwitchState::RUNNING;
                break;

            case SwitchState::RUNNING:
            // std::cout<<"Running "<<std::endl;
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

        bool ControllerManager::requiresBridging(BaseController *oldCtrl, BaseController *newCtrl)
        {
            if (!oldCtrl || !newCtrl)
            {
                return false;
            }
            // e.g., if old is torque-based and new is position-based => bridging needed
            return false;
        }

        std::shared_ptr<BaseController> ControllerManager::findControllerById(seven_axis_robot::merai::ControllerID id)
        {
            int idx = static_cast<int>(id);
            if (idx < 0 || idx >= static_cast<int>(idToController_.size()))
            {
                return nullptr;
            }
            return idToController_[idx];
        }
    } // namespace control
} // namespace seven_axis_robot
