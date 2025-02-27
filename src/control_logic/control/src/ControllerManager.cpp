#include <iostream>
#include <stdexcept>

#include "control/ControllerManager.h"

namespace hand_control
{
    namespace control
    {
        ControllerManager::ControllerManager(const hand_control::merai::ParameterServer *paramServer)
            : paramServer_(paramServer)
        {
            if (!paramServer_)
            {
                throw std::runtime_error("[ControllerManager] Null paramServer.");
            }
            jointCount_ = static_cast<int>(paramServer_->jointCount);
        }

        ControllerManager::~ControllerManager()
        {
            // cleanup if needed
        }

        bool ControllerManager::registerController(std::shared_ptr<BaseController> controller)
        {
            if (num_controllers_ >= MAX_CONTROLLERS)
            {
                std::cerr << "[ControllerManager] Reached max controllers.\n";
                return false;
            }
            controllers_[num_controllers_++] = controller;
            return true;
        }

        bool ControllerManager::init()
        {
            bool success = true;
            for (int i = 0; i < num_controllers_; i++)
            {
                if (controllers_[i] &&
                    !controllers_[i]->init(controllers_[i]->name()))
                {
                    success = false;
                }
            }

            // default to first if any
            if (num_controllers_ > 0)
            {
                active_controller_ = controllers_[0];
                switchState_ = SwitchState::RUNNING;
                std::cout << "[ControllerManager] Active controller: "
                          << active_controller_->name() << std::endl;
            }
            return success;
        }

        // If JointState / JointCommand come from hand_control::merai, fully qualify below:
        //   void ControllerManager::update(
        //       const hand_control::merai::JointState* states,
        //       hand_control::merai::JointCommand* commands,
        //       const hand_control::merai::ControllerUserCommand* userCmdArray,
        //       hand_control::merai::ControllerFeedback* feedbackArray,
        //       int jointCount,
        //       double dt)
        void ControllerManager::update(const hand_control::merai::JointState *states,
                                       hand_control::merai::JointCommand *commands,
                                       const hand_control::merai::ControllerUserCommand *userCmdArray,
                                       hand_control::merai::ControllerFeedback *feedbackArray,
                                       int jointCount,
                                       double dt)
        {
            // e.g. we only have 1 user command
            const auto &userCmd = userCmdArray[0];

            // Check if user requests a controller switch
            if (userCmd.requestSwitch)
            {
                // store the request
                target_controller_name_ = userCmd.targetControllerName;
                switch_pending_.store(true);
                // Optionally clear the request in shared memory
            }

            // (1) handle bridging logic
            processControllerSwitch();

            // (2) run the active or bridging controller
            if (active_controller_)
            {
                active_controller_->update(states, commands, jointCount, dt);
            }
            else
            {
                // fallback: hold current position
                for (int i = 0; i < jointCount; ++i)
                {
                    commands[i].position = states[i].position;
                    commands[i].velocity = 0.0;
                    commands[i].torque = 0.0;
                }
            }

            // (3) write minimal feedback
            bool bridging = (switchState_ == SwitchState::BRIDGING);
            bool switching = (switchState_ != SwitchState::RUNNING);

            feedbackArray[0].bridgingActive = bridging;
            feedbackArray[0].switchInProgress = switching;
            feedbackArray[0].controllerFailed = false; // e.g. if new controller not found or bridging fails
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
                    newController_ = findControllerByName(target_controller_name_);
                    if (!newController_)
                    {
                        std::cerr << "[ControllerManager] Controller not found: "
                                  << target_controller_name_ << "\n";
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
                    // bridging logic (create bridgingController_, etc.)
                    switchState_ = SwitchState::BRIDGING;
                }
                else
                {
                    switchState_ = SwitchState::START_NEW;
                }
                break;

            case SwitchState::START_NEW:
                if (newController_)
                {
                    newController_->start();
                    active_controller_ = newController_;
                    std::cout << "[ControllerManager] Switched to "
                              << active_controller_->name() << "\n";
                }
                switchState_ = SwitchState::RUNNING;
                break;

            case SwitchState::RUNNING:
                if (switch_pending_.load())
                {
                    switch_pending_.store(false);
                    oldController_ = active_controller_;
                    newController_ = findControllerByName(target_controller_name_);
                    if (!newController_)
                    {
                        std::cerr << "[ControllerManager] Controller not found: "
                                  << target_controller_name_ << "\n";
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
            // Example logic
            if (!oldCtrl || !newCtrl)
            {
                return false;
            }
            // e.g. bridging if torque->position
            // (We haven't shown controlMode() or ControlMode enum, so adjust as needed)
            return false;
        }

        std::shared_ptr<BaseController> ControllerManager::findControllerByName(const std::string &name)
        {
            for (int i = 0; i < num_controllers_; i++)
            {
                if (controllers_[i] && controllers_[i]->name() == name)
                {
                    return controllers_[i];
                }
            }
            return nullptr;
        }
    } // namespace control
} // namespace hand_control
