#include <iostream>
#include <stdexcept>

#include "control/ControllerManager.h"

// Example specialized controllers
#include "control/controllers/HomingController.h"
#include "control/controllers/GravityDampingController.h"
#include "control/controllers/EStopController.h"

// If needed for merai::ControllerID
#include "merai/Enums.h"

namespace hand_control
{
    namespace control
    {
        ControllerManager::ControllerManager(const hand_control::merai::ParameterServer* paramServer)
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
            // Initialize all registered controllers
            for (int i = 0; i < num_controllers_; i++)
            {
                if (controllers_[i] &&
                    !controllers_[i]->init(controllers_[i]->name()))
                {
                    success = false;
                }
            }

            // Default to the first registered controller if any
            if (num_controllers_ > 0)
            {
                active_controller_ = controllers_[0];
                switchState_ = SwitchState::RUNNING;
                std::cout << "[ControllerManager] Active controller: "
                          << active_controller_->name() << std::endl;
            }
            return success;
        }

        void ControllerManager::update(const hand_control::merai::JointState* states,
                                       hand_control::merai::JointCommand* commands,
                                       const hand_control::merai::ControllerUserCommand* userCmdArray,
                                       hand_control::merai::ControllerFeedback* feedbackArray,
                                       int jointCount,
                                       double dt)
        {
            // We only have 1 user command in this example
            const auto& userCmd = userCmdArray[0];

            // If the user requests a controller switch, store the ID
            if (userCmd.requestSwitch)
            {
                target_controller_id_ = userCmd.controllerId;
                switch_pending_.store(true);
            }

            // (1) Handle bridging logic (or skip if bridgingNeeded_ = false)
            processControllerSwitch();

            // (2) Run the active controller if we have one
            if (active_controller_)
            {
                active_controller_->update(states, commands, jointCount, dt);
            }
            else
            {
                // Fallback: hold current position if no controller is active
                for (int i = 0; i < jointCount; ++i)
                {
                    commands[i].position = states[i].position;
                    commands[i].velocity = 0.0;
                    commands[i].torque   = 0.0;
                }
            }

            // (3) Minimal feedback
            bool bridging  = (switchState_ == SwitchState::BRIDGING);
            bool switching = (switchState_ != SwitchState::RUNNING);

            feedbackArray[0].bridgingActive   = bridging;
            feedbackArray[0].switchInProgress = switching;
            feedbackArray[0].controllerFailed = false; // set true if newController_ not found, etc.
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
                        std::cerr << "[ControllerManager] Controller not found for ID: "
                                  << static_cast<int>(target_controller_id_) << "\n";
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
                    // bridging logic
                    switchState_ = SwitchState::BRIDGING;
                }
                else
                {
                    switchState_ = SwitchState::START_NEW;
                }
                break;

            case SwitchState::BRIDGING:
                // If bridging is needed, do partial transitions
                // For simplicity, go directly to START_NEW
                switchState_ = SwitchState::START_NEW;
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
                    newController_ = findControllerById(target_controller_id_);
                    if (!newController_)
                    {
                        std::cerr << "[ControllerManager] Controller not found for ID: "
                                  << static_cast<int>(target_controller_id_) << "\n";
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
            // Example logic
            if (!oldCtrl || !newCtrl)
            {
                return false;
            }
            // e.g., if old is torque-based and new is position-based, bridging might be needed
            return false;
        }

        /**
         * @brief We assume each BaseController either has a known ID or we keep a 
         *        mapping from ID to index. For demonstration, we'll do a naive loop
         *        checking each controller's "id()" if you store it. Otherwise, you could
         *        do a switch-case or a direct array index if IDs map 1:1 to array positions.
         */
        std::shared_ptr<BaseController> ControllerManager::findControllerById(hand_control::merai::ControllerID id)
        {
            // Example: if your controllers store an ID in their derived classes:
            // 
            // for (int i = 0; i < num_controllers_; i++)
            // {
            //     if (controllers_[i] && controllers_[i]->controllerId() == id)
            //     {
            //         return controllers_[i];
            //     }
            // }
            // 
            // Return nullptr if not found

            // For now, we’ll just do name-based fallback or a simple switch 
            // if your real code hasn't been fully adapted to ID-based:
            switch (id)
            {
                case hand_control::merai::ControllerID::HOMING:
                    // find the HomingController in our array, or create it
                    return findControllerByName("HomingController");
                case hand_control::merai::ControllerID::GRAVITY_COMP:
                    return findControllerByName("GravityDampingController");
                case hand_control::merai::ControllerID::E_STOP:
                    return findControllerByName("EStopController");
                default:
                    return nullptr;
            }
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
