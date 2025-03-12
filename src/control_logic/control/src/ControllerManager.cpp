#include <stdexcept> // for std::runtime_error
#include "control/ControllerManager.h"
#include "control/controllers/HomingController.h"
#include "control/controllers/GravityCompController.h"
// #include "control/controllers/EStopController.h"

namespace hand_control
{
    namespace control
    {
        ControllerManager::ControllerManager(const hand_control::merai::ParameterServer *paramServer)
            : paramServer_(paramServer)
        {
            if (!paramServer_)
            {
                throw std::runtime_error("ControllerManager: Null paramServer pointer.");
            }
            jointCount = static_cast<int>(paramServer_->jointCount);

            // By default, idToController_ array is empty (all nullptr)
            // e.g. [ControllerID::NONE, ControllerID::HOMING, ...] => indexes
        }

        ControllerManager::~ControllerManager()
        {
            // cleanup if needed
        }

        bool ControllerManager::registerController(hand_control::merai::ControllerID id,
                                                   std::shared_ptr<BaseController> controller)
        {
            if (!controller)
            {
                return false;
            }

            // Convert the ID to an index
            int idx = static_cast<int>(id);
            // e.g. if idx is out of range or already occupied => fail
            if (idx < 0 || idx >= static_cast<int>(idToController_.size()))
            {
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
            bool success = true;

            // Initialize all registered controllers
            for (auto &ctrlPtr : idToController_)
            {
                if (ctrlPtr)
                {
                    if (!ctrlPtr->init())
                    {
                        success = false;
                    }
                }
            }

            // Do NOT set any default active controller here;
            // leave active_controller_ = nullptr
            active_controller_ = nullptr;
            switchState_ = SwitchState::IDLE;

            return success;
        }

        void ControllerManager::update(const hand_control::merai::JointState *states,
                                       hand_control::merai::JointCommand *commands,
                                       const hand_control::merai::ControllerCommand *ctrlCmdArray,
                                       hand_control::merai::ControllerFeedback *feedbackArray,
                                       double dt)
        {
            // We only have 1 aggregator element in ControllerCommandData
            const auto &cmd = ctrlCmdArray[0];

            if (cmd.requestSwitch)
            {
                target_controller_id_ = cmd.controllerId;
                switch_pending_.store(true);
            }

            // (1) Handle bridging logic if needed
            processControllerSwitch();

            // (2) Run the active controller if we have one
            if (active_controller_)
            {
                active_controller_->update(states, commands, jointCount, dt);
            }
            else
            {
                // fallback: hold position if no controller active
                for (int i = 0; i < jointCount; ++i)
                {
                    commands[i].position = states[i].position;
                    commands[i].velocity = 0.0;
                    commands[i].torque = 0.0;
                }
            }

            // (3) Minimal feedback
            bool bridging = (switchState_ == SwitchState::BRIDGING);
            bool switching = (switchState_ != SwitchState::RUNNING);

            feedbackArray[0].bridgingActive = bridging;
            feedbackArray[0].switchInProgress = switching;
            feedbackArray[0].controllerFailed = false; // if we detect failure below, set true
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
                        // possibly set feedback aggregator
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

        bool ControllerManager::requiresBridging(BaseController *oldCtrl, BaseController *newCtrl)
        {
            if (!oldCtrl || !newCtrl)
            {
                return false;
            }
            // e.g. if old is torque-based and new is position-based => bridging needed
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
