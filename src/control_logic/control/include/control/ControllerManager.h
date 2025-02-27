#pragma once

#include <atomic>
#include <string>
#include <memory>
#include <array>

#include "merai/ParameterServer.h"   // hand_control::merai::ParameterServer
#include "merai/RTMemoryLayout.h"    // hand_control::merai::JointState, JointCommand, ControllerUserCommand, ControllerFeedback
#include "control/controllers/BaseController.h"         // hand_control::control::BaseController

namespace hand_control
{
    namespace control
    {
        static constexpr int MAX_CONTROLLERS = 10;

        enum class SwitchState
        {
            IDLE,
            STOP_OLD,
            BRIDGING,
            START_NEW,
            RUNNING
        };

        class ControllerManager
        {
        public:
            ControllerManager(const hand_control::merai::ParameterServer* paramServer);

            ~ControllerManager();

            bool registerController(std::shared_ptr<BaseController> controller);
            bool init();

            /**
             * @brief update
             *   - Takes in userCommands array (from shared memory, but read externally)
             *   - Processes bridging or switching logic
             *   - Runs the active or bridging controller
             *   - Writes minimal feedback bits (like bridgingActive) into feedback array
             *
             * @param states        Array of JointState
             * @param commands      Array of JointCommand
             * @param userCmdArray  Array of ControllerUserCommand
             * @param feedbackArray Array of ControllerFeedback
             * @param jointCount    Number of joints
             * @param dt            Time step (seconds)
             */
            void update(const hand_control::merai::JointState* states,
                        hand_control::merai::JointCommand*     commands,
                        const hand_control::merai::ControllerUserCommand* userCmdArray,
                        hand_control::merai::ControllerFeedback*          feedbackArray,
                        int jointCount,
                        double dt);

        private:
            void processControllerSwitch();
            bool requiresBridging(BaseController* oldCtrl, BaseController* newCtrl);
            std::shared_ptr<BaseController> findControllerByName(const std::string& name);

            const hand_control::merai::ParameterServer* paramServer_ = nullptr;
            int jointCount_ = 0;

            std::array<std::shared_ptr<BaseController>, MAX_CONTROLLERS> controllers_{};
            int num_controllers_{0};

            std::shared_ptr<BaseController> active_controller_{nullptr};

            SwitchState switchState_{SwitchState::IDLE};
            bool bridgingNeeded_{false};

            std::shared_ptr<BaseController> oldController_{nullptr};
            std::shared_ptr<BaseController> newController_{nullptr};

            std::atomic<bool> switch_pending_{false};
            std::string target_controller_name_;
        };
    } // namespace control
} // namespace hand_control
