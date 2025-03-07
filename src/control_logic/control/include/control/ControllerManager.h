#pragma once

#include <atomic>
#include <string>
#include <memory>
#include <array>

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/Enums.h" // for merai::ControllerID
#include "control/controllers/BaseController.h"

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

        /**
         * @brief Manages multiple controllers, allowing for real-time switching from one
         *        active controller to another. Uses a double-buffer aggregator in the shared
         *        memory layout (ControllerUserCommand + ControllerFeedback).
         */
        class ControllerManager
        {
        public:
            /**
             * @brief Constructor
             * 
             * @param paramServer Pointer to the shared ParameterServer, used to retrieve
             *                    system configuration (jointCount, etc.).
             */
            ControllerManager(const hand_control::merai::ParameterServer* paramServer);
            ~ControllerManager();

            /**
             * @brief Register a controller (e.g. HomingController, GravityDampingController, etc.)
             * @param controller Shared pointer to a controller derived from BaseController
             * @return false if too many controllers or an error, true otherwise
             */
            bool registerController(std::shared_ptr<BaseController> controller);

            /**
             * @brief Initializes all registered controllers. Sets the first one as active if present.
             * @return true if all controllers initialized OK, false otherwise
             */
            bool init();

            /**
             * @brief Main update function called from the real-time loop. Checks if the user
             *        requested a controller switch, processes bridging if needed, then runs
             *        the active controller to produce joint commands.
             * 
             * @param states   Pointer to array of JointState
             * @param commands Pointer to array of JointCommand
             * @param userCmdArray Pointer to aggregator buffer (ControllerUserCommand)
             * @param feedbackArray Pointer to aggregator feedback (ControllerFeedback)
             * @param jointCount Number of joints
             * @param dt Delta time (seconds) for this update cycle
             */
            void update(const hand_control::merai::JointState*    states,
                        hand_control::merai::JointCommand*        commands,
                        const hand_control::merai::ControllerUserCommand* userCmdArray,
                        hand_control::merai::ControllerFeedback*  feedbackArray,
                        int jointCount,
                        double dt);

        private:
            /**
             * @brief Internal method that handles the state machine of controller switching
             *        (stop old controller, bridging if needed, start new controller, etc.)
             */
            void processControllerSwitch();

            /**
             * @brief Whether bridging is needed between the old and new controllers
             *        (e.g. torque-based -> position-based)
             */
            bool requiresBridging(BaseController* oldCtrl, BaseController* newCtrl);

            /**
             * @brief Finds a controller by name. This may be used for bridging if we
             *        have a partial ID -> name mapping. If you prefer fully ID-based,
             *        you can remove this or keep for debugging.
             */
            std::shared_ptr<BaseController> findControllerByName(const std::string& name);

            /**
             * @brief Finds a controller by enum ID. This is the preferred method in an
             *        enum-based system, possibly using a direct array lookup or switch-case
             *        to map from ControllerID -> the correct registered controller.
             */
            std::shared_ptr<BaseController> findControllerById(hand_control::merai::ControllerID id);

        private:
            // Parameter server
            const hand_control::merai::ParameterServer* paramServer_ = nullptr;
            int jointCount_ = 0;

            // Array of possible controllers
            std::array<std::shared_ptr<BaseController>, MAX_CONTROLLERS> controllers_{};
            int num_controllers_{0};

            // The currently active controller
            std::shared_ptr<BaseController> active_controller_{nullptr};

            // Switch state machine
            SwitchState switchState_{SwitchState::IDLE};
            bool bridgingNeeded_{false};

            // Controller pointers during switch
            std::shared_ptr<BaseController> oldController_{nullptr};
            std::shared_ptr<BaseController> newController_{nullptr};

            // Whether a switch was requested this cycle
            std::atomic<bool> switch_pending_{false};

            // If you still need bridging from name-based:
            // std::string target_controller_name_;

            // The ID of the next desired controller
            hand_control::merai::ControllerID target_controller_id_
                = hand_control::merai::ControllerID::NONE;
        };

    } // namespace control
} // namespace hand_control
