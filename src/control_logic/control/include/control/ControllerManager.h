#pragma once

#include <atomic>
#include <memory>
#include <array>

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/Enums.h" // For merai::ControllerID
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
         *        memory layout (ControllerCommandData + ControllerFeedbackData).
         *
         *  In this ID-based approach, we do not store or search by name. Instead, each
         *  ControllerID is mapped to a specific index or direct pointer. If the user
         *  tries to switch to an ID that wasn't registered, we skip or set 'controllerFailed'.
         */
        class ControllerManager
        {
        public:
            /**
             * @brief Constructor
             * @param paramServer pointer to the shared ParameterServer, used for config (jointCount, etc.).
             */
            ControllerManager(const hand_control::merai::ParameterServer* paramServer);
            ~ControllerManager();

            /**
             * @brief Register a controller for a specific ID. If you have a direct mapping
             *        from ID -> index, store it. Otherwise, store in a list and let the code
             *        below do a small ID->index approach.
             *
             * @return false if we exceed MAX_CONTROLLERS or an error, true otherwise
             */
            bool registerController(hand_control::merai::ControllerID id,
                                    std::shared_ptr<BaseController> controller);

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
             * @param states       pointer to array of JointState
             * @param commands     pointer to array of JointCommand
             * @param ctrlCmdArray pointer to aggregator buffer (ControllerCommand),
             *                     where [0] has (requestSwitch, controllerId).
             * @param feedbackArray pointer to aggregator feedback (ControllerFeedback),
             *                     where [0] is where we fill bridging/switch status.
             * @param jointCount   number of joints
             * @param dt           delta time (seconds) for this update cycle
             */
            void update(const hand_control::merai::JointState*    states,
                        hand_control::merai::JointCommand*        commands,
                        const hand_control::merai::ControllerCommand* ctrlCmdArray,
                        hand_control::merai::ControllerFeedback*  feedbackArray,
                        double dt);

        private:
            /**
             * @brief Internal method that handles the state machine of controller switching
             */
            void processControllerSwitch();

            /**
             * @brief Check if bridging is needed (e.g. torque-based -> position-based).
             */
            bool requiresBridging(BaseController* oldCtrl, BaseController* newCtrl);

            /**
             * @brief Looks up the pointer for a given ID. Return nullptr if not found.
             */
            std::shared_ptr<BaseController> findControllerById(hand_control::merai::ControllerID id);

        private:
            const hand_control::merai::ParameterServer* paramServer_ = nullptr;
            int jointCount_ = 0;

            // We store an array or map from ID -> controller pointer
            // Because MAX_CONTROLLERS=10, we can store them in a small array
            // where the index is static_cast<int>(id).
            std::array<std::shared_ptr<BaseController>, static_cast<int>(hand_control::merai::ControllerID::E_STOP) + 1> idToController_{};

            // The currently active controller
            std::shared_ptr<BaseController> active_controller_{nullptr};

            // Switch state machine
            SwitchState switchState_{SwitchState::IDLE};
            bool bridgingNeeded_{false};

            // Controller pointers during switch
            std::shared_ptr<BaseController> oldController_{nullptr};
            std::shared_ptr<BaseController> newController_{nullptr};

            // Whether a switch was requested
            std::atomic<bool> switch_pending_{false};

            // The ID of the next desired controller
            hand_control::merai::ControllerID target_controller_id_
                = hand_control::merai::ControllerID::NONE;
        };

    } // namespace control
} // namespace hand_control
