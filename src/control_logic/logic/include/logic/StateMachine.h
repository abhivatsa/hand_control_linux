#pragma once

#include "merai/Enums.h"
#include "merai/RTMemoryLayout.h"
#include "merai/ParameterServer.h"

namespace hand_control
{
    namespace logic
    {
        // Output struct from StateMachine update that bundles all commands and status.
        struct StateManagerOutput
        {
            hand_control::merai::DriveCommandData driveCmd;
            hand_control::merai::ControllerCommand ctrlCmd;
            merai::AppState appState;
        };

        /**
         * @brief StateMachine (formerly SystemOrchestrator)
         *  - Manages high-level state transitions (INIT, HOMING, ACTIVE, FAULT)
         *  - Optionally sets drive signals or controller switches
         *  - Replaces references to "SystemOrchestrator"
         */
        class StateMachine
        {
        public:
            StateMachine(const hand_control::merai::ParameterServer *paramServerPtr);

            /**
             * @brief Initializes internal flags and sets default state to INIT.
             */
            bool init();

            /**
             * @brief Updates the state machine:
             *  - Checks for faults and user requests,
             *  - Performs state transitions,
             *  - Computes drive and controller commands.
             *
             * @param faultActive True if a fault is detected.
             * @param isHomingCompleted True if homing has been completed.
             * @param userCmds The latest user commands.
             * @return StateManagerOutput struct containing drive command, controller command, and current application state.
             */
            StateManagerOutput update(bool faultActive, bool isHomingCompleted,
                                      const hand_control::merai::DriveFeedbackData &driveFdbk,
                                      const hand_control::merai::UserCommands &userCmds,
                                      const hand_control::merai::ControllerFeedback &ctrlFdbk);

        private:
            const hand_control::merai::ParameterServer *paramServerPtr_ = nullptr;
            // The current application state
            hand_control::merai::AppState currentState_ = hand_control::merai::AppState::INIT;
            hand_control::merai::DriveCommandData driveCmd_ = hand_control::merai::DriveCommandData{};
            hand_control::merai::ControllerCommand ctrlCmd_ = hand_control::merai::ControllerCommand{};

            // Example: number of drives / joints
            std::size_t driveCount_ = 0;
            static constexpr int MAX_JOINTS = 7;

            bool state_transistion = true;
            bool all_drive_operation_enable = true;
            bool all_drives_switched_on = true;
            bool all_drives_switch_on_disabled = true;
        };
    } // namespace logic
} // namespace hand_control
