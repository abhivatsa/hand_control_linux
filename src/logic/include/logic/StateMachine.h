#pragma once

#include "merai/Enums.h"
#include "merai/RTMemoryLayout.h"
#include "merai/ParameterServer.h"
#include "merai/SharedLogger.h"

namespace seven_axis_robot
{
    namespace logic
    {
        // Output struct from StateMachine update that bundles all commands and status.
        struct StateManagerOutput
        {
            seven_axis_robot::merai::DriveCommandData driveCmd;
            seven_axis_robot::merai::ControllerCommand ctrlCmd;
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
            StateMachine(const seven_axis_robot::merai::ParameterServer *paramServerPtr,
                         seven_axis_robot::merai::multi_ring_logger_memory* loggerMem);

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
                                      const seven_axis_robot::merai::DriveFeedbackData &driveFdbk,
                                      const seven_axis_robot::merai::UserCommands &userCmds,
                                      const seven_axis_robot::merai::ControllerFeedback &ctrlFdbk);

        private:
            const seven_axis_robot::merai::ParameterServer *paramServerPtr_ = nullptr;
            seven_axis_robot::merai::multi_ring_logger_memory* loggerMem_ = nullptr;
            // The current application state
            seven_axis_robot::merai::AppState currentState_ = seven_axis_robot::merai::AppState::INIT;
            seven_axis_robot::merai::DriveCommandData driveCmd_ = seven_axis_robot::merai::DriveCommandData{};
            seven_axis_robot::merai::ControllerCommand ctrlCmd_ = seven_axis_robot::merai::ControllerCommand{};

            // Example: number of drives / joints
            std::size_t driveCount_ = 0;
            static constexpr int MAX_JOINTS = 7;

            bool state_transistion = true;
            bool all_drive_operation_enable = true;
            bool all_drives_switched_on = true;
            bool all_drives_switch_on_disabled = true;
        };
    } // namespace logic
} // namespace seven_axis_robot
