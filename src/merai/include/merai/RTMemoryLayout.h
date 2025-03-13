#pragma once

#include <array>
#include <atomic>
#include <cstdint>

// Include your newly created Enums.h for DriveCommand, ControllerID, etc.
#include "merai/Enums.h"

namespace hand_control
{
    namespace merai
    {
        // =======================================
        // 1) Maximum constants
        // =======================================
        constexpr int MAX_SERVO_DRIVES = 12;
        constexpr int MAX_IO_DRIVES = 4;

        // =======================================
        // 2) Basic Fieldbus Structures
        // =======================================
        struct ServoRxPdo
        {
            uint16_t controlWord = 0;
            uint8_t modeOfOperation = 0;
            int16_t targetTorque = 0;
            int32_t targetPosition = 0;
            int32_t targetVelocity = 0;
        };

        struct ServoTxPdo
        {
            uint16_t statusWord = 0;
            int32_t positionActual = 0;
            int32_t velocityActual = 0;
            int16_t torqueActual = 0;
        };

        struct IoRxPdo
        {
            std::array<bool, 16> digitalOutputs{};
            std::array<float, 4> analogOutputs{};
        };

        struct IoTxPdo
        {
            std::array<bool, 16> digitalInputs{};
            std::array<float, 4> analogInputs{};
        };

        // =======================================
        // 3) High-Level Joint Data (SI units)
        // =======================================
        struct JointCommand
        {
            double position = 0.0;
            double velocity = 0.0;
            double torque = 0.0;
        };

        struct JointState
        {
            double position = 0.0;
            double velocity = 0.0;
            double torque = 0.0;
        };

        // =======================================
        // 4) High-Level I/O Data
        // =======================================
        struct IoCommand
        {
            std::array<bool, 8> digitalOutputs{};
            std::array<float, 2> analogOutputs{};
        };

        struct IoState
        {
            std::array<bool, 8> digitalInputs{};
            std::array<float, 2> analogInputs{};
        };

        // =======================================
        // 5) Drive-Level Signals & Feedback
        // =======================================
        /**
         * @brief Per-drive control signals:
         *  - The logic layer writes these booleans for each drive.
         *  - The control layer reads them in real time and sets CiA 402 bits accordingly.
         */
        struct DriveControlSignals
        {
            bool faultReset = false;
            bool allowOperation = false;
            bool quickStop = false;
            bool forceDisable = false;
        };

        /**
         * @brief Minimal per-drive feedback from Control:
         *  - e.g., faultActive, operationEnabled
         */
        struct DriveFeedback
        {
            bool fault = false;            // bit 3
            bool switchOnDisabled = false; // bit 6
            bool readyToSwitchOn = false;  // bit 0
            bool switchedOn = false;       // bit 1
            bool operationEnabled = false; // bit 2
            bool quickStop = false;        // bit 5
        };

        // =======================================
        // 6) Controller-Level Commands & Feedback
        // =======================================
        /**
         * @brief If we store an enum (ControllerID) for controller switching,
         *        no strings are needed.
         */
        struct ControllerCommand
        {
            bool requestSwitch = false;
            hand_control::merai::ControllerID controllerId = hand_control::merai::ControllerID::NONE;
        };

        struct ControllerFeedback
        {
            bool switchInProgress = false;
            bool bridgingActive = false;
            bool controllerFailed = false;
        };

        // =======================================
        // 7) DoubleBuffer Template
        // =======================================
        template <typename T>
        struct DoubleBuffer
        {
            T buffer[2];
            std::atomic<int> frontIndex{0}; // 0 or 1
        };

        // =======================================
        // 8) Aggregated Fieldbus Data
        // =======================================
        struct ServoSharedData
        {
            std::array<ServoRxPdo, MAX_SERVO_DRIVES> rx;
            std::array<ServoTxPdo, MAX_SERVO_DRIVES> tx;
        };

        struct IoSharedData
        {
            std::array<IoRxPdo, MAX_IO_DRIVES> rx;
            std::array<IoTxPdo, MAX_IO_DRIVES> tx;
        };

        // =======================================
        // 9) Aggregated High-Level Data
        // =======================================
        struct JointData
        {
            std::array<JointCommand, MAX_SERVO_DRIVES> commands;
            std::array<JointState, MAX_SERVO_DRIVES> states;
        };

        struct IoData
        {
            std::array<IoCommand, MAX_IO_DRIVES> commands;
            std::array<IoState, MAX_IO_DRIVES> states;
        };

        // =======================================
        // 10) Aggregated Drive Signals & Feedback
        // =======================================
        /**
         * @brief For logic->control: an array of drive control signals,
         *        one for each drive.
         */
        struct DriveControlSignalsData
        {
            std::array<DriveControlSignals, MAX_SERVO_DRIVES> signals;
        };

        struct DriveFeedbackData
        {
            std::array<DriveFeedback, MAX_SERVO_DRIVES> feedback;
        };

        // =======================================
        // 11) Controller Commands & Feedback
        // =======================================
        struct ControllerCommandData
        {
            std::array<ControllerCommand, 1> commands;
        };

        struct ControllerFeedbackData
        {
            std::array<ControllerFeedback, 1> feedback;
        };

        // =======================================
        // 12) Additional Aggregators for Logic <-> Control
        // =======================================

        /**
         * @brief ControllerCommandAggregated
         *  - The Logic side writes whether we want a new controller switch
         *  - The Control side reads requestSwitch + which controller ID
         */
        struct ControllerCommandAggregated
        {
            bool requestSwitch = false;
            hand_control::merai::ControllerID targetController = hand_control::merai::ControllerID::NONE;
        };

        // 14) Logic <-> User Command & Feedback

        /**
         * @brief Simple user commands aggregator:
         *  - Logic reads these to see if user pressed eStop, resetFault, etc.
         *  - 'desiredMode' could be an enum like IDLE, TELEOP, HOMING, etc.
         */
        struct UserCommands
        {
            bool eStop = false;
            bool resetFault = false;
            bool shutdownRequest = false;
            hand_control::merai::SystemMode desiredMode = hand_control::merai::SystemMode::IDLE; // or however your enum is defined
        };

        /**
         * @brief Simple user feedback aggregator:
         *  - Logic writes these so the user process sees the current state,
         *    whether there's a fault, etc.
         */
        struct UserFeedback
        {
            bool faultActive = false;
            hand_control::merai::OrchestratorState currentState = hand_control::merai::OrchestratorState::IDLE;
            hand_control::merai::SystemMode desiredMode = hand_control::merai::SystemMode::IDLE;
            // Add more fields (like homingInProgress) if desired
        };

        // =======================================
        // 13) Top-Level Shared Layout
        // =======================================
        struct RTMemoryLayout
        {
            // Existing fields
            DoubleBuffer<ServoSharedData> servoBuffer;
            DoubleBuffer<IoSharedData> ioBuffer;
            DoubleBuffer<JointData> jointBuffer;
            DoubleBuffer<IoData> ioDataBuffer;

            DoubleBuffer<DriveControlSignalsData> driveControlSignalsBuffer;
            DoubleBuffer<DriveFeedbackData> driveFeedbackBuffer;

            DoubleBuffer<ControllerCommandData> controllerCommandBuffer;
            DoubleBuffer<ControllerFeedbackData> controllerFeedbackBuffer;

            DoubleBuffer<ControllerCommandAggregated> controllerCommandsAggBuffer;

            // NEW aggregators for user commands & user feedback:
            DoubleBuffer<UserCommands> userCommandsBuffer;
            DoubleBuffer<UserFeedback> userFeedbackBuffer;
        };
    } // namespace merai
} // namespace hand_control
