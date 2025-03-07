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
        constexpr int MAX_IO_DRIVES    = 4;

        // =======================================
        // 2) Basic Fieldbus Structures
        // =======================================
        struct ServoRxPdo
        {
            uint16_t controlWord     = 0;
            uint8_t  modeOfOperation = 0;
            int16_t  targetTorque    = 0;
            int32_t  targetPosition  = 0;
            int32_t  targetVelocity  = 0;
        };

        struct ServoTxPdo
        {
            uint16_t statusWord     = 0;
            int32_t  positionActual = 0;
            int32_t  velocityActual = 0;
            int16_t  torqueActual   = 0;
        };

        struct IoRxPdo
        {
            std::array<bool, 16>  digitalOutputs{};
            std::array<float, 4>  analogOutputs{};
        };

        struct IoTxPdo
        {
            std::array<bool, 16>  digitalInputs{};
            std::array<float, 4>  analogInputs{};
        };

        // =======================================
        // 3) High-Level Joint Data (SI units)
        // =======================================
        struct JointCommand
        {
            double position = 0.0;
            double velocity = 0.0;
            double torque   = 0.0;
        };

        struct JointState
        {
            double position = 0.0;
            double velocity = 0.0;
            double torque   = 0.0;
        };

        // =======================================
        // 4) High-Level I/O Data
        // =======================================
        struct IoCommand
        {
            std::array<bool, 8>  digitalOutputs{};
            std::array<float, 2> analogOutputs{};
        };

        struct IoState
        {
            std::array<bool, 8>  digitalInputs{};
            std::array<float, 2> analogInputs{};
        };

        // =======================================
        // 5) Drive-Level Signals & Feedback
        // =======================================
        struct DriveUserSignals
        {
            bool faultReset     = false;
            bool allowOperation = false;
            bool quickStop      = false;
            bool forceDisable   = false;
        };

        struct DriveFeedback
        {
            bool faultActive      = false;
            bool operationEnabled = false;
        };

        // =======================================
        // 6) Controller-Level Commands & Feedback
        // =======================================
        /**
         * @brief ControllerUserCommand
         *  - If we want to store an enum (ControllerID) instead of a string,
         *    use controllerId. This avoids string copying in RT code.
         */
        struct ControllerUserCommand
        {
            bool requestSwitch = false;
            hand_control::merai::ControllerID controllerId = hand_control::merai::ControllerID::NONE;
        };

        struct ControllerFeedback
        {
            bool switchInProgress = false;
            bool bridgingActive   = false;
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
            std::array<JointState,   MAX_SERVO_DRIVES> states;
        };

        struct IoData
        {
            std::array<IoCommand, MAX_IO_DRIVES> commands;
            std::array<IoState,   MAX_IO_DRIVES> states;
        };

        // =======================================
        // 10) Aggregated Drive Signals & Feedback
        // =======================================
        struct DriveUserSignalsData
        {
            std::array<DriveUserSignals, MAX_SERVO_DRIVES> signals;
        };

        struct DriveFeedbackData
        {
            std::array<DriveFeedback, MAX_SERVO_DRIVES> feedback;
        };

        // =======================================
        // 11) Controller Commands & Feedback
        // =======================================
        struct ControllerUserCommandData
        {
            // We only store a single command in this example, but you can store more if needed
            std::array<ControllerUserCommand, 1> commands;
        };

        struct ControllerFeedbackData
        {
            std::array<ControllerFeedback, 1> feedback;
        };

        // =======================================
        // 12) Additional Aggregators for Logic <-> Control
        // =======================================

        /**
         * @brief DriveSummary
         *  - The Control side writes whether any drive is faulted and how severe
         *  - The Logic side reads this aggregator to decide next actions
         */
        struct DriveSummary
        {
            bool anyFaulted     = false;
            int  faultSeverity  = 0; // 1 => recoverable, 2 => major, etc.
        };

        /**
         * @brief DriveCommandAggregated
         *  - The Logic side writes a single enumerated drive command
         *  - The Control side reads it and applies the final drive actions
         */
        struct DriveCommandAggregated
        {
            hand_control::merai::DriveCommand driveCommand
                = hand_control::merai::DriveCommand::NONE;
        };

        /**
         * @brief ControllerCommandAggregated
         *  - The Logic side writes whether we want a new controller switch
         *  - The Control side sees requestSwitch + which controller ID
         */
        struct ControllerCommandAggregated
        {
            bool requestSwitch = false;
            hand_control::merai::ControllerID targetController
                = hand_control::merai::ControllerID::NONE;
        };

        // =======================================
        // 13) Top-Level Shared Layout
        // =======================================
        struct RTMemoryLayout
        {
            // Real-time buffer for servo + IO data
            DoubleBuffer<ServoSharedData> servoBuffer;
            DoubleBuffer<IoSharedData>    ioBuffer;

            // High-level data for joints + IO
            DoubleBuffer<JointData> jointBuffer;
            DoubleBuffer<IoData>    ioDataBuffer;

            // Drive-level signals & feedback
            DoubleBuffer<DriveUserSignalsData> driveUserSignalsBuffer;
            DoubleBuffer<DriveFeedbackData>    driveFeedbackBuffer;

            // Controller user commands & feedback
            DoubleBuffer<ControllerUserCommandData> controllerUserCmdBuffer;
            DoubleBuffer<ControllerFeedbackData>    controllerFeedbackBuffer;

            // Additional aggregator buffers:
            DoubleBuffer<DriveSummary> driveSummaryBuffer;                // Control->Logic
            DoubleBuffer<DriveCommandAggregated> driveCommandBuffer;      // Logic->Control
            DoubleBuffer<ControllerCommandAggregated> controllerCommandsAggBuffer; // Logic->Control
        };
    } // namespace merai
} // namespace hand_control
