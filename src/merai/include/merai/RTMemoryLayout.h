#pragma once

#include <array>
#include <atomic>
#include <cstdint>

// Bring in your Enums.h for DriveCommand, DriveStatus, ControllerID, etc.
#include "merai/Enums.h"

namespace hand_control
{
    namespace merai
    {
        // =======================================
        // 1) Maximum constants
        // =======================================
        constexpr int MAX_SERVO_DRIVES = 12;

        // =======================================
        // 2) Basic Fieldbus Structures (Servo)
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
        // 4) Drive-Level Command & Feedback (Enums)
        //    (Matches the DriveCommand / DriveStatus)
        // =======================================
        struct DriveCommandData
        {
            std::array<hand_control::merai::DriveCommand, MAX_SERVO_DRIVES> commands;
        };

        struct DriveFeedbackData
        {
            std::array<hand_control::merai::DriveStatus, MAX_SERVO_DRIVES> status;
        };

        // =======================================
        // 5) Controller-Level Commands & Feedback
        // =======================================
        struct ControllerCommand
        {
            bool requestSwitch = false;
            hand_control::merai::ControllerID controllerId = hand_control::merai::ControllerID::NONE;
        };

        struct ControllerFeedback
        {
            hand_control::merai::ControllerFeedbackState feedback =
                hand_control::merai::ControllerFeedbackState::IDLE;
        };

        // =======================================
        // 6) DoubleBuffer Template
        // =======================================
        template <typename T>
        struct DoubleBuffer
        {
            T buffer[2];
            std::atomic<int> frontIndex{0}; // 0 or 1
        };

        // =======================================
        // 7) Aggregated Fieldbus Data
        // =======================================
        struct ServoSharedData
        {
            std::array<ServoRxPdo, MAX_SERVO_DRIVES> rx;
            std::array<ServoTxPdo, MAX_SERVO_DRIVES> tx;
        };

        // =======================================
        // 8) Aggregated High-Level Data
        // =======================================
        struct JointData
        {
            std::array<JointCommand, MAX_SERVO_DRIVES> commands;
            std::array<JointState, MAX_SERVO_DRIVES> states;
        };

        // =======================================
        // 9) Aggregated Controller Data
        // =======================================
        struct ControllerCommandData
        {
            ControllerCommand commands;
        };

        struct ControllerFeedbackData
        {
            ControllerFeedback feedback;
        };

        // Optionally, if you need a separate struct for partial or combined info:
        struct ControllerCommandAggregated
        {
            bool requestSwitch = false;
            hand_control::merai::ControllerID targetController = hand_control::merai::ControllerID::NONE;
        };

        // =======================================
        // 10) Logic <-> User Command & Feedback
        // =======================================
        struct UserCommands
        {
            bool eStop = false;
            bool resetFault = false;
            bool shutdownRequest = false;
            hand_control::merai::UserMode desiredMode = hand_control::merai::UserMode::HOMING;
        };

        struct UserFeedback
        {
            bool faultActive = false;
            hand_control::merai::AppState currentState = hand_control::merai::AppState::HOMING;
            hand_control::merai::UserMode desiredMode = hand_control::merai::UserMode::HOMING;
        };

        // =======================================
        // 11) Top-Level Shared Layout
        // =======================================
        struct RTMemoryLayout
        {
            // Servo fieldbus data
            DoubleBuffer<ServoSharedData> servoBuffer;

            // High-level joint data
            DoubleBuffer<JointData> jointBuffer;

            // Drive-level commands & feedback
            DoubleBuffer<DriveCommandData> driveCommandBuffer;
            DoubleBuffer<DriveFeedbackData> driveFeedbackBuffer;

            // Controller-level commands & feedback
            DoubleBuffer<ControllerCommandData> controllerCommandBuffer;
            DoubleBuffer<ControllerFeedbackData> controllerFeedbackBuffer;
            DoubleBuffer<ControllerCommandAggregated> controllerCommandsAggBuffer;

            // User commands & feedback
            DoubleBuffer<UserCommands> userCommandsBuffer;
            DoubleBuffer<UserFeedback> userFeedbackBuffer;
        };

    } // namespace merai
} // namespace hand_control
