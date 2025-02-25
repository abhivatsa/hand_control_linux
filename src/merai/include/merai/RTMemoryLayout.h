#pragma once

#include <array>
#include <atomic>
#include <cstdint>

namespace motion_control
{
    namespace merai
    {
        // =======================
        // 1) Maximum constants
        // =======================
        constexpr int MAX_SERVO_DRIVES = 12;
        constexpr int MAX_IO_DRIVES    = 4;

        // =======================
        // 2) Basic Fieldbus Structures
        // =======================
        struct ServoRxPdo
        {
            uint16_t controlWord     = 0; // e.g. 0x6040
            uint8_t  modeOfOperation = 0; // e.g. 0x6060
            int16_t  targetTorque    = 0; // e.g. 0x6071
            int32_t  targetPosition  = 0; // e.g. 0x607A
            int32_t  targetVelocity  = 0; // e.g. 0x60FF
        };

        struct ServoTxPdo
        {
            uint16_t statusWord     = 0; // e.g. 0x6041
            int32_t  positionActual = 0; // e.g. 0x6064
            int32_t  velocityActual = 0; // e.g. 0x606C
            int16_t  torqueActual   = 0; // e.g. 0x6077
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

        // =======================
        // 3) High-Level Joint Data (SI units)
        // =======================
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

        // =======================
        // 4) High-Level I/O Data
        // =======================
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

        // =======================
        // 5) Drive-Level Signals & Feedback
        // =======================
        struct DriveUserSignals
        {
            bool faultReset     = false;
            bool allowOperation = false;
            bool quickStop      = false;
            bool forceDisable   = false;
        };

        struct DriveFeedback
        {
            bool faultActive       = false;
            bool operationEnabled  = false;
        };

        // =======================
        // 6) Controller-Level Commands & Feedback
        // =======================
        struct ControllerUserCommand
        {
            bool requestSwitch = false;
            char targetControllerName[64] = {0};
        };

        struct ControllerFeedback
        {
            bool switchInProgress = false;
            bool bridgingActive   = false;
            bool controllerFailed = false;
        };

        // =======================
        // 7) DoubleBuffer Template
        // =======================
        template <typename T>
        struct DoubleBuffer
        {
            T buffer[2];
            std::atomic<int> frontIndex{0}; // 0 or 1
        };

        // =======================
        // 8) Aggregated Fieldbus Data
        // =======================
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

        // =======================
        // 9) Aggregated High-Level Data
        // =======================
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

        // =======================
        // 10) Aggregated Drive Signals & Feedback
        // =======================
        struct DriveUserSignalsData
        {
            std::array<DriveUserSignals, MAX_SERVO_DRIVES> signals;
        };

        struct DriveFeedbackData
        {
            std::array<DriveFeedback, MAX_SERVO_DRIVES> feedback;
        };

        // =======================
        // 11) Controller Signals & Feedback
        // =======================
        struct ControllerUserCommandData
        {
            std::array<ControllerUserCommand, 1> commands;
        };

        struct ControllerFeedbackData
        {
            std::array<ControllerFeedback, 1> feedback;
        };

        // =======================
        // 12) Top-Level Shared Layout
        // =======================
        struct RTMemoryLayout
        {
            DoubleBuffer<ServoSharedData> servoBuffer;
            DoubleBuffer<IoSharedData>    ioBuffer;

            DoubleBuffer<JointData> jointBuffer;
            DoubleBuffer<IoData>    ioDataBuffer;

            DoubleBuffer<DriveUserSignalsData> driveUserSignalsBuffer;
            DoubleBuffer<DriveFeedbackData>    driveFeedbackBuffer;

            DoubleBuffer<ControllerUserCommandData> controllerUserCmdBuffer;
            DoubleBuffer<ControllerFeedbackData>    controllerFeedbackBuffer;
        };
    } // namespace merai
} // namespace motion_control
