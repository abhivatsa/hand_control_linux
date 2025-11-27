#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

// Include enumerations for DriveCommand, DriveStatus, ControllerID, etc.
#include "merai/Enums.h"

namespace seven_axis_robot
{
    namespace merai
    {

        //====================================================
        // Constants
        //====================================================

        // Maximum number of servo drives supported by the system
        constexpr int MAX_SERVO_DRIVES = 12;
        constexpr uint32_t RT_MEMORY_MAGIC = 0x52544D4D;   // 'RTMM'
        constexpr uint32_t RT_MEMORY_VERSION = 1;

        //====================================================
        // Servo Fieldbus Data Structures
        //====================================================

        // ----------- Servo Rx (Commands -> from Controller to Servo) -----------

        // Control-related fields (e.g. control word, mode of operation)
        struct ServoRxControl
        {
            uint16_t controlWord = 0; // Command control word
        };

        // Motion-related fields (e.g. target torque, position, velocity)
        struct ServoRxMotion
        {
            uint8_t modeOfOperation = 8; // Operating mode indicator
            int16_t targetTorque = 0;    // Desired torque value (0x6071)
            int32_t targetPosition = 0;  // Desired position value (0x607A)
            uint16_t maxTorque = 0;      // Max torque (0x6072)
        };

        // IO-related fields (digital/analog IO, if any)
        struct ServoRxIO
        {
            uint32_t digitalOutputs = 0; // 0x60FE:01
        };

        // Consolidated Rx PDO that groups the above sub-structs
        struct ServoRxPdo
        {
            ServoRxControl ctrl;
            ServoRxMotion motion;
            ServoRxIO io; // empty for now
        };

        // ----------- Servo Tx (Feedback -> from Servo to Controller) -----------

        // Control-related feedback fields (e.g. status word, operation mode feedback)
        struct ServoTxControl
        {
            uint16_t statusWord = 0; // Feedback status word
        };

        // Motion-related feedback fields (e.g. position, velocity, torque actual)
        struct ServoTxMotion
        {
            int32_t positionActual = 0; // Actual position feedback
            int32_t velocityActual = 0; // Actual velocity feedback
            int16_t torqueActual = 0;   // Actual torque feedback (0x6077)
        };

        // IO-related feedback fields (digital/analog IO inputs, if any)
        // Again, empty for future use
        struct ServoTxIO
        {
            // e.g. digitalInputs, analogInput, etc.
            uint32_t digitalInputs = 0; // 0x60FD
            uint16_t analogInput = 0;   // 0x2401
            uint16_t error_code = 0;    // 0x603F
        };

        // Consolidated Tx PDO that groups the above sub-structs
        struct ServoTxPdo
        {
            ServoTxControl ctrl;
            ServoTxMotion motion;
            ServoTxIO io; // empty for now
        };

        // Structure holding arrays of both receive and transmit PDOs
        // for all servo drives.
        struct ServoSharedData
        {
            // Array of servo Rx PDOs (commands to servo)
            std::array<ServoRxPdo, MAX_SERVO_DRIVES> rx;
            // Array of servo Tx PDOs (feedback from servo)
            std::array<ServoTxPdo, MAX_SERVO_DRIVES> tx;
        };

        // Dedicated double buffer for servo data with split indices
        struct ServoBuffers
        {
            ServoSharedData buffer[2];
            std::atomic<int> txFrontIndex{0}; // owned/published by fieldbus producer
            std::atomic<int> rxFrontIndex{0}; // owned/published by control producer
            std::atomic<uint64_t> txSeq{0};   // sequence counter for Tx publishes
            std::atomic<uint64_t> rxSeq{0};   // sequence counter for Rx publishes
        };

        ///-------------------------------
        /// Joint Command Structures
        ///-------------------------------

        /// Control-related commands (e.g., servo control words).
        struct JointControlCommand
        {
            uint16_t controlWord = 0; // E.g., enable/disable bits, fault reset, etc.
        };

        /// Motion-related commands (target position, velocity, torque, mode, etc.)
        struct JointMotionCommand
        {
            double targetPosition = 0.0;
            double targetTorque = 0.0;
            uint8_t modeOfOperation = 8; // Tightly coupled to how the servo interprets these commands
        };

        /// I/O-related commands (digital outputs, analog outputs, etc.)
        /// Currently just placeholders; expand as needed.
        struct JointCommandIO
        {
        };

        /// Consolidated command data for each joint (what the controller *sends* to the servo).
        struct JointCommandData
        {
            JointControlCommand control; // e.g., enable, fault reset
            JointMotionCommand motion;   // position, velocity, torque, mode
            JointCommandIO io;           // digital/analog output signals
        };

        ///-------------------------------
        /// Joint Feedback Structures
        ///-------------------------------

        /// Control-related feedback (e.g., servo status word).
        struct JointControlFeedback
        {
            uint16_t statusWord = 0; // e.g., ready, enabled, fault, etc.
        };

        /// Motion-related feedback (actual position, velocity, torque).
        struct JointMotionFeedback
        {
            double positionActual = 0.0;
            double velocityActual = 0.0;
            double torqueActual = 0.0;
        };

        /// I/O-related feedback (digital inputs, analog inputs, etc.)
        struct JointFeedbackIO
        {
            bool digitalInputClutch = false;
            bool digitalInputThumb = false;
            double analogInputPinch = 0.0;
        };

        /// Consolidated feedback data for each joint (what the servo *reports* to the controller).
        struct JointFeedbackData
        {
            JointControlFeedback control;
            JointMotionFeedback motion;
            JointFeedbackIO io;
        };

        ///-------------------------------
        /// Main Container
        ///-------------------------------

        /// Holds command (to servo) and feedback (from servo) for each joint.
        /// Mirroring your servo-level “Rx” and “Tx,” but using simpler terms.
        struct JointData
        {
            // Commands going out to each joint (formerly “Rx”).
            std::array<JointCommandData, MAX_SERVO_DRIVES> commands;

            // Feedback coming back from each joint (formerly “Tx”).
            std::array<JointFeedbackData, MAX_SERVO_DRIVES> feedback;
        };

        //====================================================
        // Drive Data Structures
        //====================================================

        // Structure for drive-level command data.
        struct DriveCommandData
        {
            std::array<seven_axis_robot::merai::DriveCommand, MAX_SERVO_DRIVES> commands; // Commands for each drive
        };

        // Structure for drive-level feedback data.
        struct DriveFeedbackData
        {
            std::array<seven_axis_robot::merai::DriveStatus, MAX_SERVO_DRIVES> status; // Status for each drive
        };

        //====================================================
        // Controller Data Structures
        //====================================================

        // Structure for sending a controller command.
        struct ControllerCommand
        {
            bool requestSwitch = false;                                                               // Flag to request a controller switch
            seven_axis_robot::merai::ControllerID controllerId = seven_axis_robot::merai::ControllerID::NONE; // Target controller ID
        };

        // Structure for receiving controller feedback.
        struct ControllerFeedback
        {
            seven_axis_robot::merai::ControllerFeedbackState feedbackState = seven_axis_robot::merai::ControllerFeedbackState::IDLE;
            seven_axis_robot::merai::ControllerID activeControllerId = seven_axis_robot::merai::ControllerID::NONE;
            seven_axis_robot::merai::ControllerSwitchResult switchResult = seven_axis_robot::merai::ControllerSwitchResult::IDLE;
            uint64_t loopOverrunCount = 0;
            uint64_t halErrorCount = 0;
            bool loopOverrun = false;
            bool controllerCommandFresh = true;
            bool driveCommandFresh = true;
        };

        //====================================================
        // User Command and Feedback Structures
        //====================================================

        // Structure for commands issued by the user.
        struct UserCommands
        {
            bool eStop = false;                                                                // Emergency stop flag
            bool resetFault = false;                                                           // Reset fault command
            bool shutdownRequest = false;                                                      // Shutdown request flag
            seven_axis_robot::merai::UserMode desiredMode = seven_axis_robot::merai::UserMode::HOMING; // Desired user mode
        };

        // Structure for feedback provided to the user.
        struct UserFeedback
        {
            seven_axis_robot::merai::AppState currentState = seven_axis_robot::merai::AppState::HOMING; // Current application state
        };

        //====================================================
        // Double Buffer Template and Real-Time Memory Layout
        //====================================================

        template <typename T>
        struct DoubleBuffer
        {
            T buffer[2];                    // Two buffers for double buffering
            std::atomic<int> frontIndex{0}; // Index of the buffer currently being used for reading (0 or 1)
            std::atomic<uint64_t> seq{0};   // Monotonic publish counter
        };

        // Layout of the shared real-time memory.
        // It aggregates all the double buffers used to exchange various data types.
        struct RTMemoryLayout
        {
            uint32_t magic = RT_MEMORY_MAGIC;
            uint32_t version = RT_MEMORY_VERSION;

            // Servo fieldbus data: both transmit and receive PDOs.
            ServoBuffers servoBuffer;

            // High-level joint data: commands and state feedback for each joint.
            DoubleBuffer<JointData> jointBuffer;

            // Drive-level command and feedback buffers.
            DoubleBuffer<DriveCommandData> driveCommandBuffer;
            DoubleBuffer<DriveFeedbackData> driveFeedbackBuffer;

            // Controller-level command and feedback buffers.
            DoubleBuffer<ControllerCommand> controllerCommandBuffer;
            DoubleBuffer<ControllerFeedback> controllerFeedbackBuffer;

            // User commands and feedback buffers.
            DoubleBuffer<UserCommands> userCommandsBuffer;
            DoubleBuffer<UserFeedback> userFeedbackBuffer;
        };

        static_assert(std::is_trivially_copyable<ServoRxPdo>::value, "ServoRxPdo must be trivially copyable");
        static_assert(std::is_trivially_copyable<ServoTxPdo>::value, "ServoTxPdo must be trivially copyable");
        static_assert(std::is_trivially_copyable<ServoSharedData>::value, "ServoSharedData must be trivially copyable");
        static_assert(std::is_trivially_copyable<JointData>::value, "JointData must be trivially copyable");
        static_assert(std::is_trivially_copyable<DriveCommandData>::value, "DriveCommandData must be trivially copyable");
        static_assert(std::is_trivially_copyable<DriveFeedbackData>::value, "DriveFeedbackData must be trivially copyable");
        static_assert(std::is_trivially_copyable<ControllerCommand>::value, "ControllerCommand must be trivially copyable");
        static_assert(std::is_trivially_copyable<ControllerFeedback>::value, "ControllerFeedback must be trivially copyable");
        static_assert(std::is_trivially_copyable<UserCommands>::value, "UserCommands must be trivially copyable");
        static_assert(std::is_trivially_copyable<UserFeedback>::value, "UserFeedback must be trivially copyable");
        static_assert(std::is_trivially_copyable<RTMemoryLayout>::value, "RTMemoryLayout must be trivially copyable");
    } // namespace merai
} // namespace seven_axis_robot
