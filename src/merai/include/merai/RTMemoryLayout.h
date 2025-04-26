#pragma once

#include <array>
#include <atomic>
#include <cstdint>

// Include enumerations for DriveCommand, DriveStatus, ControllerID, etc.
#include "merai/Enums.h"

namespace hand_control
{
    namespace merai
    {

        //====================================================
        // Constants
        //====================================================

        // Maximum number of servo drives supported by the system
        constexpr int MAX_SERVO_DRIVES = 12;

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
            float targetCurrent = 0;    // Desired Current value
            int32_t targetPosition = 0;  // Desired position value
            uint16_t maxCurrent = 1000; // Max Current Input
        };

        // IO-related fields (digital/analog IO, if any)
        // Currently empty, but reserved for future expansion.
        struct ServoRxIO
        {
            // uint32_t digitalOutputs = 0; // Placeholder for digital outputs
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
            float currentActual = 0;   // Actual current feedback
        };

        // IO-related feedback fields (digital/analog IO inputs, if any)
        // Again, empty for future use
        struct ServoTxIO
        {
            // e.g. digitalInputs, analogInput, etc.
            uint32_t digitalInputs = 0; // Placeholder for digital inputs
            uint16_t analogInput = 0;   // Placeholder for analog input
            uint16_t error_code = 0;
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
            std::array<hand_control::merai::DriveCommand, MAX_SERVO_DRIVES> commands; // Commands for each drive
        };

        // Structure for drive-level feedback data.
        struct DriveFeedbackData
        {
            std::array<hand_control::merai::DriveStatus, MAX_SERVO_DRIVES> status; // Status for each drive
        };

        //====================================================
        // Controller Data Structures
        //====================================================

        // Structure for sending a controller command.
        struct ControllerCommand
        {
            bool requestSwitch = false;                                                               // Flag to request a controller switch
            hand_control::merai::ControllerID controllerId = hand_control::merai::ControllerID::NONE; // Target controller ID
        };

        // Structure for receiving controller feedback.
        struct ControllerFeedback
        {
            hand_control::merai::ControllerFeedbackState feedbackState = hand_control::merai::ControllerFeedbackState::IDLE;
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
            hand_control::merai::UserMode desiredMode = hand_control::merai::UserMode::HOMING; // Desired user mode
        };

        // Structure for feedback provided to the user.
        struct UserFeedback
        {
            hand_control::merai::AppState currentState = hand_control::merai::AppState::HOMING; // Current application state
        };

        //====================================================
        // Double Buffer Template and Real-Time Memory Layout
        //====================================================

        template <typename T>
        struct DoubleBuffer
        {
            T buffer[2];                    // Two buffers for double buffering
            std::atomic<int> frontIndex{0}; // Index of the buffer currently being used for reading (0 or 1)
        };

        // Layout of the shared real-time memory.
        // It aggregates all the double buffers used to exchange various data types.
        struct RTMemoryLayout
        {
            // Servo fieldbus data: both transmit and receive PDOs.
            DoubleBuffer<ServoSharedData> servoBuffer;

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

    } // namespace merai
} // namespace hand_control
