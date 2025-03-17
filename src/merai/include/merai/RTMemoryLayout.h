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
            uint16_t controlWord = 0;    // Command control word
            uint8_t modeOfOperation = 0; // Operating mode indicator
        };

        // Motion-related fields (e.g. target torque, position, velocity)
        struct ServoRxMotion
        {
            int16_t targetTorque = 0;   // Desired torque value
            int32_t targetPosition = 0; // Desired position value
            int32_t targetVelocity = 0; // Desired velocity value
        };

        // IO-related fields (digital/analog IO, if any)
        // Currently empty, but reserved for future expansion.
        struct ServoRxIO
        {
            // e.g. digitalOutputs, analogOutput, etc.
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
            // If needed, you can also store a "modeOfOperationDisplay" here, etc.
        };

        // Motion-related feedback fields (e.g. position, velocity, torque actual)
        struct ServoTxMotion
        {
            int32_t positionActual = 0; // Actual position feedback
            int32_t velocityActual = 0; // Actual velocity feedback
            int16_t torqueActual = 0;   // Actual torque feedback
        };

        // IO-related feedback fields (digital/analog IO inputs, if any)
        // Again, empty for future use
        struct ServoTxIO
        {
            // e.g. digitalInputs, analogInput, etc.
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

        //====================================================
        // Joint Data Structures
        //====================================================

        // Structure for I/O signals related to each joint.
        struct JointIO
        {
            // Example: digital or analog inputs/outputs at the joint level.
            // You can rename or expand as needed for your hardware.

            bool digitalInputA = false; // Example digital input flag
            bool digitalInputB = false; // Another digital input
            double analogInput = 0.0;   // Example analog reading

            bool digitalOutputA = false; // Example digital output (if the joint has any)
            double analogOutput = 0.0;   // Example analog output command
        };

        // Structure for commanding a joint.
        struct JointCommand
        {
            double position = 0.0; // Desired joint position
            double velocity = 0.0; // Desired joint velocity
            double torque = 0.0;   // Desired joint torque
        };

        // Structure representing the current state of a joint.
        struct JointState
        {
            double position = 0.0; // Measured joint position
            double velocity = 0.0; // Measured joint velocity
            double torque = 0.0;   // Measured joint torque
        };

        // Aggregates command, state feedback, and I/O for all joints.
        struct JointData
        {
            std::array<JointCommand, MAX_SERVO_DRIVES> commands; // Commands for each joint
            std::array<JointState, MAX_SERVO_DRIVES> states;     // States for each joint
            std::array<JointIO, MAX_SERVO_DRIVES> io;            // I/O signals for each joint
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
            hand_control::merai::ControllerFeedbackState feedback = hand_control::merai::ControllerFeedbackState::IDLE;
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
