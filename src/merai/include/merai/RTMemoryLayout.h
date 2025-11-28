#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <type_traits>

// Include enumerations for DriveCommand, DriveStatus, ControllerID, etc.
#include "merai/Enums.h"

namespace merai
{

    //====================================================
    // Constants
    //====================================================

    // Maximum number of servo drives supported by the system
    constexpr int MAX_SERVO_DRIVES = 12;
    constexpr uint32_t RT_MEMORY_MAGIC = 0x52544D4D; // 'RTMM'
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

    //====================================================
    // Drive Data Structures
    //====================================================

    // Structure for drive-level command data.
    struct DriveCommandData
    {
        std::array<merai::DriveCommand, MAX_SERVO_DRIVES> commands; // Commands for each drive
    };

    // Structure for drive-level feedback data.
    struct DriveFeedbackData
    {
        std::array<merai::DriveStatus, MAX_SERVO_DRIVES> status; // Status for each drive
    };

    //====================================================
    // Controller Data Structures
    //====================================================

    // Structure for sending a controller command.
    struct ControllerCommand
    {
        bool requestSwitch = false;                                   // Flag to request a controller switch
        merai::ControllerID controllerId = merai::ControllerID::NONE; // Target controller ID
    };

    // Structure for receiving controller feedback.
    struct ControllerFeedback
    {
        merai::ControllerFeedbackState feedbackState = merai::ControllerFeedbackState::IDLE;
        merai::ControllerID activeControllerId = merai::ControllerID::NONE;
        merai::ControllerSwitchResult switchResult = merai::ControllerSwitchResult::IDLE;
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
        bool eStop = false;                                    // Emergency stop flag
        bool resetFault = false;                               // Reset fault command
        bool shutdownRequest = false;                          // Shutdown request flag
        merai::UserMode desiredMode = merai::UserMode::HOMING; // Desired user mode
    };

    // Structure for feedback provided to the user.
    struct UserFeedback
    {
        merai::AppState currentState = merai::AppState::HOMING; // Current application state
    };

    //====================================================
    // Double Buffer Template and Real-Time Memory Layout
    //====================================================

    template <typename T>
    struct alignas(64) DoubleBuffer
    {
        T buffer[2];                     // two buffers for double buffering
        std::atomic<int> activeIndex{0}; // index of the currently published buffer (0 or 1)
    };

    // Layout of the shared real-time memory.
    // It aggregates all the double buffers used to exchange various data types.
    struct alignas(64) RTMemoryLayout
    {
        uint32_t magic = RT_MEMORY_MAGIC;
        uint32_t version = RT_MEMORY_VERSION;

        // Servo fieldbus data: separate buffers for Tx feedback (fieldbus-owned) and Rx commands (control-owned).
        DoubleBuffer<std::array<ServoTxPdo, MAX_SERVO_DRIVES>> servoTxBuffer;
        DoubleBuffer<std::array<ServoRxPdo, MAX_SERVO_DRIVES>> servoRxBuffer;

        // High-level joint data: separate buffers for commands and feedback.
        DoubleBuffer<std::array<JointCommandData, MAX_SERVO_DRIVES>> jointCommandBuffer;
        DoubleBuffer<std::array<JointFeedbackData, MAX_SERVO_DRIVES>> jointFeedbackBuffer;

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

    // SHM contract: layout must stay trivially copyable/POD-friendly.
    static_assert(std::is_trivially_copyable<RTMemoryLayout>::value, "RTMemoryLayout must be trivially copyable");
} // namespace merai
