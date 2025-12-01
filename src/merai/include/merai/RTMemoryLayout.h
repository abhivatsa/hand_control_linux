#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "merai/Enums.h"

namespace merai
{

    //====================================================
    // Constants
    //====================================================

    // Maximum number of servo drives supported by the system
    constexpr std::size_t  MAX_SERVO_DRIVES   = 12;
    constexpr std::uint32_t RT_MEMORY_MAGIC   = 0x52544D4D; // 'RTMM'
    constexpr std::uint32_t RT_MEMORY_VERSION = 1;

    //====================================================
    // Servo Fieldbus Data Structures
    //====================================================

    // ----------- Servo Rx (Commands -> from Controller to Servo) -----------

    struct ServoRxControl
    {
        std::uint16_t controlWord = 0; // Command control word
    };

    struct ServoRxMotion
    {
        std::uint8_t  modeOfOperation = 8; // Operating mode indicator
        std::int16_t  targetTorque    = 0; // 0x6071
        std::int32_t  targetPosition  = 0; // 0x607A
        std::uint16_t maxTorque       = 0; // 0x6072
    };

    struct ServoRxIO
    {
        std::uint32_t digitalOutputs = 0; // 0x60FE:01
    };

    struct ServoRxPdo
    {
        ServoRxControl ctrl;
        ServoRxMotion  motion;
        ServoRxIO      io;
    };

    // ----------- Servo Tx (Feedback -> from Servo to Controller) -----------

    struct ServoTxControl
    {
        std::uint16_t statusWord = 0; // Feedback status word
    };

    struct ServoTxMotion
    {
        std::int32_t positionActual = 0; // Actual position feedback
        std::int32_t velocityActual = 0; // Actual velocity feedback
        std::int16_t torqueActual   = 0; // 0x6077
    };

    struct ServoTxIO
    {
        std::uint32_t digitalInputs = 0; // 0x60FD
        std::uint16_t analogInput   = 0; // 0x2401
        std::uint16_t error_code    = 0; // 0x603F
    };

    struct ServoTxPdo
    {
        ServoTxControl ctrl;
        ServoTxMotion  motion;
        ServoTxIO      io;
    };

    //====================================================
    // Joint Command / Feedback Structures
    //====================================================

    struct JointControlCommand
    {
        std::uint16_t controlWord = 0;
    };

    struct JointMotionCommand
    {
        double        targetPosition  = 0.0;
        double        targetTorque    = 0.0;
        std::uint8_t  modeOfOperation = 8; // tightly coupled to servo mode
    };

    struct JointCommandIO
    {
        // reserved for future digital / analog outputs
    };

    struct JointCommandData
    {
        JointControlCommand control;
        JointMotionCommand  motion;
        JointCommandIO      io;
    };

    struct JointControlFeedback
    {
        std::uint16_t statusWord = 0;
    };

    struct JointMotionFeedback
    {
        double positionActual = 0.0;
        double velocityActual = 0.0;
        double torqueActual   = 0.0;
    };

    struct JointFeedbackIO
    {
        bool   digitalInputClutch = false;
        bool   digitalInputThumb  = false;
        double analogInputPinch   = 0.0;
    };

    struct JointFeedbackData
    {
        JointControlFeedback control;
        JointMotionFeedback  motion;
        JointFeedbackIO      io;
    };

    //====================================================
    // Drive Data Structures
    //====================================================

    struct DriveCommandData
    {
        std::array<merai::DriveCommand, MAX_SERVO_DRIVES> commands{};
    };

    struct DriveFeedbackData
    {
        std::array<merai::DriveStatus, MAX_SERVO_DRIVES> status{};
    };

    //====================================================
    // Controller Data Structures
    //====================================================

    struct ControllerCommand
    {
        bool                requestSwitch = false;
        merai::ControllerID controllerId  = merai::ControllerID::NONE;
    };

    struct ControllerFeedback
    {
        merai::ControllerFeedbackState feedbackState      = merai::ControllerFeedbackState::IDLE;
        merai::ControllerID            activeControllerId = merai::ControllerID::NONE;
        merai::ControllerSwitchResult  switchResult       = merai::ControllerSwitchResult::IDLE;

        std::uint64_t loopOverrunCount = 0;
        std::uint64_t halErrorCount    = 0;

        bool          loopOverrun            = false;
        bool          controllerCommandFresh = true;
        bool          driveCommandFresh      = true;
    };

    //====================================================
    // User Command and Feedback Structures
    //====================================================

    struct UserCommands
    {
        bool            eStop           = false;
        bool            resetFault      = false;
        bool            shutdownRequest = false;
        merai::UserMode desiredMode     = merai::UserMode::HOMING;
    };

    struct UserFeedback
    {
        merai::AppState currentState = merai::AppState::HOMING;
    };

    //====================================================
    // Double Buffer Template and Real-Time Memory Layout
    //====================================================

    // Single-producer, multi-consumer double buffer.
    // Use RTIpc helpers (back_index / publish / read_snapshot) to access.
    template <typename T>
    struct alignas(64) DoubleBuffer
    {
        static_assert(std::is_trivially_copyable<T>::value,
                      "DoubleBuffer<T> requires T to be trivially copyable");

        T                           buffer[2];          // two buffers for double buffering
        std::atomic<int>           activeIndex{0};     // index of the currently published buffer (0 or 1)
        std::atomic<std::uint32_t> sequence{0};        // seqlock-style version counter
    };

    struct alignas(64) RTMemoryLayout
    {
        std::uint32_t magic   = RT_MEMORY_MAGIC;
        std::uint32_t version = RT_MEMORY_VERSION;

        // Servo fieldbus data: Tx feedback (fieldbus-owned) and Rx commands (control-owned).
        DoubleBuffer<std::array<ServoTxPdo, MAX_SERVO_DRIVES>> servoTxBuffer;
        DoubleBuffer<std::array<ServoRxPdo, MAX_SERVO_DRIVES>> servoRxBuffer;

        // High-level joint data.
        DoubleBuffer<std::array<JointCommandData,   MAX_SERVO_DRIVES>> jointCommandBuffer;
        DoubleBuffer<std::array<JointFeedbackData, MAX_SERVO_DRIVES>> jointFeedbackBuffer;

        // Drive-level command and feedback buffers.
        DoubleBuffer<DriveCommandData>  driveCommandBuffer;
        DoubleBuffer<DriveFeedbackData> driveFeedbackBuffer;

        // Controller-level command and feedback buffers.
        DoubleBuffer<ControllerCommand>  controllerCommandBuffer;
        DoubleBuffer<ControllerFeedback> controllerFeedbackBuffer;

        // User commands and feedback buffers.
        DoubleBuffer<UserCommands> userCommandsBuffer;
        DoubleBuffer<UserFeedback> userFeedbackBuffer;
    };

    // SHM contract: layout must stay trivially copyable/POD-friendly.
    static_assert(std::is_trivially_copyable<RTMemoryLayout>::value,
                  "RTMemoryLayout must be trivially copyable");
    static_assert(std::is_standard_layout<RTMemoryLayout>::value,
                  "RTMemoryLayout must be standard layout");

} // namespace merai
