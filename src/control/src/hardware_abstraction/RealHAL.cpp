#include <stdexcept>
#include <cmath> // for M_PI or other math constants

#include "control/hardware_abstraction/RealHAL.h"
#include "merai/RTIpc.h"

namespace control
{

    //------------------------------------------------------------------------------
    // Inline helper functions for reading
    //------------------------------------------------------------------------------

    /// Convert raw servo position to radians (plus offset).
    static inline double convertPositionRawToRad(double rawPos, const merai::JointConfig &cfg)
    {
        // Example: rawPos * 0.001 => rad, then apply gear ratio, direction, offset
        // return (rawPos * 0.001) * cfg.gear_ratio * cfg.axis_direction + cfg.position_offset;
        return ((rawPos - cfg.position_offset) / cfg.drive.encoder_counts) * 2 * M_PI * cfg.drive.axis_direction;
    }

    /// Convert raw servo velocity to rad/s.
    static inline double convertVelocityRawToRad(double rawVel, const merai::JointConfig &cfg)
    {
        // Example: rawVel * 0.0001 => rad/s, then apply gear ratio, direction
        return (rawVel * 0.0001) * cfg.drive.gear_ratio * cfg.drive.axis_direction;
    }

    /// Convert raw servo torque to Nm.
    static inline double convertTorqueRawToNm(double rawTorque, const merai::JointConfig &cfg)
    {
        // Example: rawTorque * 0.01 => Nm, then apply gear ratio, direction
        return (rawTorque / 1000) * cfg.drive.rated_torque * cfg.drive.gear_ratio * cfg.drive.torque_axis_direction;
    }

    //------------------------------------------------------------------------------
    // Inline helper functions for writing
    //------------------------------------------------------------------------------

    /// Convert from target position in radians to raw servo position units.
    static inline int32_t convertPositionRadToRaw(double posRad, const merai::JointConfig &cfg)
    {
        // Reverse of convertPositionRawToRad: (posRad - offset) / (gearRatio * axisDir)
        // Example scaling: 1 raw => 0.001 rad => so multiply by 1000
        double desPos = posRad / (2 * M_PI) * cfg.drive.encoder_counts * cfg.drive.axis_direction + cfg.position_offset;
        return static_cast<int32_t>(desPos);
    }

    /// Convert from target torque in Nm to raw servo torque units.
    static inline int16_t convertTorqueNmToRaw(double torqueNm, const merai::JointConfig &cfg)
    {
        // Reverse of convertTorqueRawToNm: 1 raw => 0.01 Nm
        // So multiply by 100, plus handle gear ratio/direction
        double adjTorque = torqueNm / (cfg.drive.gear_ratio * cfg.drive.torque_axis_direction);
        return static_cast<int16_t>(adjTorque * 100.0);
    }

    //------------------------------------------------------------------------------
    // Constructor / Destructor
    //------------------------------------------------------------------------------

    RealHAL::RealHAL(
        merai::RTMemoryLayout *rtLayout,
        const merai::ParameterServer *paramServerPtr,
        merai::multi_ring_logger_memory *loggerMem)
        : rtLayout_(rtLayout),
          paramServerPtr_(paramServerPtr),
          loggerMem_(loggerMem)
    {
        // Basic pointer checks
        if (!rtLayout_)
        {
            throw std::runtime_error("[RealHAL] Null rtLayout passed to constructor.");
        }
        if (!paramServerPtr_)
        {
            throw std::runtime_error("[RealHAL] Null paramServerPtr passed to constructor.");
        }

        // 1) Determine how many drives from paramServerPtr_
        driveCount_ = paramServerPtr_->driveCount;
        if (driveCount_ > merai::MAX_SERVO_DRIVES)
        {
            merai::log_warn(loggerMem_, "Control", 2200, "[RealHAL] driveCount exceeds MAX_SERVO_DRIVES; capping");
            driveCount_ = merai::MAX_SERVO_DRIVES;
        }
    }

    bool RealHAL::init()
    {
        merai::log_info(loggerMem_, "Control", 2201, "[RealHAL] init() called");

        // 2) Load static servo/joint config from ParameterServer
        for (int i = 0; i < driveCount_; ++i)
        {
            localJointConfigs_[i] = paramServerPtr_->joints[i];
        }

        // 3) Zero out local arrays
        for (int i = 0; i < driveCount_; ++i)
        {
            localJointControlCommand_[i] = {};
            localJointControlFeedback_[i] = {};
            localJointMotionCommand_[i] = {};
            localJointMotionFeedback_[i] = {};
            localJointFeedbackIO_[i] = {};
        }

        merai::log_info(loggerMem_, "Control", 2202, "[RealHAL] init() complete");
        return true;
    }

    //------------------------------------------------------------------------------
    // Public BaseHAL Methods
    //------------------------------------------------------------------------------

    bool RealHAL::read()
    {
        // 1) Safety check
        if (!rtLayout_)
        {
            merai::log_error(loggerMem_, "Control", 2203, "[RealHAL] read() failed: rtLayout_ is null");
            return false;
        }

        // 2) Acquire the latest servo Tx array from shared memory
        std::array<merai::ServoTxPdo,
                   merai::MAX_SERVO_DRIVES>
            servoSnapshot{};
        merai::read_snapshot(rtLayout_->servoTxBuffer, servoSnapshot);

        if (!mapAndConvertServoTxData(servoSnapshot))
        {
            merai::log_error(loggerMem_, "Control", 2204, "[RealHAL] mapAndConvertServoTxData() failed");
            return false;
        }

        return true;
    }

    bool RealHAL::write()
    {
        // 1) Safety check
        if (!rtLayout_)
        {
            merai::log_error(loggerMem_, "Control", 2205, "[RealHAL] write() failed: rtLayout_ is null");
            return false;
        }

        int servoBackIdx = merai::back_index(rtLayout_->servoRxBuffer);
        auto &servoRxArray = rtLayout_->servoRxBuffer.buffer[servoBackIdx];

        if (!convertAndMapJointCommandsToServoRxData(servoRxArray))
        {
            merai::log_error(loggerMem_, "Control", 2206, "[RealHAL] convertAndMapJointCommandsToServoRxData() failed");
            return false;
        }

        // Publish new commands for the fieldbus side
        merai::publish(rtLayout_->servoRxBuffer, servoBackIdx);

        return true;
    }

    //------------------------------------------------------------------------------
    // Private Helpers (Read path)
    //------------------------------------------------------------------------------

    bool RealHAL::mapAndConvertServoTxData(
        const std::array<merai::ServoTxPdo,
                         merai::MAX_SERVO_DRIVES> &servoTxArray)
    {
        // Single pass: copy raw -> local arrays, then convert to SI

        for (int i = 0; i < driveCount_; ++i)
        {
            // 1) Control feedback
            localJointControlFeedback_[i].statusWord = servoTxArray[i].ctrl.statusWord;

            // 2) Extract raw motion data
            double rawPos = static_cast<double>(servoTxArray[i].motion.positionActual);
            double rawVel = static_cast<double>(servoTxArray[i].motion.velocityActual);
            double rawTorque = static_cast<double>(servoTxArray[i].motion.torqueActual);

            // 3) Convert to SI
            localJointMotionFeedback_[i].positionActual =
                convertPositionRawToRad(rawPos, localJointConfigs_[i]);
            localJointMotionFeedback_[i].velocityActual =
                convertVelocityRawToRad(rawVel, localJointConfigs_[i]);
            localJointMotionFeedback_[i].torqueActual =
                convertTorqueRawToNm(rawTorque, localJointConfigs_[i]);

            // 4) I/O feedback
            if (i == driveCount_ - 1)
            {

                uint32_t digitalInputs = servoTxArray[i].io.digitalInputs;
                localJointFeedbackIO_[i].digitalInputClutch = (digitalInputs & 0x01) != 0; // example bit
                localJointFeedbackIO_[i].digitalInputThumb = (digitalInputs & 0x02) != 0;  // example bit

                localJointFeedbackIO_[i].analogInputPinch =
                    static_cast<double>(servoTxArray[i].io.analogInput);
            }
        }
        return true;
    }

    //------------------------------------------------------------------------------
    // Private Helpers (Write path)
    //------------------------------------------------------------------------------

    bool RealHAL::convertAndMapJointCommandsToServoRxData(
        std::array<merai::ServoRxPdo,
                   merai::MAX_SERVO_DRIVES> &servoRxArray)
    {
        // Single pass: read local SI commands, convert to raw, write into servoRxArray
        for (int i = 0; i < driveCount_; ++i)
        {
            // 1) Retrieve local joint commands
            const auto &ctrlCmd = localJointControlCommand_[i];
            const auto &motionCmd = localJointMotionCommand_[i];
            const auto &cfg = localJointConfigs_[i];

            // 2) Convert to raw servo units
            double desPosRad = motionCmd.targetPosition;
            double desTorqueNm = motionCmd.targetTorque;

            int32_t rawPos = convertPositionRadToRaw(desPosRad, cfg);
            int16_t rawTorque = convertTorqueNmToRaw(desTorqueNm, cfg);
            uint16_t rawMaxTorque = static_cast<uint16_t>(convertTorqueNmToRaw(cfg.drive.rated_torque, cfg));

            // 3) Fill servoRx data based on requested mode
            servoRxArray[i].ctrl.controlWord = ctrlCmd.controlWord;
            servoRxArray[i].motion.modeOfOperation = motionCmd.modeOfOperation;
            servoRxArray[i].motion.maxTorque = rawMaxTorque;

            switch (motionCmd.modeOfOperation)
            {
            // Cyclic synchronous position
            case 8:
                servoRxArray[i].motion.targetPosition = rawPos;
                servoRxArray[i].motion.targetTorque = 0;
                break;
            // Cyclic synchronous torque
            case 10:
                servoRxArray[i].motion.targetPosition = 0;
                servoRxArray[i].motion.targetTorque = rawTorque;
                break;
            // Default / unsupported: fall back to position mode to avoid sending junk
            default:
                servoRxArray[i].motion.modeOfOperation = 8;
                servoRxArray[i].motion.targetPosition = rawPos;
                servoRxArray[i].motion.targetTorque = 0;
                break;
            }

            // 4) If you have joint-level digital/analog outputs, you can map them here
            //    e.g. servoRxArray[i].io.digitalOutputs = ...
        }
        return true;
    }

    bool RealHAL::publishJointFeedbackToShm()
    {
        if (!rtLayout_)
        {
            return false;
        }

        int backIdx = merai::back_index(rtLayout_->jointFeedbackBuffer);
        auto &feedbackArray = rtLayout_->jointFeedbackBuffer.buffer[backIdx];
        feedbackArray.fill(merai::JointFeedbackData{});

        for (int i = 0; i < driveCount_; ++i)
        {
            feedbackArray[i].control = localJointControlFeedback_[i];
            feedbackArray[i].motion = localJointMotionFeedback_[i];
            feedbackArray[i].io = localJointFeedbackIO_[i];
        }

        merai::publish(rtLayout_->jointFeedbackBuffer, backIdx);
        return true;
    }

} // namespace control
