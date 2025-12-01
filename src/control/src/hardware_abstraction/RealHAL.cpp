#include <stdexcept>
#include <cmath>

#include "control/hardware_abstraction/RealHAL.h"
#include "merai/RTIpc.h"

namespace control
{
    namespace
    {
        constexpr double PI = 3.14159265358979323846;

        // Convert raw servo position to radians.
        inline double convertPositionRawToRad(double rawPos,
                                              const merai::JointConfig &cfg)
        {
            // Example: rawPos in encoder counts.
            // One full revolution = encoder_counts -> 2*pi radians.
            return ((rawPos - cfg.position_offset) /
                    static_cast<double>(cfg.drive.encoder_counts)) *
                   (2.0 * PI) *
                   static_cast<double>(cfg.drive.axis_direction);
        }

        // Convert raw servo velocity to rad/s.
        inline double convertVelocityRawToRad(double rawVel,
                                              const merai::JointConfig &cfg)
        {
            // Example scaling; adjust to actual drive semantics as needed.
            return (rawVel * 0.0001) *
                   cfg.drive.gear_ratio *
                   static_cast<double>(cfg.drive.axis_direction);
        }

        // Convert raw servo torque to Nm.
        inline double convertTorqueRawToNm(double rawTorque,
                                           const merai::JointConfig &cfg)
        {
            // Example: rawTorque is in milli-units of rated torque.
            return (rawTorque / 1000.0) *
                   cfg.drive.rated_torque *
                   cfg.drive.gear_ratio *
                   static_cast<double>(cfg.drive.torque_axis_direction);
        }

        // Convert from target position in radians to raw servo units.
        inline int32_t convertPositionRadToRaw(double posRad,
                                               const merai::JointConfig &cfg)
        {
            double desPosCounts =
                (posRad / (2.0 * PI)) *
                static_cast<double>(cfg.drive.encoder_counts) *
                static_cast<double>(cfg.drive.axis_direction) +
                cfg.position_offset;

            return static_cast<int32_t>(desPosCounts);
        }

        // Convert from target torque in Nm to raw servo units.
        inline int16_t convertTorqueNmToRaw(double torqueNm,
                                            const merai::JointConfig &cfg)
        {
            // Reverse of convertTorqueRawToNm; approximate.
            double adjTorque =
                torqueNm / (cfg.drive.gear_ratio *
                            static_cast<double>(cfg.drive.torque_axis_direction));

            return static_cast<int16_t>(adjTorque * 100.0);
        }
    } // namespace

    //------------------------------------------------------------------------------
    // Constructor / init
    //------------------------------------------------------------------------------
    RealHAL::RealHAL(merai::RTMemoryLayout           *rtLayout,
                     const merai::ParameterServer    *paramServerPtr,
                     merai::multi_ring_logger_memory *loggerMem)
        : rtLayout_(rtLayout),
          paramServerPtr_(paramServerPtr),
          loggerMem_(loggerMem)
    {
        if (!rtLayout_)
        {
            throw std::runtime_error("[RealHAL] Null rtLayout passed to constructor.");
        }
        if (!paramServerPtr_)
        {
            throw std::runtime_error("[RealHAL] Null paramServerPtr passed to constructor.");
        }

        driveCount_ = paramServerPtr_->driveCount;
        if (driveCount_ > merai::MAX_SERVO_DRIVES)
        {
            driveCount_ = merai::MAX_SERVO_DRIVES;
            if (loggerMem_)
            {
                merai::log_warn(loggerMem_, "Control", 2200,
                                "[RealHAL] driveCount exceeds MAX_SERVO_DRIVES; capping");
            }
        }
    }

    bool RealHAL::init()
    {
        if (loggerMem_)
        {
            merai::log_info(loggerMem_, "Control", 2201,
                            "[RealHAL] init() called");
        }

        // Copy static joint configs
        for (int i = 0; i < driveCount_; ++i)
        {
            localJointConfigs_[i] = paramServerPtr_->joints[i];
        }

        // Zero local arrays
        for (int i = 0; i < driveCount_; ++i)
        {
            localJointControlCommand_[i]  = {};
            localJointControlFeedback_[i] = {};
            localJointMotionCommand_[i]   = {};
            localJointMotionFeedback_[i]  = {};
            localJointFeedbackIO_[i]      = {};
            localJointCommandIO_[i]       = {};
        }

        if (loggerMem_)
        {
            merai::log_info(loggerMem_, "Control", 2202,
                            "[RealHAL] init() complete");
        }
        return true;
    }

    //------------------------------------------------------------------------------
    // BaseHAL interface
    //------------------------------------------------------------------------------
    bool RealHAL::read()
    {
        if (!rtLayout_)
        {
            if (loggerMem_)
            {
                merai::log_error(loggerMem_, "Control", 2203,
                                 "[RealHAL] read() failed: rtLayout_ is null");
            }
            return false;
        }

        std::array<merai::ServoTxPdo,
                   merai::MAX_SERVO_DRIVES> servoSnapshot{};
        merai::read_snapshot(rtLayout_->servoTxBuffer, servoSnapshot);

        if (!mapAndConvertServoTxData(servoSnapshot))
        {
            if (loggerMem_)
            {
                merai::log_error(loggerMem_, "Control", 2204,
                                 "[RealHAL] mapAndConvertServoTxData() failed");
            }
            return false;
        }

        return true;
    }

    bool RealHAL::write()
    {
        if (!rtLayout_)
        {
            if (loggerMem_)
            {
                merai::log_error(loggerMem_, "Control", 2205,
                                 "[RealHAL] write() failed: rtLayout_ is null");
            }
            return false;
        }

        int backIdx = merai::back_index(rtLayout_->servoRxBuffer);
        auto &servoRxArray = rtLayout_->servoRxBuffer.buffer[backIdx];

        if (!convertAndMapJointCommandsToServoRxData(servoRxArray))
        {
            if (loggerMem_)
            {
                merai::log_error(loggerMem_, "Control", 2206,
                                 "[RealHAL] convertAndMapJointCommandsToServoRxData() failed");
            }
            return false;
        }

        merai::publish(rtLayout_->servoRxBuffer, backIdx);
        return true;
    }

    //------------------------------------------------------------------------------
    // Read path: servo Tx → local feedback
    //------------------------------------------------------------------------------
    bool RealHAL::mapAndConvertServoTxData(
        const std::array<merai::ServoTxPdo,
                         merai::MAX_SERVO_DRIVES> &servoTxArray)
    {
        for (int i = 0; i < driveCount_; ++i)
        {
            const auto &cfg = localJointConfigs_[i];
            const auto &tx  = servoTxArray[i];

            // Control feedback
            localJointControlFeedback_[i].statusWord = tx.ctrl.statusWord;

            // Raw motion data
            double rawPos    = static_cast<double>(tx.motion.positionActual);
            double rawVel    = static_cast<double>(tx.motion.velocityActual);
            double rawTorque = static_cast<double>(tx.motion.torqueActual);

            // Convert to SI
            localJointMotionFeedback_[i].positionActual =
                convertPositionRawToRad(rawPos, cfg);
            localJointMotionFeedback_[i].velocityActual =
                convertVelocityRawToRad(rawVel, cfg);
            localJointMotionFeedback_[i].torqueActual =
                convertTorqueRawToNm(rawTorque, cfg);

            // IO feedback (example: only last joint uses IO)
            if (i == driveCount_ - 1)
            {
                uint32_t digitalInputs = tx.io.digitalInputs;
                localJointFeedbackIO_[i].digitalInputClutch = (digitalInputs & 0x01) != 0;
                localJointFeedbackIO_[i].digitalInputThumb  = (digitalInputs & 0x02) != 0;
                localJointFeedbackIO_[i].analogInputPinch   =
                    static_cast<double>(tx.io.analogInput);
            }
        }
        return true;
    }

    //------------------------------------------------------------------------------
    // Write path: local commands (SI) → servo Rx
    //------------------------------------------------------------------------------
    bool RealHAL::convertAndMapJointCommandsToServoRxData(
        std::array<merai::ServoRxPdo,
                   merai::MAX_SERVO_DRIVES> &servoRxArray)
    {
        for (int i = 0; i < driveCount_; ++i)
        {
            const auto &ctrlCmd   = localJointControlCommand_[i];
            const auto &motionCmd = localJointMotionCommand_[i];
            const auto &cfg       = localJointConfigs_[i];

            double desPosRad    = motionCmd.targetPosition;
            double desTorqueNm  = motionCmd.targetTorque;

            int32_t rawPos      = convertPositionRadToRaw(desPosRad, cfg);
            int16_t rawTorque   = convertTorqueNmToRaw(desTorqueNm, cfg);
            uint16_t rawMaxTorque =
                static_cast<uint16_t>(
                    convertTorqueNmToRaw(cfg.drive.rated_torque, cfg));

            auto &rx = servoRxArray[i];

            rx.ctrl.controlWord       = ctrlCmd.controlWord;
            rx.motion.modeOfOperation = motionCmd.modeOfOperation;
            rx.motion.maxTorque       = rawMaxTorque;

            switch (motionCmd.modeOfOperation)
            {
            // Cyclic synchronous position
            case 8:
                rx.motion.targetPosition = rawPos;
                rx.motion.targetTorque   = 0;
                break;

            // Cyclic synchronous torque
            case 10:
                rx.motion.targetPosition = 0;
                rx.motion.targetTorque   = rawTorque;
                break;

            default:
                // Default / unsupported: fall back to position mode
                rx.motion.modeOfOperation = 8;
                rx.motion.targetPosition  = rawPos;
                rx.motion.targetTorque    = 0;
                break;
            }

            // IO outputs can be mapped here if needed (digitalOutputs, etc.).
        }
        return true;
    }

    //------------------------------------------------------------------------------
    // Publish joint feedback to SHM
    //------------------------------------------------------------------------------
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
            feedbackArray[i].motion  = localJointMotionFeedback_[i];
            feedbackArray[i].io      = localJointFeedbackIO_[i];
        }

        merai::publish(rtLayout_->jointFeedbackBuffer, backIdx);
        return true;
    }

} // namespace control
