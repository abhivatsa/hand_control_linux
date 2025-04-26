#include <iostream>
#include <stdexcept>
#include <cmath> // for M_PI or other math constants

#include "control/hardware_abstraction/RealHAL.h"

namespace hand_control
{
    namespace control
    {

        //------------------------------------------------------------------------------
        // Inline helper functions for reading
        //------------------------------------------------------------------------------

        /// Convert raw servo position to radians (plus offset).
        static inline double convertPositionRawToRad(double rawPos, const hand_control::merai::JointConfig &cfg)
        {
            // Example: rawPos * 0.001 => rad, then apply gear ratio, direction, offset
            // return (rawPos * 0.001) * cfg.gear_ratio * cfg.axis_direction + cfg.position_offset;
            return ((rawPos - cfg.position_offset)/cfg.encoder_counts) * 2 * M_PI * cfg.axis_direction;
        }

        /// Convert raw servo velocity to rad/s.
        static inline double convertVelocityRawToRad(double rawVel, const hand_control::merai::JointConfig &cfg)
        {
            // Example: rawVel * 0.0001 => rad/s, then apply gear ratio, direction
            return (rawVel * 0.0001) * cfg.gear_ratio * cfg.axis_direction;
        }

        /// Convert raw servo torque to Nm.
        static inline double convertTorqueRawToNm(double rawTorque, const hand_control::merai::JointConfig &cfg)
        {
            // Example: rawTorque * 0.01 => Nm, then apply gear ratio, direction
            return (rawTorque / 1000) * cfg.rated_torque * cfg.gear_ratio * cfg.torque_axis_direction;
        }

        /// Convert raw servo torque to Nm.
        static inline double convertCurrentRawToNm(double rawCurrent, const hand_control::merai::JointConfig &cfg)
        {
            // Example: rawTorque * 0.01 => Nm, then apply gear ratio, direction
            return (rawCurrent) * cfg.torque_constant * cfg.gear_ratio * cfg.torque_axis_direction;
        }

        //------------------------------------------------------------------------------
        // Inline helper functions for writing
        //------------------------------------------------------------------------------

        /// Convert from target position in radians to raw servo position units.
        static inline int32_t convertPositionRadToRaw(double posRad, const hand_control::merai::JointConfig &cfg)
        {
            // Reverse of convertPositionRawToRad: (posRad - offset) / (gearRatio * axisDir)
            // Example scaling: 1 raw => 0.001 rad => so multiply by 1000
            double desPos = posRad/(2 * M_PI) * cfg.encoder_counts * cfg.axis_direction + cfg.position_offset;
            return static_cast<int32_t>(desPos);
        }

        /// Convert from target torque in Nm to raw servo torque units.
        static inline int16_t convertTorqueNmToRaw(double torqueNm, const hand_control::merai::JointConfig &cfg)
        {
            // Reverse of convertTorqueRawToNm: 1 raw => 0.01 Nm
            // So multiply by 100, plus handle gear ratio/direction
            double adjTorque = torqueNm / (cfg.gear_ratio * cfg.torque_axis_direction);
            return static_cast<int16_t>(adjTorque * 100.0);
        }

        /// Convert from target torque in Nm to raw servo torque units.
        static inline float convertCurrentNmToRaw(double torqueNm, const hand_control::merai::JointConfig &cfg)
        {
            // Reverse of convertTorqueRawToNm: 1 raw => 0.01 Nm
            // So multiply by 100, plus handle gear ratio/direction
            return torqueNm / (cfg.gear_ratio * cfg.torque_axis_direction * cfg.torque_constant);
        }

        //------------------------------------------------------------------------------
        // Constructor / Destructor
        //------------------------------------------------------------------------------

        RealHAL::RealHAL(
            hand_control::merai::RTMemoryLayout *rtLayout,
            const hand_control::merai::ParameterServer *paramServerPtr,
            hand_control::merai::multi_ring_logger_memory *loggerMem)
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
            if (driveCount_ > hand_control::merai::MAX_SERVO_DRIVES)
            {
                std::cerr << "[RealHAL] Requested driveCount (" << driveCount_
                          << ") exceeds MAX_SERVO_DRIVES; capping.\n";
                driveCount_ = hand_control::merai::MAX_SERVO_DRIVES;
            }

        }

        bool RealHAL::init()
        {
            std::cout << "[RealHAL] init() called.\n";

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

            std::cout << "[RealHAL] init() complete. driveCount_ = " << driveCount_ << "\n";
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
                std::cerr << "[RealHAL] read() failed: rtLayout_ is null.\n";
                return false;
            }

            // 2) Acquire the latest servo Tx array from shared memory
            int servoFrontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &servoTxArray = rtLayout_->servoBuffer.buffer[servoFrontIdx].tx;

            // 3) Map and convert from servo Tx data → joint feedback in SI
            if (!mapAndConvertServoTxData(servoTxArray))
            {
                std::cerr << "[RealHAL] mapAndConvertServoTxData() failed.\n";
                return false;
            }

            return true;
        }

        bool RealHAL::write()
        {
            // 1) Safety check
            if (!rtLayout_)
            {
                std::cerr << "[RealHAL] write() failed: rtLayout_ is null.\n";
                return false;
            }

            // 2) Convert and map from joint commands in SI → servo Rx data
            int servoFrontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
            auto &servoRxArray = rtLayout_->servoBuffer.buffer[servoFrontIdx].rx;

            if (!convertAndMapJointCommandsToServoRxData(servoRxArray))
            {
                std::cerr << "[RealHAL] convertAndMapJointCommandsToServoRxData() failed.\n";
                return false;
            }

            return true;
        }

        //------------------------------------------------------------------------------
        // Private Helpers (Read path)
        //------------------------------------------------------------------------------

        bool RealHAL::mapAndConvertServoTxData(
            const std::array<hand_control::merai::ServoTxPdo,
                             hand_control::merai::MAX_SERVO_DRIVES> &servoTxArray)
        {
            // Single pass: copy raw -> local arrays, then convert to SI

            // std::cout<<"********************* New Data ************************"<<std::endl;
            for (int i = 0; i < driveCount_; ++i)
            {
                // 1) Control feedback
                localJointControlFeedback_[i].statusWord = servoTxArray[i].ctrl.statusWord;

                // 2) Extract raw motion data
                double rawPos = static_cast<double>(servoTxArray[i].motion.positionActual);
                double rawVel = static_cast<double>(servoTxArray[i].motion.velocityActual);
                double rawCurrent = static_cast<double>(servoTxArray[i].motion.currentActual);

                // 3) Convert to SI
                localJointMotionFeedback_[i].positionActual =
                    convertPositionRawToRad(rawPos, localJointConfigs_[i]);
                localJointMotionFeedback_[i].velocityActual =
                    convertVelocityRawToRad(rawVel, localJointConfigs_[i]);
                // localJointMotionFeedback_[i].torqueActual =
                //     convertTorqueRawToNm(rawTorque, localJointConfigs_[i]);

                localJointMotionFeedback_[i].torqueActual =
                    convertCurrentRawToNm(rawCurrent, localJointConfigs_[i]);

                // 4) I/O feedback
                if (i == driveCount_ - 1)
                {

                    uint32_t digitalInputs = servoTxArray[i].io.digitalInputs;
                    localJointFeedbackIO_[i].digitalInputClutch = (digitalInputs & 0x01) != 0; // example bit
                    localJointFeedbackIO_[i].digitalInputThumb = (digitalInputs & 0x02) != 0;  // example bit

                    localJointFeedbackIO_[i].analogInputPinch =
                        static_cast<double>(servoTxArray[i].io.analogInput);
                }

                std::cout<<"************** Read joint cnt : "<<i<<std::endl;
                // std::cout<<"Statusword: "<<servoTxArray[i].ctrl.statusWord<<std::endl;
                std::cout<<"raw position : "<<rawPos<<", Modified Raw Pos : "<<(rawPos - localJointConfigs_[i].position_offset)<<", actual_pos : "<<localJointMotionFeedback_[i].positionActual<<std::endl;
                // std::cout<<"raw velocity : "<<rawVel<<", actual_vel : "<<localJointMotionFeedback_[i].velocityActual<<std::endl;
                // std::cout<<"raw Current : "<<rawCurrent<<", actual_Torque : "<<localJointMotionFeedback_[i].torqueActual<<std::endl;
                
                // if (i == driveCount_ - 1){
                //     std::cout<<"digital input drive : "<<servoTxArray[i].io.digitalInputs<<"Clutch_inputs : "<<localJointFeedbackIO_[i].digitalInputClutch<<std::endl;
                //     std::cout<<"digital input drive : "<<servoTxArray[i].io.digitalInputs<<"Thumb_inputs : "<<localJointFeedbackIO_[i].digitalInputThumb<<std::endl;
                //     std::cout<<"analog input drive : "<<servoTxArray[i].io.analogInput<<"pinch_inputs : "<<localJointFeedbackIO_[i].analogInputPinch<<std::endl;
                // }

            }
            return true;
        }

        //------------------------------------------------------------------------------
        // Private Helpers (Write path)
        //------------------------------------------------------------------------------

        bool RealHAL::convertAndMapJointCommandsToServoRxData(
            std::array<hand_control::merai::ServoRxPdo,
                       hand_control::merai::MAX_SERVO_DRIVES> &servoRxArray)
        {
            // Single pass: read local SI commands, convert to raw, write into servoRxArray
            for (int i = 0; i < driveCount_; ++i)
            {
                // 1) Retrieve local joint commands
                const auto &ctrlCmd = localJointControlCommand_[i];
                const auto &motionCmd = localJointMotionCommand_[i];
                const auto &cfg = localJointConfigs_[i];

                // 2) Convert to raw servo units
                int32_t rawPos = convertPositionRadToRaw(motionCmd.targetPosition, cfg);
                // int16_t rawTorque = convertTorqueNmToRaw(motionCmd.targetTorque, cfg);
                float rawCurrent = convertCurrentNmToRaw(motionCmd.targetTorque, cfg);

                // 3) Fill servoRx data

                servoRxArray[i].ctrl.controlWord = ctrlCmd.controlWord;
                servoRxArray[i].motion.modeOfOperation = 8;

                servoRxArray[i].motion.targetCurrent = rawCurrent;
                servoRxArray[i].motion.targetPosition = rawPos;

                // if (ctrlCmd.controlWord == 15){

                // std::cout<<"************** Write joint cnt : "<<i<<std::endl;
                // std::cout<<"Controlword: "<<ctrlCmd.controlWord<<std::endl;
                // std::cout<<"modeofOperation: "<<int(motionCmd.modeOfOperation)<<std::endl;
                // std::cout<<", target_pos : "<<motionCmd.targetPosition<<", raw_pos : "<< rawPos<<std::endl;

                // }

                std::cout<<"************** Write joint cnt : "<<i<<std::endl;
                // std::cout<<"Controlword: "<<ctrlCmd.controlWord<<std::endl;
                // std::cout<<"modeofOperation: "<<int(motionCmd.modeOfOperation)<<std::endl;
                std::cout<<", target_pos : "<<motionCmd.targetPosition<<", raw_pos : "<< rawPos<<std::endl;

                

                // 4) If you have joint-level digital/analog outputs, you can map them here
                //    e.g. servoRxArray[i].io.digitalOutputs = ...
            }
            return true;
        }

    } // namespace control
} // namespace hand_control
