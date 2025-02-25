#include <iostream>
#include <stdexcept>
#include <cmath>  // for M_PI

#include "control/hardware_abstraction/RealHardwareAbstractionLayer.h"

namespace motion_control
{
    namespace control
    {
        // ---------------------------------------------------------------------
        // Constructor
        // ---------------------------------------------------------------------
        RealHardwareAbstractionLayer::RealHardwareAbstractionLayer(
            motion_control::merai::RTMemoryLayout*           rtLayout,
            const motion_control::merai::ParameterServer*    paramServerPtr,
            motion_control::merai::multi_ring_logger_memory* loggerMem
        )
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
        }

        // ---------------------------------------------------------------------
        // init()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::init()
        {
            std::cout << "[RealHAL] init() called.\n";

            // 1) Determine driveCount (servo) from ParameterServer
            driveCount_ = paramServerPtr_->jointCount;  // Example field in ParameterServer
            if (driveCount_ > motion_control::merai::MAX_SERVO_DRIVES)
            {
                driveCount_ = motion_control::merai::MAX_SERVO_DRIVES;
            }

            // 2) Load static servo parameters (JointParameters)
            for (int i = 0; i < driveCount_; i++)
            {
                // Copy directly from your ParameterServer
                localParams_[i] = paramServerPtr_->joints[i].parameters;
            }

            // Zero out localJointStates_ / localJointCommands_
            for (int i = 0; i < driveCount_; ++i)
            {
                localJointStates_[i].position = 0.0;
                localJointStates_[i].velocity = 0.0;
                localJointStates_[i].torque   = 0.0;

                localJointCommands_[i].position = 0.0;
                localJointCommands_[i].velocity = 0.0;
                localJointCommands_[i].torque   = 0.0;
            }

            // 3) Determine ioCount (I/O modules) from ParameterServer
            ioCount_ = paramServerPtr_->ioModuleCount;  // Example field
            if (ioCount_ > motion_control::merai::MAX_IO_DRIVES)
            {
                ioCount_ = motion_control::merai::MAX_IO_DRIVES;
            }

            // Zero out localIoStates_ / localIoCommands_
            for (int i = 0; i < ioCount_; ++i)
            {
                // Digital
                for (int ch = 0; ch < 8; ++ch)
                {
                    localIoStates_[i].digitalInputs[ch]   = false;
                    localIoCommands_[i].digitalOutputs[ch] = false;
                }
                // Analog
                for (int ch = 0; ch < 2; ++ch)
                {
                    localIoStates_[i].analogInputs[ch]    = 0.0f;
                    localIoCommands_[i].analogOutputs[ch] = 0.0f;
                }
            }

            return true;
        }

        // ---------------------------------------------------------------------
        // read()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::read()
        {
            if (!rtLayout_)
            {
                std::cerr << "[RealHAL] read() failed: rtLayout_ is null.\n";
                return false;
            }

            // ===============================
            // 1) Servo: Read from EtherCAT => localDriveInputs_ => localJointStates_
            // ===============================
            int servoFrontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
            auto& servoTxArray = rtLayout_->servoBuffer.buffer[servoFrontIdx].tx;

            // (a) Map raw Tx data into localDriveInputs_
            if (!mapServoTxToDriveInputs(servoTxArray))
            {
                std::cerr << "[RealHAL] mapServoTxToDriveInputs failed.\n";
                return false;
            }

            // (b) Convert localDriveInputs_ into localJointStates_ (SI units)
            if (!convertDriveInputsToJointStates())
            {
                std::cerr << "[RealHAL] convertDriveInputsToJointStates failed.\n";
                return false;
            }

            // ===============================
            // 2) I/O Read
            // ===============================
            int ioFrontIdx = rtLayout_->ioBuffer.frontIndex.load(std::memory_order_acquire);
            auto& ioTxArray = rtLayout_->ioBuffer.buffer[ioFrontIdx].tx;

            if (!mapIoTxToIoState(ioTxArray))
            {
                std::cerr << "[RealHAL] mapIoTxToIoState failed.\n";
                return false;
            }

            return true;
        }

        // ---------------------------------------------------------------------
        // write()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::write()
        {
            if (!rtLayout_)
            {
                std::cerr << "[RealHAL] write() failed: rtLayout_ is null.\n";
                return false;
            }

            // ===============================
            // 1) Servo: localJointCommands_ => localDriveOutputs_ => EtherCAT
            // ===============================
            int servoFrontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
            auto& servoRxArray = rtLayout_->servoBuffer.buffer[servoFrontIdx].rx;

            // (a) Convert localJointCommands_ (SI) => localDriveOutputs_ (raw)
            if (!convertJointCommandsToDriveOutputs())
            {
                std::cerr << "[RealHAL] convertJointCommandsToDriveOutputs failed.\n";
                return false;
            }

            // (b) Map localDriveOutputs_ to servoRxArray
            if (!mapDriveOutputsToServoRx(servoRxArray))
            {
                std::cerr << "[RealHAL] mapDriveOutputsToServoRx failed.\n";
                return false;
            }

            // ===============================
            // 2) I/O Write
            // ===============================
            int ioFrontIdx = rtLayout_->ioBuffer.frontIndex.load(std::memory_order_acquire);
            auto& ioRxArray = rtLayout_->ioBuffer.buffer[ioFrontIdx].rx;

            if (!mapIoCommandToIoRx(ioRxArray))
            {
                std::cerr << "[RealHAL] mapIoCommandToIoRx failed.\n";
                return false;
            }

            return true;
        }

        // ---------------------------------------------------------------------
        // mapServoTxToDriveInputs()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::mapServoTxToDriveInputs(
            const std::array<motion_control::merai::ServoTxPdo,
                             motion_control::merai::MAX_SERVO_DRIVES>& servoTxArray)
        {
            for (int i = 0; i < driveCount_; ++i)
            {
                localDriveInputs_[i].statusWord   = servoTxArray[i].statusWord;
                localDriveInputs_[i].positionRaw  = servoTxArray[i].positionActual;
                localDriveInputs_[i].velocityRaw  = servoTxArray[i].velocityActual;
                localDriveInputs_[i].torqueRaw    = servoTxArray[i].torqueActual;
                // If servoTx has modeOfOperation display, you could read it too
            }
            return true;
        }

        // ---------------------------------------------------------------------
        // convertDriveInputsToJointStates()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::convertDriveInputsToJointStates()
        {
            for (int i = 0; i < driveCount_; ++i)
            {
                auto& input  = localDriveInputs_[i];
                auto& p      = localParams_[i];  // JointParameters
                auto& jstate = localJointStates_[i];

                double cpr = static_cast<double>(p.encoder_resolution.counts_per_revolution);

                // Example position conversion
                double posRad = (static_cast<double>(input.positionRaw) * (2.0 * M_PI))
                                / (cpr * p.gear_ratio);
                posRad *= static_cast<double>(p.axis_direction);
                posRad += p.joint_position_offset;

                double velRad = (static_cast<double>(input.velocityRaw) * (2.0 * M_PI))
                                / (cpr * p.gear_ratio);
                velRad *= static_cast<double>(p.axis_direction);

                double torqueNm = (static_cast<double>(input.torqueRaw) / 1000.0)
                                  * p.motor_rated_torque
                                  * static_cast<double>(p.torque_axis_direction)
                                  * p.gear_ratio;

                jstate.position = posRad;
                jstate.velocity = velRad;
                jstate.torque   = torqueNm;
            }
            return true;
        }

        // ---------------------------------------------------------------------
        // convertJointCommandsToDriveOutputs()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::convertJointCommandsToDriveOutputs()
        {
            for (int i = 0; i < driveCount_; ++i)
            {
                auto& p   = localParams_[i];
                auto& cmd = localJointCommands_[i];
                auto& out = localDriveOutputs_[i];

                double cpr = static_cast<double>(p.encoder_resolution.counts_per_revolution);

                double desiredPos    = cmd.position;
                double desiredVel    = cmd.velocity;
                double desiredTorque = cmd.torque;

                int rawPos = static_cast<int>(
                    (desiredPos * p.gear_ratio * cpr) / (2.0 * M_PI)
                );
                int rawVel = static_cast<int>(
                    (desiredVel * p.gear_ratio * cpr) / (2.0 * M_PI)
                );
                int rawTorque = static_cast<int>(
                    (desiredTorque / p.motor_rated_torque) * 1000.0
                );

                // Default controlWord / modeOfOperation
                out.controlWord       = 0x000F; // e.g. "Enable Operation"
                out.modeOfOperation   = 9;      // e.g. torque mode
                out.targetPositionRaw = rawPos;
                out.targetVelocityRaw = rawVel;
                out.targetTorqueRaw   = rawTorque;
            }
            return true;
        }

        // ---------------------------------------------------------------------
        // mapDriveOutputsToServoRx()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::mapDriveOutputsToServoRx(
            std::array<motion_control::merai::ServoRxPdo,
                       motion_control::merai::MAX_SERVO_DRIVES>& servoRxArray)
        {
            for (int i = 0; i < driveCount_; ++i)
            {
                auto& out = localDriveOutputs_[i];
                servoRxArray[i].controlWord     = out.controlWord;
                servoRxArray[i].modeOfOperation = out.modeOfOperation;
                servoRxArray[i].targetPosition  = out.targetPositionRaw;
                servoRxArray[i].targetVelocity  = out.targetVelocityRaw;
                servoRxArray[i].targetTorque    = out.targetTorqueRaw;
            }
            return true;
        }

        // ---------------------------------------------------------------------
        // mapIoTxToIoState()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::mapIoTxToIoState(
            const std::array<motion_control::merai::IoTxPdo,
                             motion_control::merai::MAX_IO_DRIVES>& ioTxArray)
        {
            for (int i = 0; i < ioCount_; ++i)
            {
                // Digital
                for (int ch = 0; ch < 8; ++ch)
                {
                    localIoStates_[i].digitalInputs[ch] = ioTxArray[i].digitalInputs[ch];
                }
                // Analog
                for (int ch = 0; ch < 2; ++ch)
                {
                    localIoStates_[i].analogInputs[ch] = ioTxArray[i].analogInputs[ch];
                }
            }
            return true;
        }

        // ---------------------------------------------------------------------
        // mapIoCommandToIoRx()
        // ---------------------------------------------------------------------
        bool RealHardwareAbstractionLayer::mapIoCommandToIoRx(
            std::array<motion_control::merai::IoRxPdo,
                       motion_control::merai::MAX_IO_DRIVES>& ioRxArray)
        {
            for (int i = 0; i < ioCount_; ++i)
            {
                // Digital
                for (int ch = 0; ch < 8; ++ch)
                {
                    ioRxArray[i].digitalOutputs[ch] = localIoCommands_[i].digitalOutputs[ch];
                }
                // Analog
                for (int ch = 0; ch < 2; ++ch)
                {
                    ioRxArray[i].analogOutputs[ch] = localIoCommands_[i].analogOutputs[ch];
                }
            }
            return true;
        }
    } // namespace control
} // namespace motion_control
