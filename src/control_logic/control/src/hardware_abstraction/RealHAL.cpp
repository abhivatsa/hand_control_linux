#include <iostream>
#include <stdexcept>
#include <cmath>  // for M_PI

#include "control/hardware_abstraction/RealHAL.h"

namespace hand_control
{
    namespace control
    {

        RealHAL::RealHAL(
            hand_control::merai::RTMemoryLayout*           rtLayout,
            const hand_control::merai::ParameterServer*    paramServerPtr,
            hand_control::merai::multi_ring_logger_memory* loggerMem
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

        bool RealHAL::init()
        {
            std::cout << "[RealHAL] init() called.\n";

            // 1) Determine servo/driveCount from ParameterServer's jointCount
            driveCount_ = paramServerPtr_->driveCount;  // new ParameterServer field
            if (driveCount_ > hand_control::merai::MAX_DRIVES)
            {
                driveCount_ = hand_control::merai::MAX_DRIVES;
            }

            // 2) Load static servo parameters from paramServerPtr_->joints[i]
            //    Each element has gear_ratio, axis_direction, etc.
            for (int i = 0; i < driveCount_; i++)
            {
                localParams_[i] = paramServerPtr_->joints[i];
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

            return true;
        }

        bool RealHAL::read()
        {
            if (!rtLayout_)
            {
                std::cerr << "[RealHAL] read() failed: rtLayout_ is null.\n";
                return false;
            }

            // 1) Read servo data from EtherCAT => localDriveInputs_ => localJointStates_
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

            return true;
        }

        bool RealHAL::write()
        {
            if (!rtLayout_)
            {
                std::cerr << "[RealHAL] write() failed: rtLayout_ is null.\n";
                return false;
            }

            // 1) localJointCommands_ => localDriveOutputs_ => EtherCAT
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

            return true;
        }

        bool RealHAL::mapServoTxToDriveInputs(
            const std::array<hand_control::merai::ServoTxPdo,
                             hand_control::merai::MAX_DRIVES>& servoTxArray)
        {
            for (int i = 0; i < driveCount_; ++i)
            {
                localDriveInputs_[i].statusWord   = servoTxArray[i].statusWord;
                localDriveInputs_[i].positionRaw  = servoTxArray[i].positionActual;
                localDriveInputs_[i].velocityRaw  = servoTxArray[i].velocityActual;
                localDriveInputs_[i].torqueRaw    = servoTxArray[i].torqueActual;

                std::cout << "pos raw " << i << " : " << servoTxArray[i].positionActual << std::endl;
            }
            return true;
        }

        bool RealHAL::convertDriveInputsToJointStates()
        {
            for (int i = 0; i < driveCount_; ++i)
            {
                auto& input  = localDriveInputs_[i];
                auto& p      = localParams_[i];  // This is a JointConfig from paramServer
                auto& jstate = localJointStates_[i];

                // Example conversions:
                double gearRatio    = p.gear_ratio;        // from JointConfig
                int axisDir         = p.axis_direction;    // ±1
                double offset       = p.position_offset;   // e.g. zero offset in your JSON

                // Example position conversion from raw counts:
                double posRad = static_cast<double>(input.positionRaw) * 0.001;  // Assume 1 count => 0.001 rad
                posRad *= gearRatio * static_cast<double>(axisDir);
                posRad += offset;

                // Example velocity conversion from raw counts:
                double velRad = static_cast<double>(input.velocityRaw) * 0.0001;  // Assume 1 count => 0.0001 rad/s
                velRad *= gearRatio * static_cast<double>(axisDir);

                // Example torque from raw => we assume 1 raw => 0.01 Nm
                double torqueNm = static_cast<double>(input.torqueRaw) * 0.01;
                torqueNm *= gearRatio * static_cast<double>(p.torque_axis_direction);

                jstate.position = posRad;
                jstate.velocity = velRad;
                jstate.torque   = torqueNm;
            }
            return true;
        }

        bool RealHAL::convertJointCommandsToDriveOutputs()
        {
            for (int i = 0; i < driveCount_; ++i)
            {
                auto& p   = localParams_[i]; // JointConfig
                auto& cmd = localJointCommands_[i];
                auto& out = localDriveOutputs_[i];

                double gearRatio  = p.gear_ratio;
                int axisDir       = p.axis_direction;
                double offset     = p.position_offset;

                // Inverse of your read() conversion logic:
                double desiredPos = cmd.position - offset;
                desiredPos /= static_cast<double>(axisDir);
                desiredPos /= gearRatio;
                int rawPos = static_cast<int>(desiredPos * 1000.0);

                double desiredVel = cmd.velocity;
                desiredVel /= (axisDir * gearRatio);
                int rawVel = static_cast<int>(desiredVel * 10000.0);

                double desiredTorque = cmd.torque;
                desiredTorque /= (gearRatio * static_cast<double>(p.torque_axis_direction));
                int rawTorque = static_cast<int>(desiredTorque * 100.0); // if 1 raw => 0.01 Nm

                // Default controlWord / modeOfOperation
                out.controlWord       = 0x000F; // e.g. "Enable Operation"
                out.modeOfOperation   = 9;      // e.g. torque mode
                out.targetPositionRaw = rawPos;
                out.targetVelocityRaw = rawVel;
                out.targetTorqueRaw   = rawTorque;
            }
            return true;
        }

        bool RealHAL::mapDriveOutputsToServoRx(
            std::array<hand_control::merai::ServoRxPdo,
                       hand_control::merai::MAX_DRIVES>& servoRxArray)
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

    } // namespace control
} // namespace hand_control
