#include <iostream>
#include <stdexcept>
#include <cmath>  // for M_PI or any math

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

            // 1) Determine how many drives from paramServerPtr_
            driveCount_ = paramServerPtr_->driveCount;
            if (driveCount_ > hand_control::merai::MAX_DRIVES)
            {
                driveCount_ = hand_control::merai::MAX_DRIVES;
            }

            // 2) Load static servo/joint config
            //    Adjust if your paramServer uses a different structure or naming.
            for (int i = 0; i < driveCount_; i++)
            {
                localJointConfigs_[i] = paramServerPtr_->joints[i];
            }

            // 3) Zero out local arrays
            for (int i = 0; i < driveCount_; ++i)
            {
                // Clear DriveInputControl_ / DriveOutputControl_ if needed
                DriveInputControl_[i] = {};  
                DriveOutputControl_[i] = {};

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

            // 1) Acquire the latest servoTx array from shared memory
            int servoFrontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
            auto& servoTxArray = rtLayout_->servoBuffer.buffer[servoFrontIdx].tx;

            // 2) Map servoTxArray -> DriveInputControl_
            // if (!mapServoTxData(servoTxArray))
            // {
            //     std::cerr << "[RealHAL] mapServoTxData failed.\n";
            //     return false;
            // }

            // 3) Convert DriveInputControl_ -> localJointStates_
            if (!convertTxToJointStates())
            {
                std::cerr << "[RealHAL] convertTxToJointStates failed.\n";
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

            // 1) Convert localJointCommands_ -> DriveOutputControl_
            if (!convertJointCommandsToRx())
            {
                std::cerr << "[RealHAL] convertJointCommandsToRx failed.\n";
                return false;
            }

            // 2) Map DriveOutputControl_ -> servoRx array
            int servoFrontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
            auto& servoRxArray = rtLayout_->servoBuffer.buffer[servoFrontIdx].rx;

            // if (!mapServoRxData(servoRxArray))
            // {
            //     std::cerr << "[RealHAL] mapServoRxData failed.\n";
            //     return false;
            // }

            return true;
        }

        // --------------------------------------------------------------------
        // Private Helpers
        // --------------------------------------------------------------------

        bool RealHAL::mapServoTxData(
            const std::array<hand_control::merai::ServoTxPdo,
                             hand_control::merai::MAX_DRIVES>& servoTxArray)
        {
            // Copy raw data from servoTxArray into DriveInputControl_
            for (int i = 0; i < driveCount_; ++i)
            {
                // DriveInputControl_[i].statusWord  = servoTxArray[i].statusWord;
                // DriveInputControl_[i].positionRaw = servoTxArray[i].positionActual;
                // DriveInputControl_[i].velocityRaw = servoTxArray[i].velocityActual;
                // DriveInputControl_[i].torqueRaw   = servoTxArray[i].torqueActual;

                // debug
                // std::cout << "[RealHAL] Drive " << i
                //           << " posRaw=" << servoTxArray[i].positionActual << "\n";
            }
            return true;
        }

        bool RealHAL::convertTxToJointStates()
        {
            // Convert from raw drive inputs -> SI-based joint states
            for (int i = 0; i < driveCount_; ++i)
            {
                const auto& driveIn = DriveInputControl_[i];
                const auto& cfg     = localJointConfigs_[i];
                auto&       jState  = localJointStates_[i];

                // Example usage:
                double gearRatio      = cfg.gear_ratio;
                int    axisDirection  = cfg.axis_direction; // ±1
                double posOffset      = cfg.position_offset;
                int    torqueDir      = cfg.torque_axis_direction; // ±1

                // // Convert raw counts to radians
                // // e.g. 1 raw => 0.001 rad
                // double posRad = static_cast<double>(driveIn.positionRaw) * 0.001;
                // posRad *= gearRatio * axisDirection;
                // posRad += posOffset;

                // double velRad = static_cast<double>(driveIn.velocityRaw) * 0.0001;
                // velRad *= (gearRatio * axisDirection);

                // double torqueNm = static_cast<double>(driveIn.torqueRaw) * 0.01;
                // torqueNm *= (gearRatio * torqueDir);

                // jState.position = posRad;
                // jState.velocity = velRad;
                // jState.torque   = torqueNm;
            }
            return true;
        }

        bool RealHAL::convertJointCommandsToRx()
        {
            // Convert from localJointCommands_ (SI) -> DriveOutputControl_ (raw)
            for (int i = 0; i < driveCount_; ++i)
            {
                const auto& cfg = localJointConfigs_[i];
                const auto& cmd = localJointCommands_[i];
                auto&       out = DriveOutputControl_[i];

                double gearRatio      = cfg.gear_ratio;
                int    axisDirection  = cfg.axis_direction;
                double posOffset      = cfg.position_offset;
                int    torqueDir      = cfg.torque_axis_direction;

                // position (rad -> raw)
                double posRad = cmd.position - posOffset;
                posRad /= (gearRatio * axisDirection);
                int rawPos = static_cast<int>(posRad * 1000.0); // inverse of 1 raw => 0.001 rad

                // velocity (rad/s -> raw)
                double velRad = cmd.velocity / (gearRatio * axisDirection);
                int rawVel = static_cast<int>(velRad * 10000.0);

                // torque (Nm -> raw)
                double tNm = cmd.torque / (gearRatio * torqueDir);
                int rawTorque = static_cast<int>(tNm * 100.0); // inverse of 1 raw => 0.01 Nm

                // Fill out typical fields
                out.controlWord       = 0x000F; // example
                out.modeOfOperation   = 9;      // e.g. torque mode
                // out.targetPositionRaw = rawPos;
                // out.targetVelocityRaw = rawVel;
                // out.targetTorqueRaw   = rawTorque;
            }
            return true;
        }

        bool RealHAL::mapServoRxData(
            std::array<hand_control::merai::ServoRxPdo,
                       hand_control::merai::MAX_DRIVES>& servoRxArray)
        {
            // Copy from DriveOutputControl_ -> servoRxArray
            for (int i = 0; i < driveCount_; ++i)
            {
                const auto& out = DriveOutputControl_[i];
                // servoRxArray[i].controlWord     = out.controlWord;
                // servoRxArray[i].modeOfOperation = out.modeOfOperation;
                // servoRxArray[i].targetPosition  = out.targetPositionRaw;
                // servoRxArray[i].targetVelocity  = out.targetVelocityRaw;
                // servoRxArray[i].targetTorque    = out.targetTorqueRaw;
            }
            return true;
        }

    } // namespace control
} // namespace hand_control
