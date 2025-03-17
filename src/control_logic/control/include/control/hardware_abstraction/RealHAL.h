#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <cmath>

// merai includes
#include "merai/RTMemoryLayout.h"  
#include "merai/ParameterServer.h" 
#include "merai/SharedLogger.h"    

#include "control/hardware_abstraction/BaseHAL.h"

namespace hand_control
{
    namespace control
    {
        class RealHAL : public BaseHAL
        {
        public:
            RealHAL(
                hand_control::merai::RTMemoryLayout *rtLayout,
                const hand_control::merai::ParameterServer *paramServerPtr,
                hand_control::merai::multi_ring_logger_memory *loggerMem);

            ~RealHAL() override = default;

            // --------------------------------------------------------------------
            // Implementing BaseHAL
            // --------------------------------------------------------------------
            bool init() override;
            bool read() override;
            bool write() override;

            // Joint data
            hand_control::merai::JointState* getJointStatesPtr() override
            {
                return localJointStates_.data();
            }

            hand_control::merai::JointCommand* getJointCommandsPtr() override
            {
                return localJointCommands_.data();
            }

            hand_control::merai::JointIO* getJointIOPtr() override
            {
                return localJointIOs_.data();
            }

            size_t getJointCount() const override
            {
                return driveCount_;
            }

            // Drive control data
            hand_control::merai::ServoTxControl* getDriveInputControlPtr() override
            {
                return DriveInputControl_.data();
            }

            hand_control::merai::ServoRxControl* getDriveOutputControlPtr() override
            {
                return DriveOutputControl_.data();
            }

            size_t getDriveCount() const override
            {
                return driveCount_;
            }

        private:
            // Shared memory, config, logger references
            hand_control::merai::RTMemoryLayout* rtLayout_ = nullptr;
            const hand_control::merai::ParameterServer* paramServerPtr_ = nullptr;
            hand_control::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            // Number of drives/joints
            int driveCount_ = 0;

            // Local arrays for servo control bits (feedback and commands)
            std::array<hand_control::merai::ServoTxControl, hand_control::merai::MAX_DRIVES> DriveInputControl_;
            std::array<hand_control::merai::ServoRxControl, hand_control::merai::MAX_DRIVES> DriveOutputControl_;

            // Joint data in SI
            std::array<hand_control::merai::JointState,   hand_control::merai::MAX_DRIVES> localJointStates_;
            std::array<hand_control::merai::JointCommand, hand_control::merai::MAX_DRIVES> localJointCommands_;
            std::array<hand_control::merai::JointIO,      hand_control::merai::MAX_DRIVES> localJointIOs_;

            // **Local copy of each joint's config** (gear ratio, offset, etc.).
            // Adjust the type to match your paramServer->joints[i] type.
            std::array<hand_control::merai::JointConfig, hand_control::merai::MAX_DRIVES> localJointConfigs_;

        private:
            // Private helpers for reading/writing EtherCAT data
            bool mapServoTxData(const std::array<hand_control::merai::ServoTxPdo, hand_control::merai::MAX_DRIVES>& servoTxArray);
            bool convertTxToJointStates();
            bool convertJointCommandsToRx();
            bool mapServoRxData(std::array<hand_control::merai::ServoRxPdo, hand_control::merai::MAX_DRIVES>& servoRxArray);
        };

    } // namespace control
} // namespace hand_control
