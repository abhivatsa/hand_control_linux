#pragma once

#include <array>

#include "merai/RTMemoryLayout.h"      // hand_control::merai::RTMemoryLayout, JointState, etc.
#include "merai/ParameterServer.h"     // hand_control::merai::ParameterServer
#include "merai/SharedLogger.h"        // hand_control::merai::multi_ring_logger_memory

#include "control/hardware_abstraction/BaseHAL.h"  // hand_control::control::BaseHAL

namespace hand_control
{
    namespace control
    {
        class SimHAL : public hand_control::control::BaseHAL
        {
        public:
            SimHAL(
                hand_control::merai::RTMemoryLayout* rtLayout,
                const hand_control::merai::ParameterServer* paramServerPtr,
                hand_control::merai::multi_ring_logger_memory* loggerMem
            );

            ~SimHAL() override = default;

            bool init() override;
            bool read() override;
            bool write() override;

            // Joint Data
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

            // Drive Data
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
            hand_control::merai::RTMemoryLayout*            rtLayout_       = nullptr;
            const hand_control::merai::ParameterServer*     paramServerPtr_ = nullptr;
            hand_control::merai::multi_ring_logger_memory*  loggerMem_      = nullptr;

            int driveCount_ = 0;

            // Raw drive data for DSM + simulation
            std::array<hand_control::merai::ServoTxControl,  hand_control::merai::MAX_SERVO_DRIVES> DriveInputControl_{};
            std::array<hand_control::merai::ServoRxControl, hand_control::merai::MAX_SERVO_DRIVES> DriveOutputControl_{};

            // Joint data in SI
            std::array<hand_control::merai::JointState,   hand_control::merai::MAX_SERVO_DRIVES> localJointStates_;
            std::array<hand_control::merai::JointCommand, hand_control::merai::MAX_SERVO_DRIVES> localJointCommands_;
            std::array<hand_control::merai::JointIO,      hand_control::merai::MAX_SERVO_DRIVES> localJointIOs_;

        private:
            // Optionally, a helper to simulate transitions
            void simulateDriveStateTransitions();
            void simulateJointIOChanges();
        };
    } // namespace control
} // namespace hand_control
