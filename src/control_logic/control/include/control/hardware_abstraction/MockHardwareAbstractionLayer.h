#pragma once

#include <array>

#include "merai/RTMemoryLayout.h"      // hand_control::merai::RTMemoryLayout, JointState, etc.
#include "merai/ParameterServer.h"     // hand_control::merai::ParameterServer
#include "merai/SharedLogger.h"        // hand_control::merai::multi_ring_logger_memory

#include "control/hardware_abstraction/IHardwareAbstractionLayer.h"  // hand_control::control::IHardwareAbstractionLayer
#include "control/hardware_abstraction/DriveData.h"                  // hand_control::control::DriveInput, DriveOutput

namespace hand_control
{
    namespace control
    {
        class MockHardwareAbstractionLayer : public hand_control::control::IHardwareAbstractionLayer
        {
        public:
            MockHardwareAbstractionLayer(
                hand_control::merai::RTMemoryLayout* rtLayout,
                const hand_control::merai::ParameterServer* paramServerPtr,
                hand_control::merai::multi_ring_logger_memory* loggerMem
            );

            ~MockHardwareAbstractionLayer() override = default;

            bool init() override;
            bool read() override;
            bool write() override;

            // Joint Data
            hand_control::merai::JointState*   getJointStatesPtr()   override
            {
                return localJointStates_.data();
            }

            hand_control::merai::JointCommand* getJointCommandsPtr() override
            {
                return localJointCommands_.data();
            }

            size_t getJointCount() const override
            {
                return driveCount_;
            }

            // I/O
            hand_control::merai::IoState* getIoStatesPtr() override
            {
                return localIoStates_.data();
            }

            hand_control::merai::IoCommand* getIoCommandsPtr() override
            {
                return localIoCommands_.data();
            }

            size_t getIoCount() const override
            {
                return ioCount_;
            }

            // Drive Data
            hand_control::control::DriveInput* getDriveInputsPtr() override
            {
                return localDriveInputs_.data();
            }

            hand_control::control::DriveOutput* getDriveOutputsPtr() override
            {
                return localDriveOutputs_.data();
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
            int ioCount_    = 0;

            // Raw drive data for DSM + simulation
            std::array<hand_control::control::DriveInput,  hand_control::merai::MAX_SERVO_DRIVES> localDriveInputs_{};
            std::array<hand_control::control::DriveOutput, hand_control::merai::MAX_SERVO_DRIVES> localDriveOutputs_{};

            // Joint data in SI
            std::array<hand_control::merai::JointState,   hand_control::merai::MAX_SERVO_DRIVES> localJointStates_{};
            std::array<hand_control::merai::JointCommand, hand_control::merai::MAX_SERVO_DRIVES> localJointCommands_{};

            // I/O data
            std::array<hand_control::merai::IoState,   hand_control::merai::MAX_IO_DRIVES> localIoStates_{};
            std::array<hand_control::merai::IoCommand, hand_control::merai::MAX_IO_DRIVES> localIoCommands_{};

        private:
            // Optionally, a helper to simulate transitions
            void simulateDriveStateTransitions();
        };
    } // namespace control
} // namespace hand_control
