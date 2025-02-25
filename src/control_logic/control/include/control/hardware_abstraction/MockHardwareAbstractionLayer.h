#pragma once

#include <array>

#include "merai/RTMemoryLayout.h"      // motion_control::merai::RTMemoryLayout, JointState, etc.
#include "merai/ParameterServer.h"     // motion_control::merai::ParameterServer
#include "merai/SharedLogger.h"        // motion_control::merai::multi_ring_logger_memory

#include "control/hardware_abstraction/IHardwareAbstractionLayer.h"  // motion_control::control::IHardwareAbstractionLayer
#include "control/hardware_abstraction/DriveData.h"                  // motion_control::control::DriveInput, DriveOutput

namespace motion_control
{
    namespace control
    {
        class MockHardwareAbstractionLayer : public motion_control::control::IHardwareAbstractionLayer
        {
        public:
            MockHardwareAbstractionLayer(
                motion_control::merai::RTMemoryLayout* rtLayout,
                const motion_control::merai::ParameterServer* paramServerPtr,
                motion_control::merai::multi_ring_logger_memory* loggerMem
            );

            ~MockHardwareAbstractionLayer() override = default;

            bool init() override;
            bool read() override;
            bool write() override;

            // Joint Data
            motion_control::merai::JointState*   getJointStatesPtr()   override
            {
                return localJointStates_.data();
            }

            motion_control::merai::JointCommand* getJointCommandsPtr() override
            {
                return localJointCommands_.data();
            }

            size_t getJointCount() const override
            {
                return driveCount_;
            }

            // I/O
            motion_control::merai::IoState* getIoStatesPtr() override
            {
                return localIoStates_.data();
            }

            motion_control::merai::IoCommand* getIoCommandsPtr() override
            {
                return localIoCommands_.data();
            }

            size_t getIoCount() const override
            {
                return ioCount_;
            }

            // Drive Data
            motion_control::control::DriveInput* getDriveInputsPtr() override
            {
                return localDriveInputs_.data();
            }

            motion_control::control::DriveOutput* getDriveOutputsPtr() override
            {
                return localDriveOutputs_.data();
            }

            size_t getDriveCount() const override
            {
                return driveCount_;
            }

        private:
            motion_control::merai::RTMemoryLayout*            rtLayout_       = nullptr;
            const motion_control::merai::ParameterServer*     paramServerPtr_ = nullptr;
            motion_control::merai::multi_ring_logger_memory*  loggerMem_      = nullptr;

            int driveCount_ = 0;
            int ioCount_    = 0;

            // Raw drive data for DSM + simulation
            std::array<motion_control::control::DriveInput,  motion_control::merai::MAX_SERVO_DRIVES> localDriveInputs_{};
            std::array<motion_control::control::DriveOutput, motion_control::merai::MAX_SERVO_DRIVES> localDriveOutputs_{};

            // Joint data in SI
            std::array<motion_control::merai::JointState,   motion_control::merai::MAX_SERVO_DRIVES> localJointStates_{};
            std::array<motion_control::merai::JointCommand, motion_control::merai::MAX_SERVO_DRIVES> localJointCommands_{};

            // I/O data
            std::array<motion_control::merai::IoState,   motion_control::merai::MAX_IO_DRIVES> localIoStates_{};
            std::array<motion_control::merai::IoCommand, motion_control::merai::MAX_IO_DRIVES> localIoCommands_{};

        private:
            // Optionally, a helper to simulate transitions
            void simulateDriveStateTransitions();
        };
    } // namespace control
} // namespace motion_control
