#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <cmath>

// merai includes
#include "merai/RTMemoryLayout.h"      // motion_control::merai::RTMemoryLayout, etc.
#include "merai/ParameterServer.h"     // motion_control::merai::ParameterServer
#include "merai/SharedLogger.h"        // motion_control::merai::multi_ring_logger_memory

// control-layer includes
#include "control/hardware_abstraction/IHardwareAbstractionLayer.h"  // motion_control::control::IHardwareAbstractionLayer
#include "control/hardware_abstraction/DriveData.h"                  // motion_control::control::DriveInput, DriveOutput

namespace motion_control
{
    namespace control
    {
        class RealHardwareAbstractionLayer : public IHardwareAbstractionLayer
        {
        public:
            RealHardwareAbstractionLayer(
                motion_control::merai::RTMemoryLayout* rtLayout,
                const motion_control::merai::ParameterServer* paramServerPtr,
                motion_control::merai::multi_ring_logger_memory* loggerMem
            );

            ~RealHardwareAbstractionLayer() override = default;

            bool init() override;
            bool read() override;
            bool write() override;

            // Overridden from base interface (joint data)
            motion_control::merai::JointState* getJointStatesPtr() override
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

            // Overridden from base interface (I/O)
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

            // Overridden from base interface (Drive data)
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
            // Shared memory, config, logger
            motion_control::merai::RTMemoryLayout*           rtLayout_       = nullptr;
            const motion_control::merai::ParameterServer*    paramServerPtr_ = nullptr;
            motion_control::merai::multi_ring_logger_memory* loggerMem_      = nullptr;

            // Number of drives/joints, I/O modules
            int driveCount_ = 0;
            int ioCount_    = 0;

            // Drive-level data (raw)
            std::array<motion_control::control::DriveInput,
                       motion_control::merai::MAX_SERVO_DRIVES> localDriveInputs_{};
            std::array<motion_control::control::DriveOutput,
                       motion_control::merai::MAX_SERVO_DRIVES> localDriveOutputs_{};

            // Joint data in SI
            std::array<motion_control::merai::JointState,
                       motion_control::merai::MAX_SERVO_DRIVES> localJointStates_{};
            std::array<motion_control::merai::JointCommand,
                       motion_control::merai::MAX_SERVO_DRIVES> localJointCommands_{};

            std::array<motion_control::merai::JointParameters,
                       motion_control::merai::MAX_SERVO_DRIVES> localParams_{};

            // I/O data
            std::array<motion_control::merai::IoState,
                       motion_control::merai::MAX_IO_DRIVES> localIoStates_{};
            std::array<motion_control::merai::IoCommand,
                       motion_control::merai::MAX_IO_DRIVES> localIoCommands_{};

        private:
            // Private helpers for reading/writing EtherCAT and converting data
            bool mapServoTxToDriveInputs(
                const std::array<motion_control::merai::ServoTxPdo,
                                 motion_control::merai::MAX_SERVO_DRIVES>& servoTxArray
            );

            bool convertDriveInputsToJointStates();

            bool convertJointCommandsToDriveOutputs();

            bool mapDriveOutputsToServoRx(
                std::array<motion_control::merai::ServoRxPdo,
                           motion_control::merai::MAX_SERVO_DRIVES>& servoRxArray
            );

            bool mapIoTxToIoState(
                const std::array<motion_control::merai::IoTxPdo,
                                 motion_control::merai::MAX_IO_DRIVES>& ioTxArray
            );

            bool mapIoCommandToIoRx(
                std::array<motion_control::merai::IoRxPdo,
                           motion_control::merai::MAX_IO_DRIVES>& ioRxArray
            );
        };
    } // namespace control
} // namespace motion_control
