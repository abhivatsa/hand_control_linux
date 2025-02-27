#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <cmath>

// merai includes
#include "merai/RTMemoryLayout.h"      // hand_control::merai::RTMemoryLayout, etc.
#include "merai/ParameterServer.h"     // hand_control::merai::ParameterServer
#include "merai/SharedLogger.h"        // hand_control::merai::multi_ring_logger_memory

// control-layer includes
#include "control/hardware_abstraction/IHardwareAbstractionLayer.h"  // hand_control::control::IHardwareAbstractionLayer
#include "control/hardware_abstraction/DriveData.h"                  // hand_control::control::DriveInput, DriveOutput

namespace hand_control
{
    namespace control
    {
        /**
         * @brief RealHardwareAbstractionLayer: Connects to EtherCAT (rx/tx),
         *        reads joint config from the new ParameterServer (with 'joints' array),
         *        and exposes position/velocity/torque in SI units.
         *
         * This updated version removes references to separate I/O modules or fields that
         * no longer exist in ParameterServer (e.g., ioModuleCount).
         */
        class RealHardwareAbstractionLayer : public IHardwareAbstractionLayer
        {
        public:
            RealHardwareAbstractionLayer(
                hand_control::merai::RTMemoryLayout* rtLayout,
                const hand_control::merai::ParameterServer* paramServerPtr,
                hand_control::merai::multi_ring_logger_memory* loggerMem
            );

            ~RealHardwareAbstractionLayer() override = default;

            bool init() override;
            bool read() override;
            bool write() override;

            // Overridden from base interface (joint data)
            hand_control::merai::JointState* getJointStatesPtr() override
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

            // Overridden from base interface (Drive data)
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
            // Shared memory, config, logger
            hand_control::merai::RTMemoryLayout*           rtLayout_       = nullptr;
            const hand_control::merai::ParameterServer*    paramServerPtr_ = nullptr;
            hand_control::merai::multi_ring_logger_memory* loggerMem_      = nullptr;

            // Number of drives/joints (we unify them here for servo)
            int driveCount_ = 0;

            // Drive-level data (raw)
            std::array<hand_control::control::DriveInput,
                       hand_control::merai::MAX_DRIVES> localDriveInputs_{};
            std::array<hand_control::control::DriveOutput,
                       hand_control::merai::MAX_DRIVES> localDriveOutputs_{};

            // Joint data in SI
            std::array<hand_control::merai::JointState,
                       hand_control::merai::MAX_DRIVES> localJointStates_{};
            std::array<hand_control::merai::JointCommand,
                       hand_control::merai::MAX_DRIVES> localJointCommands_{};

            // We store the relevant joint configs here (gear_ratio, axis_direction, etc.)
            std::array<hand_control::merai::JointConfig,
                       hand_control::merai::MAX_DRIVES> localParams_{};

        private:
            // Private helpers for reading/writing EtherCAT and converting data
            bool mapServoTxToDriveInputs(
                const std::array<hand_control::merai::ServoTxPdo,
                                 hand_control::merai::MAX_SERVO_DRIVES>& servoTxArray
            );

            bool convertDriveInputsToJointStates();

            bool convertJointCommandsToDriveOutputs();

            bool mapDriveOutputsToServoRx(
                std::array<hand_control::merai::ServoRxPdo,
                           hand_control::merai::MAX_SERVO_DRIVES>& servoRxArray
            );
        };
    } // namespace control
} // namespace hand_control
