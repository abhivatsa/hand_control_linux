#pragma once

#include <cstdint>
#include <string>
#include <ecrt.h>  // for ec_slave_config_t, ec_domain_t

#include "fieldbus/drives/BaseDrive.h"
#include "merai/ParameterServer.h"   // defines seven_axis_robot::merai::DriveConfig
#include "merai/RTMemoryLayout.h"    // defines seven_axis_robot::merai::RTMemoryLayout, ServoRxPdo, ServoTxPdo
#include "merai/SharedLogger.h"      // defines seven_axis_robot::merai::multi_ring_logger_memory

namespace seven_axis_robot
{
    namespace fieldbus
    {
        /**
         * @brief ServoDrive
         * Handles reading/writing PDOs for a single servo drive using the new RTMemoryLayout.
         */
        class ServoDrive : public BaseDrive
        {
        public:
            ServoDrive(const seven_axis_robot::merai::DriveConfig& driveCfg,
                       ec_slave_config_t* sc,
                       ec_domain_t* domain,
                       seven_axis_robot::merai::RTMemoryLayout* rtLayout,
                       int driveIndex,
                       seven_axis_robot::merai::multi_ring_logger_memory* loggerMem);

            ~ServoDrive() override = default;

            void initialize() override;
            bool configurePdos() override;

            /**
             * @brief readInputs
             *  - Reads EtherCAT domain memory for input (statusWord, positionActual, etc.)
             *  - Copies into servoTx_ struct
             *  - Then updates the double-buffer at rtLayout_->servoBuffer
             */
            bool readInputs(uint8_t* domainPd) override;

            /**
             * @brief writeOutputs
             *  - Reads the double-buffer from rtLayout_->servoBuffer (Rx portion)
             *  - Copies into servoRx_ struct
             *  - Writes EtherCAT domain memory (controlWord, targetTorque, etc.)
             */
            bool writeOutputs(uint8_t* domainPd) override;

            /**
             * @brief handleState
             *  - Further logic if needed (e.g. drive state transitions).
             */
            void handleState(const DriveUserSignals& signals) override;
            void setCycleContext(const CycleShmContext& ctx) override { cycleCtx_ = ctx; }
            
        private:
            ec_domain_t* domain_ = nullptr;

            seven_axis_robot::merai::DriveConfig driveCfg_;
            seven_axis_robot::merai::RTMemoryLayout* rtLayout_ = nullptr;
            int driveIndex_ = -1;

            // Logging pointer
            seven_axis_robot::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            // Data structures for servo I/O
            // Using the updated sub-struct style
            seven_axis_robot::merai::ServoTxPdo servoTx_; // read from domain -> store in .tx
            seven_axis_robot::merai::ServoRxPdo servoRx_; // read from .rx -> write to domain
            CycleShmContext cycleCtx_; // set per cycle by master
            bool rxFresh_ = true;

            struct ServoOffsets
            {
                // Offsets for inputs (Tx)
                unsigned int statusword                = 0; // 0x6041
                unsigned int position_actual_value     = 0; // 0x6064
                unsigned int velocity_actual_value     = 0; // 0x606C
                unsigned int torque_actual_value       = 0; // 0x6077
                unsigned int digital_input_value       = 0; // 0x60FD
                unsigned int analog_input_value        = 0; // 0x2401
                unsigned int error_code                = 0; // 0x603F

                // Offsets for outputs (Rx)
                unsigned int controlword               = 0; // 0x6040
                unsigned int modes_of_operation        = 0; // 0x6060
                unsigned int target_position           = 0; // 0x607A
                unsigned int target_torque             = 0; // 0x6071
                unsigned int max_torque                = 0; // 0x6072
                unsigned int digital_output_value      = 0; // 0x60FE:01
            } servoOffsets_;

            void* getOffsetPointerByIndex(uint16_t objectIndex, uint8_t subIndex);

            bool registerPdoEntries();
        };

    } // namespace fieldbus
} // namespace seven_axis_robot
