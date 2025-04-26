#pragma once

#include <cstdint>
#include <string>
#include <ecrt.h>  // for ec_slave_config_t, ec_domain_t

#include "fieldbus/drives/BaseDrive.h"
#include "merai/ParameterServer.h"   // defines hand_control::merai::DriveConfig
#include "merai/RTMemoryLayout.h"    // defines hand_control::merai::RTMemoryLayout, ServoRxPdo, ServoTxPdo
#include "merai/SharedLogger.h"      // defines hand_control::merai::multi_ring_logger_memory

namespace hand_control
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
            ServoDrive(const hand_control::merai::DriveConfig& driveCfg,
                       ec_slave_config_t* sc,
                       ec_domain_t* domain,
                       hand_control::merai::RTMemoryLayout* rtLayout,
                       int driveIndex,
                       hand_control::merai::multi_ring_logger_memory* loggerMem);

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
            
        private:
            ec_domain_t* domain_ = nullptr;

            hand_control::merai::DriveConfig driveCfg_;
            hand_control::merai::RTMemoryLayout* rtLayout_ = nullptr;
            int driveIndex_ = -1;

            // Logging pointer
            hand_control::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            // Data structures for servo I/O
            // Using the updated sub-struct style
            hand_control::merai::ServoTxPdo servoTx_; // read from domain -> store in .tx
            hand_control::merai::ServoRxPdo servoRx_; // read from .rx -> write to domain

            struct ServoOffsets
            {
                // Offsets for inputs (Tx)
                unsigned int statusword                = 0; // 0x6041
                unsigned int position_actual_value     = 0; // 0x6064
                unsigned int velocity_actual_value     = 0; // 0x606C
                unsigned int current_actual_value      = 0; // 0x2076
                unsigned int digital_input_value       = 0; // 0x2600
                unsigned int analog_input_value        = 0; // 0x2081
                unsigned int error_code                = 0; // 0x603F

                
                // Possibly mode_of_operation_display

                // Offsets for outputs (Rx)
                unsigned int controlword               = 0; // 0x6040
                unsigned int modes_of_operation        = 0; // 0x6060
                unsigned int target_position           = 0; // 0x607A
                unsigned int target_current            = 0; // 0x201A
                unsigned int max_current               = 0; // 0x6073
            } servoOffsets_;

            void* getOffsetPointerByIndex(uint16_t objectIndex);
            uint16_t hexStringToUint(const std::string& hexStr);

            bool registerPdoEntries();
        };

    } // namespace fieldbus
} // namespace hand_control
