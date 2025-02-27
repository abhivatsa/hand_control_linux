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
        class ServoDrive : public BaseDrive
        {
        public:
            ServoDrive(const hand_control::merai::DriveConfig& driveCfg,
                       ec_slave_config_t* sc,
                       ec_domain_t* domain,
                       hand_control::merai::RTMemoryLayout* rtLayout,
                       int driveIndex,
                       hand_control::merai::multi_ring_logger_memory* loggerMem);

            virtual ~ServoDrive() = default;

            void initialize() override;
            bool configurePdos() override;
            bool readInputs(uint8_t* domainPd) override;
            bool writeOutputs(uint8_t* domainPd) override;
            void handleState(const DriveUserSignals& signals) override;
            
        private:
            ec_domain_t* domain_ = nullptr;

            hand_control::merai::DriveConfig driveCfg_;
            hand_control::merai::RTMemoryLayout* rtLayout_ = nullptr;
            int driveIndex_ = -1;

            // Logging pointer
            hand_control::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            // Data structures for servo I/O
            hand_control::merai::ServoTxPdo servoTx_;
            hand_control::merai::ServoRxPdo servoRx_;

            struct ServoOffsets
            {
                unsigned int statusword                 = 0;
                unsigned int mode_of_operation_display  = 0;
                unsigned int position_actual_value      = 0;
                unsigned int velocity_actual_value      = 0;
                unsigned int current_actual_value       = 0;
                unsigned int torque_actual_value        = 0;
                unsigned int error_code                 = 0;

                unsigned int controlword                = 0;
                unsigned int modes_of_operation         = 0;
                unsigned int target_position            = 0;
                unsigned int max_current                = 0;
                unsigned int target_torque              = 0;
            } servoOffsets_;

            void* getOffsetPointerByIndex(uint16_t objectIndex);
            uint16_t hexStringToUint(const std::string& hexStr);
            bool registerPdoEntries();
        };
    } // namespace fieldbus
} // namespace hand_control
