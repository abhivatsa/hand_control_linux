#pragma once

#include <cstdint>
#include <string>
#include <ecrt.h>  // ec_slave_config_t, ec_domain_t

#include "fieldbus/drives/BaseDrive.h"
#include "merai/ParameterServer.h"     // defines hand_control::merai::DriveConfig
#include "merai/RTMemoryLayout.h"      // defines hand_control::merai::RTMemoryLayout
#include "merai/SharedLogger.h"        // defines hand_control::merai::multi_ring_logger_memory

namespace hand_control
{
    namespace fieldbus
    {
        class IoDrive : public BaseDrive
        {
        public:
            IoDrive(const hand_control::merai::DriveConfig& driveCfg,
                    ec_slave_config_t* sc,
                    ec_domain_t* domain,
                    hand_control::merai::RTMemoryLayout* rtLayout,
                    int driveIndex,
                    hand_control::merai::multi_ring_logger_memory* loggerMem);

            virtual ~IoDrive() = default;

            void initialize() override;
            bool configurePdos() override;
            bool readInputs(uint8_t* domainPd) override;
            bool writeOutputs(uint8_t* domainPd) override;
            void handleState(const DriveUserSignals& signals) override;

            bool registerPdoEntries();

            void setDigitalOutput(uint16_t output);
            uint16_t getDigitalInput() const;

        private:
            ec_domain_t* domain_ = nullptr;

            // Fully qualified merai types
            hand_control::merai::DriveConfig driveCfg_;
            hand_control::merai::RTMemoryLayout* rtLayout_ = nullptr;
            int driveIndex_ = -1;

            hand_control::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            uint16_t digitalInputs_  = 0;
            uint16_t digitalOutputs_ = 0;

            struct IoPdoOffsets
            {
                unsigned int inputOffset  = 0;
                unsigned int outputOffset = 0;
            } ioOffsets_;

            void* getOffsetPointerByIndex(uint16_t index, uint8_t subIndex);
            uint16_t hexStringToUint(const std::string& hexStr);
        };
    } // namespace fieldbus
} // namespace hand_control
