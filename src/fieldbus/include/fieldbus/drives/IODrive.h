#pragma once

#include <cstdint>
#include <string>
#include <ecrt.h>  // ec_slave_config_t, ec_domain_t

#include "fieldbus/drives/BaseDrive.h"
#include "merai/ParameterServer.h"     // defines seven_axis_robot::merai::DriveConfig
#include "merai/RTMemoryLayout.h"      // defines seven_axis_robot::merai::RTMemoryLayout
#include "merai/SharedLogger.h"        // defines seven_axis_robot::merai::multi_ring_logger_memory

namespace seven_axis_robot
{
    namespace fieldbus
    {
        class IoDrive : public BaseDrive
        {
        public:
            IoDrive(const seven_axis_robot::merai::DriveConfig& driveCfg,
                    ec_slave_config_t* sc,
                    ec_domain_t* domain,
                    seven_axis_robot::merai::RTMemoryLayout* rtLayout,
                    int driveIndex,
                    seven_axis_robot::merai::multi_ring_logger_memory* loggerMem);

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
            seven_axis_robot::merai::DriveConfig driveCfg_;
            seven_axis_robot::merai::RTMemoryLayout* rtLayout_ = nullptr;
            int driveIndex_ = -1;

            seven_axis_robot::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            uint16_t digitalInputs_  = 0;
            uint16_t digitalOutputs_ = 0;

            struct IoPdoOffsets
            {
                unsigned int inputOffset  = 0;
                unsigned int outputOffset = 0;
            } ioOffsets_;

            void* getOffsetPointerByIndex(uint16_t index, uint8_t subIndex);
        };
    } // namespace fieldbus
} // namespace seven_axis_robot
