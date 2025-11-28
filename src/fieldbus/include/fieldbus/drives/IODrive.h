#pragma once

#include <cstdint>
#include <string>
#include <ecrt.h> // ec_slave_config_t, ec_domain_t

#include "fieldbus/drives/BaseDrive.h"
#include "merai/ParameterServer.h" // defines merai::DriveConfig
#include "merai/SharedLogger.h"    // defines merai::multi_ring_logger_memory

namespace fieldbus
{
    class IoDrive : public BaseDrive
    {
    public:
        IoDrive(const merai::DriveConfig &driveCfg,
                ec_slave_config_t *sc,
                ec_domain_t *domain,
                merai::multi_ring_logger_memory *loggerMem);

        virtual ~IoDrive() = default;

        void initialize() override;
        bool configurePdos() override;
        bool readInputs(uint8_t *domainPd) override;
        bool writeOutputs(uint8_t *domainPd) override;
        void handleState(const DriveUserSignals &signals) override;

        bool registerPdoEntries();

        void setDigitalOutput(uint16_t output);
        uint16_t getDigitalInput() const;

    private:
        ec_domain_t *domain_ = nullptr;

        merai::DriveConfig driveCfg_;

        merai::multi_ring_logger_memory *loggerMem_ = nullptr;

        uint16_t digitalInputs_ = 0;
        uint16_t digitalOutputs_ = 0;

        struct IoPdoOffsets
        {
            unsigned int inputOffset = 0;
            unsigned int outputOffset = 0;
        } ioOffsets_;

        void *getOffsetPointerByIndex(uint16_t index, uint8_t subIndex);
    };
} // namespace fieldbus
