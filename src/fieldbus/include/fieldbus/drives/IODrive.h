#pragma once

#include <cstdint>
#include <string>

#include <ecrt.h> // ec_slave_config_t, ec_domain_t

#include "fieldbus/drives/BaseDrive.h"
#include "merai/ParameterServer.h" // merai::DriveConfig
#include "merai/SharedLogger.h"    // merai::multi_ring_logger_memory

namespace fieldbus
{
    /**
     * @brief Simple / stub IO drive adapter.
     *
     * Right now this is mostly a placeholder:
     *  - It configures Sync Managers + PDO mappings from DriveConfig.
     *  - PDO entry offsets are not fully wired (getOffsetPointerByIndex() returns nullptr),
     *    so configurePdos() will fail if an IoDrive appears in the config.
     *
     * This is OK as long as you don't actually configure IO-type drives yet.
     * When you do, you'll implement getOffsetPointerByIndex() properly.
     */
    class IoDrive : public BaseDrive
    {
    public:
        IoDrive(const merai::DriveConfig& driveCfg,
                ec_slave_config_t*        sc,
                ec_domain_t*              domain,
                merai::multi_ring_logger_memory* loggerMem);

        ~IoDrive() override = default;

        void initialize() override;
        bool configurePdos() override;
        bool readInputs(std::uint8_t* domainPd) override;
        bool writeOutputs(std::uint8_t* domainPd) override;
        void handleState(const DriveUserSignals& signals) override;

        bool registerPdoEntries();

        void     setDigitalOutput(std::uint16_t output);
        std::uint16_t getDigitalInput() const;

    private:
        merai::DriveConfig              driveCfg_;
        merai::multi_ring_logger_memory* loggerMem_ = nullptr;

        std::uint16_t digitalInputs_  = 0;
        std::uint16_t digitalOutputs_ = 0;

        struct IoPdoOffsets
        {
            unsigned int inputOffset  = 0;
            unsigned int outputOffset = 0;
        } ioOffsets_{};

        void* getOffsetPointerByIndex(std::uint16_t index, std::uint8_t subIndex);
    };

} // namespace fieldbus
