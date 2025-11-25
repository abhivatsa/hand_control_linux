#pragma once

#include <cstdint>
#include <ecrt.h>

namespace seven_axis_robot
{
    namespace fieldbus
    {
        struct DriveUserSignals
        {
            // For servo-like drives
            bool allowOperation = false;
            bool quickStop = false;
            bool faultReset = false;
            bool forceDisable = false;

            // For IO-like drives
            bool enableOutputs = false;
            bool readInputs = false;

            // Future placeholders
            bool calibrateSensor = false;
        };

        class BaseDrive
        {
        public:
            virtual ~BaseDrive() = default;

            virtual void initialize() = 0;
            virtual bool configurePdos() = 0;
            virtual bool readInputs(uint8_t* domainPd) = 0;
            virtual bool writeOutputs(uint8_t* domainPd) = 0;
            virtual void handleState(const DriveUserSignals& signals) = 0;

            // Accessors
            inline uint16_t alias() const
            {
                return alias_;
            }

            inline uint16_t position() const
            {
                return position_;
            }

        protected:
            BaseDrive(uint16_t alias,
                      uint16_t position,
                      uint32_t vendorId,
                      uint32_t productCode,
                      ec_slave_config_t* sc,
                      ec_domain_t* domain)
                : alias_(alias),
                  position_(position),
                  vendorId_(vendorId),
                  productCode_(productCode),
                  slaveConfig_(sc),
                  domain_(domain)
            {
            }

        protected:
            // Basic identifying info
            uint16_t alias_;
            uint16_t position_;
            uint32_t vendorId_;
            uint32_t productCode_;

            // EtherCAT pointers
            ec_slave_config_t* slaveConfig_ = nullptr;
            ec_domain_t*       domain_      = nullptr;
        };
    } // namespace fieldbus
} // namespace seven_axis_robot
