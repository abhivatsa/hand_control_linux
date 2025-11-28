#pragma once

#include <cstdint>
#include <string>
#include <ecrt.h> // for ec_slave_config_t, ec_domain_t

#include "fieldbus/drives/BaseDrive.h"
#include "merai/ParameterServer.h" // defines merai::DriveConfig
#include "merai/RTMemoryLayout.h"  // defines ServoRxPdo, ServoTxPdo
#include "merai/SharedLogger.h"    // defines merai::multi_ring_logger_memory

namespace fieldbus
{
    /**
     * @brief ServoDrive
     * Handles reading/writing PDOs for a single servo drive using the new RTMemoryLayout.
     */
    class ServoDrive : public BaseDrive
    {
    public:
        ServoDrive(const merai::DriveConfig &driveCfg,
                   ec_slave_config_t *sc,
                   ec_domain_t *domain,
                   merai::multi_ring_logger_memory *loggerMem);

        ~ServoDrive() override = default;

        void initialize() override;
        bool configurePdos() override;

        bool readInputs(uint8_t *domainPd) override;

        // Expose the local Tx PDO so EthercatMaster can copy it into servoTxBuffer.
        const merai::ServoTxPdo &txPdo() const { return servoTx_; }

        /**
         * @brief writeOutputs
         *  - Copies the provided rxPdo into servoRx_ struct
         *  - Writes EtherCAT domain memory (controlWord, targetTorque, etc.)
         */
        bool writeOutputs(uint8_t *domainPd, const merai::ServoRxPdo &rxPdo);

        // Legacy override (unused by EthercatMaster for servo drives, kept for interface compatibility).
        bool writeOutputs(uint8_t *domainPd) override
        {
            // Writes whatever is currently in servoRx_.
            return writeOutputs(domainPd, servoRx_);
        }

        /**
         * @brief handleState
         *  - Further logic if needed (e.g. drive state transitions).
         */
        void handleState(const DriveUserSignals &signals) override;

    private:
        ec_domain_t *domain_ = nullptr;

        merai::DriveConfig driveCfg_;

        // Logging pointer
        merai::multi_ring_logger_memory *loggerMem_ = nullptr;

        // Data structures for servo I/O
        // Using the updated sub-struct style
        merai::ServoTxPdo servoTx_; // read from domain -> store in .tx
        merai::ServoRxPdo servoRx_; // read from command -> write to domain

        struct ServoOffsets
        {
            // Offsets for inputs (Tx)
            unsigned int statusword = 0;            // 0x6041
            unsigned int position_actual_value = 0; // 0x6064
            unsigned int velocity_actual_value = 0; // 0x606C
            unsigned int torque_actual_value = 0;   // 0x6077
            unsigned int digital_input_value = 0;   // 0x60FD
            unsigned int analog_input_value = 0;    // 0x2401
            unsigned int error_code = 0;            // 0x603F

            // Offsets for outputs (Rx)
            unsigned int controlword = 0;          // 0x6040
            unsigned int modes_of_operation = 0;   // 0x6060
            unsigned int target_position = 0;      // 0x607A
            unsigned int target_torque = 0;        // 0x6071
            unsigned int max_torque = 0;           // 0x6072
            unsigned int digital_output_value = 0; // 0x60FE:01
        } servoOffsets_;

        void *getOffsetPointerByIndex(uint16_t objectIndex, uint8_t subIndex);

        bool registerPdoEntries();
    };

} // namespace fieldbus
