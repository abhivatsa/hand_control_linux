#pragma once

#include <cstdint>
#include <string>

#include <ecrt.h> // ec_slave_config_t, ec_domain_t

#include "fieldbus/drives/BaseDrive.h"
#include "merai/ParameterServer.h" // merai::DriveConfig
#include "merai/RTMemoryLayout.h"  // ServoRxPdo, ServoTxPdo
#include "merai/SharedLogger.h"    // multi_ring_logger_memory

namespace fieldbus
{
    /**
     * @brief ServoDrive
     * Handles reading/writing PDOs for a single servo drive using RTMemoryLayout's PDO structs.
     */
    class ServoDrive : public BaseDrive
    {
    public:
        ServoDrive(const merai::DriveConfig& driveCfg,
                   ec_slave_config_t*        sc,
                   ec_domain_t*              domain,
                   merai::multi_ring_logger_memory* loggerMem);

        ~ServoDrive() override = default;

        void initialize() override;
        bool configurePdos() override;

        // EtherCAT -> local Tx PDO cache
        bool readInputs(std::uint8_t* domainPd) override;

        // Expose the local Tx PDO so EthercatMaster can copy it into servoTxBuffer.
        const merai::ServoTxPdo& txPdo() const { return servoTx_; }

        /**
         * @brief writeOutputs
         *  - Copies the provided rxPdo into servoRx_ struct
         *  - Writes EtherCAT domain memory (controlWord, targetTorque, etc.)
         */
        bool writeOutputs(std::uint8_t* domainPd, const merai::ServoRxPdo& rxPdo);

        // Override for BaseDrive: uses whatever is currently in servoRx_
        bool writeOutputs(std::uint8_t* domainPd) override
        {
            return writeOutputs(domainPd, servoRx_);
        }

        void handleState(const DriveUserSignals& signals) override;

    private:
        // Mapping from object indices to offset storage
        void* getOffsetPointerByIndex(std::uint16_t objectIndex, std::uint8_t subIndex);

        bool registerPdoEntries();

    private:
        merai::DriveConfig                 driveCfg_;
        merai::multi_ring_logger_memory*   loggerMem_ = nullptr;

        // Local data structures for servo I/O
        merai::ServoTxPdo                  servoTx_{}; // EtherCAT -> here
        merai::ServoRxPdo                  servoRx_{}; // SHM command -> here -> EtherCAT

        struct ServoOffsets
        {
            // Inputs (Tx side)
            unsigned int statusword            = 0; // 0x6041
            unsigned int position_actual_value = 0; // 0x6064
            unsigned int velocity_actual_value = 0; // 0x606C
            unsigned int torque_actual_value   = 0; // 0x6077
            unsigned int digital_input_value   = 0; // 0x60FD
            unsigned int analog_input_value    = 0; // 0x2401
            unsigned int error_code            = 0; // 0x603F

            // Outputs (Rx side)
            unsigned int controlword           = 0; // 0x6040
            unsigned int modes_of_operation    = 0; // 0x6060
            unsigned int target_position       = 0; // 0x607A
            unsigned int target_torque         = 0; // 0x6071
            unsigned int max_torque            = 0; // 0x6072
            unsigned int digital_output_value  = 0; // 0x60FE:01
        } servoOffsets_{};
    };

} // namespace fieldbus
