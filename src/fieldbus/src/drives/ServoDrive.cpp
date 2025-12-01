#include "fieldbus/drives/ServoDrive.h"

#include <cstring>   // std::strcmp, etc.
#include <iostream>
#include <stdexcept>

#include "merai/RTIpc.h"

namespace fieldbus
{
    ServoDrive::ServoDrive(const merai::DriveConfig& driveCfg,
                           ec_slave_config_t*        sc,
                           ec_domain_t*              domain,
                           merai::multi_ring_logger_memory* loggerMem)
        : BaseDrive(driveCfg.alias,
                    driveCfg.position,
                    driveCfg.vendor_id,
                    driveCfg.product_code,
                    sc,
                    domain),
          driveCfg_(driveCfg),
          loggerMem_(loggerMem)
    {
        if (loggerMem_)
        {
            merai::log_debug(loggerMem_, "ServoDrive", 3100, "Constructed ServoDrive instance");
        }
    }

    void ServoDrive::initialize()
    {
        if (loggerMem_)
        {
            merai::log_debug(loggerMem_, "ServoDrive", 3101, "Initializing ServoDrive");
        }

        // Zero out local caches
        servoTx_.ctrl.statusWord        = 0;
        servoTx_.motion.positionActual  = 0;
        servoTx_.motion.velocityActual  = 0;
        servoTx_.motion.torqueActual    = 0;
        servoTx_.io.digitalInputs       = 0;
        servoTx_.io.analogInput         = 0;
        servoTx_.io.error_code          = 0;

        servoRx_.ctrl.controlWord       = 0;
        servoRx_.motion.modeOfOperation = 8;
        servoRx_.motion.targetTorque    = 0;
        servoRx_.motion.targetPosition  = 0;
        servoRx_.motion.maxTorque       = 0;
        servoRx_.io.digitalOutputs      = 0;

        if (loggerMem_)
        {
            merai::log_debug(loggerMem_, "ServoDrive", 3102, "ServoDrive Initialized");
        }
    }

    bool ServoDrive::configurePdos()
    {
        if (driveCfg_.syncManagerCount == 0)
        {
            if (loggerMem_)
            {
                merai::log_error(loggerMem_, "ServoDrive", 3401,
                                 "No sync managers found in DriveConfig");
            }
            return false;
        }

        // Configure Sync Managers and PDO assignments from driveCfg_
        for (int smIndex = 0; smIndex < driveCfg_.syncManagerCount; ++smIndex)
        {
            const auto& sm = driveCfg_.syncManagers[smIndex];
            int smId       = sm.id;

            ec_direction_t direction =
                (sm.type == merai::SyncType::RxPdo) ? EC_DIR_OUTPUT : EC_DIR_INPUT;
            ec_watchdog_mode_t wdMode =
                (sm.watchdog_enabled) ? EC_WD_ENABLE : EC_WD_DISABLE;

            if (ecrt_slave_config_sync_manager(slaveConfig_, smId, direction, wdMode))
            {
                if (loggerMem_)
                {
                    merai::log_error(loggerMem_, "ServoDrive", 3402, "Failed to configure SM");
                }
                return false;
            }

            // Clear existing PDO assignments
            if (ecrt_slave_config_pdo_assign_clear(slaveConfig_, smId))
            {
                if (loggerMem_)
                {
                    merai::log_info(loggerMem_, "ServoDrive", 3202,
                                    "Slave config PDO assignment not clear");
                }
                return false;
            }

            // Add each PDO assignment
            for (int assignIdx = 0; assignIdx < sm.assignmentCount; ++assignIdx)
            {
                std::uint16_t assignmentAddr =
                    static_cast<std::uint16_t>(sm.pdo_assignments[assignIdx]);

                if (ecrt_slave_config_pdo_assign_add(slaveConfig_, smId, assignmentAddr))
                {
                    if (loggerMem_)
                    {
                        merai::log_error(loggerMem_, "ServoDrive", 3404,
                                         "Failed to add PDO assignment");
                    }
                    return false;
                }

                // Clear mapping for this assignment
                if (ecrt_slave_config_pdo_mapping_clear(slaveConfig_, assignmentAddr))
                {
                    if (loggerMem_)
                    {
                        merai::log_info(loggerMem_, "ServoDrive", 3203,
                                        "Slave config PDO mapping not clear");
                    }
                    return false;
                }
            }

            // Add PDO entry mappings
            for (int mgIndex = 0; mgIndex < sm.mappingGroupCount; ++mgIndex)
            {
                const auto& mg       = sm.mappingGroups[mgIndex];
                std::uint16_t pdoAddress = static_cast<std::uint16_t>(mg.assignmentKey);

                for (int eIndex = 0; eIndex < mg.entryCount; ++eIndex)
                {
                    const auto& pme = mg.entries[eIndex];
                    std::uint16_t index   = static_cast<std::uint16_t>(pme.object_index);
                    std::uint8_t  subIdx  = static_cast<std::uint8_t>(pme.subindex);
                    std::uint8_t  bitLen  = static_cast<std::uint8_t>(pme.bit_length);

                    if (ecrt_slave_config_pdo_mapping_add(
                            slaveConfig_,
                            pdoAddress,
                            index,
                            subIdx,
                            bitLen))
                    {
                        if (loggerMem_)
                        {
                            merai::log_error(loggerMem_, "ServoDrive", 3403,
                                             "Failed mapping PDOs");
                        }
                        return false;
                    }
                }
            }
        }

        // Register offsets in domain
        if (!registerPdoEntries())
        {
            if (loggerMem_)
            {
                merai::log_warn(loggerMem_, "ServoDrive", 3300,
                                "Failed to register PDO entries.");
            }
            return false;
        }

        if (loggerMem_)
        {
            merai::log_info(loggerMem_, "ServoDrive", 125,
                            "ServoDrive PDOs configured successfully");
        }
        return true;
    }

    bool ServoDrive::registerPdoEntries()
    {
        constexpr int MAX_ENTRIES = 16;
        ec_pdo_entry_reg_t domainRegs[MAX_ENTRIES + 1] = {};
        int idx = 0;

        std::uint16_t alias       = driveCfg_.alias;
        std::uint16_t position    = driveCfg_.position;
        std::uint32_t vendorId    = driveCfg_.vendor_id;
        std::uint32_t productCode = driveCfg_.product_code;

        // Iterate mappings to register offsets
        for (int smIndex = 0; smIndex < driveCfg_.syncManagerCount; ++smIndex)
        {
            const auto& sm = driveCfg_.syncManagers[smIndex];

            for (int mgIndex = 0; mgIndex < sm.mappingGroupCount; ++mgIndex)
            {
                const auto& mg = sm.mappingGroups[mgIndex];

                for (int eIndex = 0; eIndex < mg.entryCount; ++eIndex)
                {
                    if (idx >= MAX_ENTRIES)
                    {
                        if (loggerMem_)
                        {
                            merai::log_warn(loggerMem_, "ServoDrive", 3301,
                                            "Too many PDO entries. Increase MAX_ENTRIES");
                        }
                        return false;
                    }

                    const auto& pme      = mg.entries[eIndex];
                    std::uint16_t objIdx = static_cast<std::uint16_t>(pme.object_index);
                    std::uint8_t  subIdx = static_cast<std::uint8_t>(pme.subindex);

                    void* offsetPtr = getOffsetPointerByIndex(objIdx, subIdx);
                    if (!offsetPtr)
                    {
                        if (loggerMem_)
                        {
                            merai::log_error(loggerMem_, "ServoDrive", 3405,
                                             "Unsupported object_index");
                        }
                        return false;
                    }

                    domainRegs[idx].alias         = alias;
                    domainRegs[idx].position      = position;
                    domainRegs[idx].vendor_id     = vendorId;
                    domainRegs[idx].product_code  = productCode;
                    domainRegs[idx].index         = objIdx;
                    domainRegs[idx].subindex      = subIdx;
                    domainRegs[idx].offset        = static_cast<unsigned int*>(offsetPtr);
                    domainRegs[idx].bit_position  = 0;
                    ++idx;
                }
            }
        }

        // Terminator
        domainRegs[idx] = {};

        if (ecrt_domain_reg_pdo_entry_list(domain_, domainRegs))
        {
            if (loggerMem_)
            {
                merai::log_warn(loggerMem_, "ServoDrive", 3302,
                                "PDO entry registration failed.");
            }
            return false;
        }

        return true;
    }

    bool ServoDrive::readInputs(std::uint8_t* domainPd)
    {
        if (!domainPd)
        {
            return false;
        }

        // EtherCAT domain memory -> servoTx_
        servoTx_.ctrl.statusWord =
            EC_READ_U16(domainPd + servoOffsets_.statusword);

        servoTx_.motion.positionActual =
            EC_READ_S32(domainPd + servoOffsets_.position_actual_value);

        servoTx_.motion.velocityActual =
            EC_READ_S32(domainPd + servoOffsets_.velocity_actual_value);

        servoTx_.motion.torqueActual =
            EC_READ_S16(domainPd + servoOffsets_.torque_actual_value);

        servoTx_.io.digitalInputs =
            EC_READ_U32(domainPd + servoOffsets_.digital_input_value);

        servoTx_.io.analogInput =
            EC_READ_U16(domainPd + servoOffsets_.analog_input_value);

        servoTx_.io.error_code =
            EC_READ_U16(domainPd + servoOffsets_.error_code);

        return true;
    }

    bool ServoDrive::writeOutputs(std::uint8_t* domainPd,
                                  const merai::ServoRxPdo& rxPdo)
    {
        if (!domainPd)
        {
            return false;
        }

        // Copy from provided Rx PDO into local cache
        servoRx_ = rxPdo;

        // Local cache -> EtherCAT domain memory
        EC_WRITE_U16(domainPd + servoOffsets_.controlword,
                     servoRx_.ctrl.controlWord);

        EC_WRITE_S8(domainPd + servoOffsets_.modes_of_operation,
                    static_cast<std::int8_t>(servoRx_.motion.modeOfOperation));

        EC_WRITE_S16(domainPd + servoOffsets_.target_torque,
                     servoRx_.motion.targetTorque);

        EC_WRITE_S32(domainPd + servoOffsets_.target_position,
                     servoRx_.motion.targetPosition);

        EC_WRITE_U16(domainPd + servoOffsets_.max_torque,
                     servoRx_.motion.maxTorque);

        EC_WRITE_U32(domainPd + servoOffsets_.digital_output_value,
                     servoRx_.io.digitalOutputs);

        return true;
    }

    void ServoDrive::handleState(const DriveUserSignals& /*signals*/)
    {
        // Placeholder for future drive state logic (e.g., state machine, faults, etc.)
    }

    void* ServoDrive::getOffsetPointerByIndex(std::uint16_t objectIndex,
                                              std::uint8_t  subIndex)
    {
        // Map object indices -> servoOffsets_ fields
        switch (objectIndex)
        {
        // Inputs (Tx side)
        case 0x6041: return &servoOffsets_.statusword;
        case 0x6064: return &servoOffsets_.position_actual_value;
        case 0x606C: return &servoOffsets_.velocity_actual_value;
        case 0x6077: return &servoOffsets_.torque_actual_value;
        case 0x60FD: return &servoOffsets_.digital_input_value;
        case 0x2401: return &servoOffsets_.analog_input_value;
        case 0x603F: return &servoOffsets_.error_code;

        // Outputs (Rx side)
        case 0x6040: return &servoOffsets_.controlword;
        case 0x6060: return &servoOffsets_.modes_of_operation;
        case 0x607A: return &servoOffsets_.target_position;
        case 0x6071: return &servoOffsets_.target_torque;
        case 0x6072: return &servoOffsets_.max_torque;
        case 0x60FE:
            if (subIndex == 1)
            {
                return &servoOffsets_.digital_output_value;
            }
            return nullptr;

        default:
            return nullptr;
        }
    }

} // namespace fieldbus
