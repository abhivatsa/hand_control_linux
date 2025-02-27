#include <stdexcept>
#include <iostream>

#include "fieldbus/drives/ServoDrive.h"

namespace hand_control
{
    namespace fieldbus
    {
        ServoDrive::ServoDrive(const hand_control::merai::DriveConfig& driveCfg,
                               ec_slave_config_t* sc,
                               ec_domain_t* domain,
                               hand_control::merai::RTMemoryLayout* rtLayout,
                               int driveIndex,
                               hand_control::merai::multi_ring_logger_memory* loggerMem)
            : BaseDrive(driveCfg.alias,
                        driveCfg.position,
                        driveCfg.vendor_id,
                        driveCfg.product_code,
                        sc,
                        domain),
              driveCfg_(driveCfg),
              domain_(domain),
              rtLayout_(rtLayout),
              driveIndex_(driveIndex),
              loggerMem_(loggerMem)
        {
            if (loggerMem_)
            {
                hand_control::merai::log_debug(
                    loggerMem_,
                    "ServoDrive",
                    100,
                    "Constructed ServoDrive instance"
                );
            }
        }

        void ServoDrive::initialize()
        {
            if (loggerMem_)
            {
                hand_control::merai::log_info(
                    loggerMem_,
                    "ServoDrive",
                    110,
                    "Initializing ServoDrive"
                );
            }

            servoTx_.statusWord     = 0;
            servoTx_.positionActual = 0;
            servoTx_.velocityActual = 0;
            servoTx_.torqueActual   = 0;

            servoRx_.controlWord      = 0;
            servoRx_.modeOfOperation  = 0;
            servoRx_.targetTorque     = 0;
            servoRx_.targetPosition   = 0;

            if (loggerMem_)
            {
                hand_control::merai::log_info(
                    loggerMem_,
                    "ServoDrive",
                    111,
                    "ServoDrive initialized"
                );
            }
        }

        bool ServoDrive::configurePdos()
        {
            if (driveCfg_.syncManagerCount == 0)
            {
                std::cerr << "[ServoDrive] No sync managers found in DriveConfig.\n";
                if (loggerMem_)
                {
                    hand_control::merai::log_error(
                        loggerMem_,
                        "ServoDrive",
                        120,
                        "No sync managers found in DriveConfig"
                    );
                }
                return false;
            }

            for (int smIndex = 0; smIndex < driveCfg_.syncManagerCount; ++smIndex)
            {
                const auto& sm = driveCfg_.syncManagers[smIndex];
                int smId       = sm.id;
                ec_direction_t direction  = (std::strcmp(sm.type.c_str(), "rxpdo") == 0) ? EC_DIR_OUTPUT : EC_DIR_INPUT;
                ec_watchdog_mode_t wdMode = sm.watchdog_enabled ? EC_WD_ENABLE : EC_WD_DISABLE;

                if (ecrt_slave_config_sync_manager(slaveConfig_, smId, direction, wdMode))
                {
                    std::cerr << "ServoDrive: Failed to configure SM " << smId << "\n";
                    if (loggerMem_)
                    {
                        hand_control::merai::log_error(
                            loggerMem_,
                            "ServoDrive",
                            121,
                            "Failed to configure SM for servo drive"
                        );
                    }
                    return false;
                }

                // Clear existing PDO assignments
                ecrt_slave_config_pdo_assign_clear(slaveConfig_, smId);

                // Add each PDO assignment
                for (int assignIdx = 0; assignIdx < sm.assignmentCount; ++assignIdx)
                {
                    uint16_t assignmentAddr = hexStringToUint(sm.pdo_assignments[assignIdx].c_str());
                    if (ecrt_slave_config_pdo_assign_add(slaveConfig_, smId, assignmentAddr))
                    {
                        std::cerr << "ServoDrive: Failed to add PDO assignment 0x"
                                  << std::hex << assignmentAddr
                                  << " to SM " << smId << std::dec << "\n";
                        if (loggerMem_)
                        {
                            hand_control::merai::log_error(
                                loggerMem_,
                                "ServoDrive",
                                122,
                                "Failed to add PDO assignment to SM"
                            );
                        }
                        return false;
                    }
                    ecrt_slave_config_pdo_mapping_clear(slaveConfig_, assignmentAddr);
                }

                // Add PDO entry mappings
                for (int mgIndex = 0; mgIndex < sm.mappingGroupCount; ++mgIndex)
                {
                    const auto& mg = sm.mappingGroups[mgIndex];
                    uint16_t pdoAddress = hexStringToUint(mg.assignmentKey.c_str());

                    for (int eIndex = 0; eIndex < mg.entryCount; ++eIndex)
                    {
                        const auto& pme = mg.entries[eIndex];
                        uint16_t index  = hexStringToUint(pme.object_index.c_str());
                        uint8_t subIdx  = static_cast<uint8_t>(pme.subindex);
                        uint8_t bitLen  = static_cast<uint8_t>(pme.bit_length);

                        if (ecrt_slave_config_pdo_mapping_add(
                                slaveConfig_,
                                pdoAddress,
                                index,
                                subIdx,
                                bitLen))
                        {
                            std::cerr << "ServoDrive: Failed mapping PDO 0x"
                                      << std::hex << pdoAddress
                                      << ", index=0x" << index
                                      << ", subIndex=" << std::dec << (int)subIdx
                                      << ", bits=" << (int)bitLen << "\n";

                            if (loggerMem_)
                            {
                                hand_control::merai::log_error(
                                    loggerMem_,
                                    "ServoDrive",
                                    123,
                                    "Failed pdo_mapping_add for servo drive"
                                );
                            }
                            return false;
                        }
                    }
                }
            }

            if (!registerPdoEntries())
            {
                std::cerr << "ServoDrive: Failed to register PDO entries.\n";
                if (loggerMem_)
                {
                    hand_control::merai::log_error(
                        loggerMem_,
                        "ServoDrive",
                        124,
                        "Failed to register PDO entries for servo drive"
                    );
                }
                return false;
            }

            if (loggerMem_)
            {
                hand_control::merai::log_info(
                    loggerMem_,
                    "ServoDrive",
                    125,
                    "ServoDrive PDOs configured successfully"
                );
            }
            return true;
        }

        bool ServoDrive::registerPdoEntries()
        {
        constexpr int MAX_ENTRIES = 10;
        ec_pdo_entry_reg_t domainRegs[MAX_ENTRIES + 1] = {};
        int idx = 0;

        uint16_t alias = driveCfg_.alias;
        uint16_t position = driveCfg_.position;
        uint32_t vendorId = driveCfg_.vendor_id;
        uint32_t productCode = driveCfg_.product_code;

        // Re-visit the same pdo mappings to register offsets
        for (int smIndex = 0; smIndex < driveCfg_.syncManagerCount; smIndex++)
        {
            const auto &sm = driveCfg_.syncManagers[smIndex];
            for (int mgIndex = 0; mgIndex < sm.mappingGroupCount; mgIndex++)
            {
                const auto &mg = sm.mappingGroups[mgIndex];
                uint16_t pdoAddress = hexStringToUint(mg.assignmentKey.c_str());

                for (int eIndex = 0; eIndex < mg.entryCount; eIndex++)
                {
                    // 1) Ensure we don't exceed our fixed capacity
                    if (idx >= MAX_ENTRIES)
                    {
                        std::cerr << "ServoDrive: Too many PDO entries. Increase MAX_ENTRIES.\n";
                        return false;
                    }

                    const auto &pme = mg.entries[eIndex];
                    uint16_t objIndex = hexStringToUint(pme.object_index.c_str());
                    uint8_t subIndex = static_cast<uint8_t>(pme.subindex);

                    // getOffsetPointerByIndex(...) decides
                    // which servoOffsets_ field to store for that objIndex
                    void *offsetPtr = getOffsetPointerByIndex(objIndex);
                    if (!offsetPtr)
                    {
                        std::cerr << "ServoDrive: Unknown or unsupported object_index=0x"
                                  << std::hex << objIndex << std::dec << "\n";
                        return false;
                    }

                    // 2) Populate one ec_pdo_entry_reg_t
                    ec_pdo_entry_reg_t &reg = domainRegs[idx++];
                    reg.alias = alias;
                    reg.position = position;
                    reg.vendor_id = vendorId;
                    reg.product_code = productCode;
                    reg.index = objIndex;
                    reg.subindex = subIndex;
                    reg.offset = static_cast<unsigned int *>(offsetPtr);
                    reg.bit_position = 0; // 0 if byte-aligned
                }
            }
        }

        // Terminate the registration list
        domainRegs[idx] = ec_pdo_entry_reg_t{};

        if (ecrt_domain_reg_pdo_entry_list(domain_, domainRegs))
        {
            std::cerr << "ServoDrive: PDO entry registration failed.\n";
            if (loggerMem_)
            {
                log_error(loggerMem_, "ServoDrive", 118, "PDO entry registration failed");
            }
            return false;
        }

        return true;
    }



        bool ServoDrive::readInputs(uint8_t* domainPd)
        {
            if (!domainPd)
            {
                return false;
            }

            servoTx_.statusWord     = EC_READ_U16(domainPd + servoOffsets_.statusword);
            servoTx_.positionActual = EC_READ_S32(domainPd + servoOffsets_.position_actual_value);
            servoTx_.velocityActual = EC_READ_S32(domainPd + servoOffsets_.velocity_actual_value);
            servoTx_.torqueActual   = EC_READ_S16(domainPd + servoOffsets_.torque_actual_value);

            if (rtLayout_)
            {
                int frontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
                auto& tx     = rtLayout_->servoBuffer.buffer[frontIdx].tx[driveIndex_];

                tx.statusWord     = servoTx_.statusWord;
                tx.positionActual = servoTx_.positionActual;
                tx.velocityActual = servoTx_.velocityActual;
                tx.torqueActual   = servoTx_.torqueActual;
            }
            return true;
        }

        bool ServoDrive::writeOutputs(uint8_t* domainPd)
        {
            if (!domainPd)
            {
                return false;
            }

            if (rtLayout_)
            {
                int frontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
                auto& rx     = rtLayout_->servoBuffer.buffer[frontIdx].rx[driveIndex_];

                servoRx_.controlWord      = rx.controlWord;
                servoRx_.modeOfOperation  = rx.modeOfOperation;
                servoRx_.targetTorque     = rx.targetTorque;
                servoRx_.targetPosition   = rx.targetPosition;
            }

            EC_WRITE_U16(domainPd + servoOffsets_.controlword,      servoRx_.controlWord);
            EC_WRITE_S8 (domainPd + servoOffsets_.modes_of_operation, servoRx_.modeOfOperation);
            EC_WRITE_S16(domainPd + servoOffsets_.target_torque,      servoRx_.targetTorque);
            EC_WRITE_S32(domainPd + servoOffsets_.target_position,    servoRx_.targetPosition);

            return true;
        }

        void ServoDrive::handleState(const DriveUserSignals& /*signals*/)
        {
            // No further logic here yet
        }

        uint16_t ServoDrive::hexStringToUint(const std::string& hexStr)
        {
            return static_cast<uint16_t>(std::strtoul(hexStr.c_str(), nullptr, 16));
        }

        void* ServoDrive::getOffsetPointerByIndex(uint16_t objectIndex)
        {
            switch (objectIndex)
            {
            case 0x6041: return &servoOffsets_.statusword;
            case 0x6061: return &servoOffsets_.mode_of_operation_display;
            case 0x6064: return &servoOffsets_.position_actual_value;
            case 0x606C: return &servoOffsets_.velocity_actual_value;
            case 0x6077: return &servoOffsets_.torque_actual_value;
            case 0x603F: return &servoOffsets_.error_code;
            case 0x6040: return &servoOffsets_.controlword;
            case 0x6060: return &servoOffsets_.modes_of_operation;
            case 0x6071: return &servoOffsets_.target_torque;
            case 0x607A: return &servoOffsets_.target_position;
            default:     return nullptr;
            }
        }
    } // namespace fieldbus
} // namespace hand_control
