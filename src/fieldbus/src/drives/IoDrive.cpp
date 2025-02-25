#include <stdexcept>
#include <iostream>

#include "fieldbus/drives/IoDrive.h"

namespace motion_control
{
    namespace fieldbus
    {
        IoDrive::IoDrive(const motion_control::merai::DriveConfig &driveCfg,
                         ec_slave_config_t *sc,
                         ec_domain_t *domain,
                         motion_control::merai::RTMemoryLayout *rtLayout,
                         int driveIndex,
                         motion_control::merai::multi_ring_logger_memory *loggerMem)
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
                motion_control::merai::log_debug(loggerMem_,
                                                 "IoDrive",
                                                 100,
                                                 "Constructed IoDrive instance");
            }
        }

        void IoDrive::initialize()
        {
            digitalOutputs_ = 0;
            digitalInputs_ = 0;

            if (loggerMem_)
            {
                log_info(loggerMem_, "IoDrive", 101, "IoDrive initialized");
            }
        }

        bool IoDrive::configurePdos()
        {
            if (driveCfg_.syncManagerCount == 0)
            {
                std::cerr << "IoDrive: No sync managers found in DriveConfig.\n";
                if (loggerMem_)
                {
                    log_error(loggerMem_, "IoDrive", 110, "No sync managers found in DriveConfig");
                }
                return false;
            }

            // Configure each Sync Manager
            for (int smIndex = 0; smIndex < driveCfg_.syncManagerCount; ++smIndex)
            {
                const auto &sm = driveCfg_.syncManagers[smIndex];
                int smId = sm.id;
                ec_direction_t direction = (sm.type == "rxpdo") ? EC_DIR_OUTPUT : EC_DIR_INPUT;
                ec_watchdog_mode_t wdMode = sm.watchdog_enabled ? EC_WD_ENABLE : EC_WD_DISABLE;

                if (ecrt_slave_config_sync_manager(slaveConfig_, smId, direction, wdMode))
                {
                    std::cerr << "IoDrive: Failed to configure SM " << smId << "\n";
                    if (loggerMem_)
                    {
                        log_error(loggerMem_, "IoDrive", 111, "Failed to configure SM");
                    }
                    return false;
                }

                // Clear existing PDO assignments
                ecrt_slave_config_pdo_assign_clear(slaveConfig_, smId);

                // Add each PDO assignment
                for (int assignIdx = 0; assignIdx < sm.assignmentCount; ++assignIdx)
                {
                    uint16_t assignmentAddr = hexStringToUint(sm.pdo_assignments[assignIdx]);
                    if (ecrt_slave_config_pdo_assign_add(slaveConfig_, smId, assignmentAddr))
                    {
                        std::cerr << "IoDrive: Failed to add PDO assignment 0x"
                                  << std::hex << assignmentAddr
                                  << " to SM " << smId << std::dec << "\n";

                        if (loggerMem_)
                        {
                            log_error(loggerMem_, "IoDrive", 112, "Failed to add PDO assignment to SM");
                        }
                        return false;
                    }
                    ecrt_slave_config_pdo_mapping_clear(slaveConfig_, assignmentAddr);
                }

                // Add PDO entries
                for (int mgIndex = 0; mgIndex < sm.mappingGroupCount; ++mgIndex)
                {
                    const auto &mg = sm.mappingGroups[mgIndex];
                    uint16_t pdoAddress = hexStringToUint(mg.assignmentKey);

                    for (int eIndex = 0; eIndex < mg.entryCount; ++eIndex)
                    {
                        const auto &pme = mg.entries[eIndex];
                        uint16_t index = hexStringToUint(pme.object_index);
                        uint8_t subIdx = static_cast<uint8_t>(pme.subindex);
                        uint8_t bitLen = static_cast<uint8_t>(pme.bit_length);

                        if (ecrt_slave_config_pdo_mapping_add(
                                slaveConfig_,
                                pdoAddress,
                                index,
                                subIdx,
                                bitLen))
                        {
                            std::cerr << "IoDrive: Failed mapping PDO 0x"
                                      << std::hex << pdoAddress
                                      << ", index=0x" << index
                                      << ", subIndex=" << std::dec << (int)subIdx
                                      << ", bits=" << (int)bitLen << "\n";

                            if (loggerMem_)
                            {
                                log_error(loggerMem_, "IoDrive", 113,
                                          "Failed pdo_mapping_add for IoDrive");
                            }
                            return false;
                        }
                    }
                }
            }

            // Register PDO entries
            if (!registerPdoEntries())
            {
                std::cerr << "IoDrive: Failed to register PDO entries.\n";
                if (loggerMem_)
                {
                    log_error(loggerMem_, "IoDrive", 114, "Failed to register PDO entries for IoDrive");
                }
                return false;
            }

            if (loggerMem_)
            {
                log_info(loggerMem_, "IoDrive", 115, "IoDrive PDOs configured successfully");
            }
            return true;
        }

        bool IoDrive::registerPdoEntries()
        {
            constexpr int MAX_ENTRIES = 10;
            ec_pdo_entry_reg_t domainRegs[MAX_ENTRIES + 1] = {};
            int idx = 0;

            uint16_t alias = driveCfg_.alias;
            uint16_t position = driveCfg_.position;
            uint32_t vendorId = driveCfg_.vendor_id;
            uint32_t productCode = driveCfg_.product_code;

            // Example: register digital input at 0x6000:0x01
            if (idx < MAX_ENTRIES)
            {
                auto &reg = domainRegs[idx++];
                reg.alias = alias;
                reg.position = position;
                reg.vendor_id = vendorId;
                reg.product_code = productCode;
                reg.index = 0x6000;
                reg.subindex = 0x01;
                reg.offset = reinterpret_cast<unsigned int *>(&ioOffsets_.inputOffset);
                reg.bit_position = 0;
            }
            else
            {
                std::cerr << "IoDrive: Too many PDO entries.\n";
                if (loggerMem_)
                {
                    log_error(loggerMem_, "IoDrive", 116, "Max PDO entries exceeded in IoDrive");
                }
                return false;
            }

            // Example: register digital output at 0x7000:0x01
            if (idx < MAX_ENTRIES)
            {
                auto &reg = domainRegs[idx++];
                reg.alias = alias;
                reg.position = position;
                reg.vendor_id = vendorId;
                reg.product_code = productCode;
                reg.index = 0x7000;
                reg.subindex = 0x01;
                reg.offset = reinterpret_cast<unsigned int *>(&ioOffsets_.outputOffset);
                reg.bit_position = 0;
            }
            else
            {
                std::cerr << "IoDrive: Too many PDO entries.\n";
                if (loggerMem_)
                {
                    log_error(loggerMem_, "IoDrive", 117, "Max PDO entries exceeded in IoDrive");
                }
                return false;
            }

            // Terminate
            domainRegs[idx] = ec_pdo_entry_reg_t{};

            if (ecrt_domain_reg_pdo_entry_list(domain_, domainRegs))
            {
                std::cerr << "IoDrive: PDO entry registration failed.\n";
                if (loggerMem_)
                {
                    log_error(loggerMem_, "IoDrive", 118, "PDO entry registration failed");
                }
                return false;
            }
            return true;
        }

        bool IoDrive::readInputs(uint8_t *domainPd)
        {
            if (!domainPd)
            {
                return false;
            }
            digitalInputs_ = EC_READ_U16(domainPd + ioOffsets_.inputOffset);
            return true;
        }

        bool IoDrive::writeOutputs(uint8_t *domainPd)
        {
            if (!domainPd)
            {
                return false;
            }
            EC_WRITE_U16(domainPd + ioOffsets_.outputOffset, digitalOutputs_);
            return true;
        }

        void IoDrive::handleState(const DriveUserSignals & /*signals*/)
        {
            // No advanced logic for simple I/O
        }

        void IoDrive::setDigitalOutput(uint16_t output)
        {
            digitalOutputs_ = output;
            if (loggerMem_)
            {
                log_debug(loggerMem_, "IoDrive", 130, "Digital output updated");
            }
        }

        uint16_t IoDrive::getDigitalInput() const
        {
            return digitalInputs_;
        }

        uint16_t IoDrive::hexStringToUint(const std::string &hexStr)
        {
            return static_cast<uint16_t>(std::strtoul(hexStr.c_str(), nullptr, 16));
        }

        void *IoDrive::getOffsetPointerByIndex(uint16_t /*index*/, uint8_t /*subIndex*/)
        {
            return nullptr;
        }
    } // namespace fieldbus
} // namespace motion_control
