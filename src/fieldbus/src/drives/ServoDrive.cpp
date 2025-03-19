#include <stdexcept>
#include <iostream>
#include <cstring> // for std::strcmp, std::strtoul
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
                log_debug( loggerMem_, "ServoDrive", 3100, "Constructed ServoDrive instance" );
            }
        }

        void ServoDrive::initialize()
        {
            if (loggerMem_)
            {
                log_debug( loggerMem_, "ServoDrive", 3101, "Initializing ServoDrive" );
            }

            // Zero out local caches
            servoTx_.ctrl.statusWord = 0;
            servoTx_.motion.positionActual = 0;
            servoTx_.motion.velocityActual = 0;
            servoTx_.motion.torqueActual   = 0;
            servoTx_.io.digitalInputs = 0;
            servoTx_.io.analogInput   = 0;
            

            servoRx_.ctrl.controlWord    = 0;
            servoRx_.motion.modeOfOperation= 0;
            servoRx_.motion.targetTorque    = 0;
            servoRx_.motion.targetPosition  = 0;

            if (loggerMem_)
            {
                log_debug( loggerMem_, "ServoDrive", 3102, "ServoDrive Initialized" );
            }
        }

        bool ServoDrive::configurePdos()
        {
            if (driveCfg_.syncManagerCount == 0)
            {
                // std::cerr << "[ServoDrive] No sync managers found in DriveConfig.\n";

                if (loggerMem_)
                {
                    log_error( loggerMem_, "ServoDrive", 3401, "No sync managers found in DriveConfig" );
                }
                return false;
            }

            // Configure Sync Managers and PDO assignments from driveCfg_...
            for (int smIndex = 0; smIndex < driveCfg_.syncManagerCount; ++smIndex)
            {
                const auto& sm = driveCfg_.syncManagers[smIndex];
                int smId = sm.id;

                ec_direction_t direction =
                    (std::strcmp(sm.type.c_str(), "rxpdo") == 0) ? EC_DIR_OUTPUT : EC_DIR_INPUT;
                ec_watchdog_mode_t wdMode =
                    (sm.watchdog_enabled) ? EC_WD_ENABLE : EC_WD_DISABLE;

                if (ecrt_slave_config_sync_manager(slaveConfig_, smId, direction, wdMode))
                {
                    // std::cerr << "ServoDrive: Failed to configure SM " << smId << "\n";
                    if (loggerMem_){
                        log_error( loggerMem_, "ServoDrive", 3402, "Failed to configure SM" );
                    }
                    return false;
                }

                // Added log msg to the function
                if (ecrt_slave_config_pdo_assign_clear(slaveConfig_, smId))
                {
                    if (loggerMem_){
                        log_info( loggerMem_, "ServoDrive", 3202, "Slave config PDO assignment not clear");
                    }
                    return false;
                }

                // Add each PDO assignment
                for (int assignIdx = 0; assignIdx < sm.assignmentCount; ++assignIdx)
                {
                    uint16_t assignmentAddr = hexStringToUint(sm.pdo_assignments[assignIdx].c_str());
                    if (ecrt_slave_config_pdo_assign_add(slaveConfig_, smId, assignmentAddr))
                    {
                        // std::cerr << "ServoDrive: Failed to add PDO assignment 0x"
                        //           << std::hex << assignmentAddr
                        //           << " to SM " << smId << std::dec << "\n";
                        log_error( loggerMem_, "ServoDrive", 3404, "Failed to add PDO assignment");
                        return false;
                    }

                    // Added log msg to the function
                    if (ecrt_slave_config_pdo_mapping_clear(slaveConfig_, assignmentAddr)) 
                    {
                        if (loggerMem_)
                        {
                            log_info( loggerMem_, "ServoDrive", 3203, "Slave config PDO mapping not clear" );
                        }
                        return false;
                    }
                }

                // Add PDO entry mappings
                for (int mgIndex = 0; mgIndex < sm.mappingGroupCount; ++mgIndex)
                {
                    const auto &mg = sm.mappingGroups[mgIndex];
                    uint16_t pdoAddress = hexStringToUint(mg.assignmentKey.c_str());

                    for (int eIndex = 0; eIndex < mg.entryCount; ++eIndex)
                    {
                        const auto &pme = mg.entries[eIndex];
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
                            // std::cerr << "ServoDrive: Failed mapping PDO 0x"
                            //           << std::hex << pdoAddress
                            //           << ", index=0x" << index
                            //           << ", subIndex=" << std::dec << (int)subIdx
                            //           << ", bits=" << (int)bitLen << "\n";
                            log_error( loggerMem_, "ServoDrive", 3403, "Failed mapping PDOs");
                            return false;
                        }
                    }
                }
            }

            // Register offsets in domain
            if (!registerPdoEntries())
            {
                // std::cerr << "ServoDrive: Failed to register PDO entries.\n";
                log_warn( loggerMem_, "ServoDrive", 3300, "Failed to register PDO entries." );
                return false;
            }

            if (loggerMem_)
            {
                log_info( loggerMem_, "ServoDrive", 125, "ServoDrive PDOs configured successfully" );
            }
            return true;
        }

        bool ServoDrive::registerPdoEntries()
        {
            constexpr int MAX_ENTRIES = 16;
            ec_pdo_entry_reg_t domainRegs[MAX_ENTRIES + 1] = {};
            int idx = 0;

            uint16_t alias = driveCfg_.alias;
            uint16_t position = driveCfg_.position;
            uint32_t vendorId = driveCfg_.vendor_id;
            uint32_t productCode = driveCfg_.product_code;

            // Revisit the same pdo mappings to register offsets
            for (int smIndex = 0; smIndex < driveCfg_.syncManagerCount; smIndex++)
            {
                const auto &sm = driveCfg_.syncManagers[smIndex];
                for (int mgIndex = 0; mgIndex < sm.mappingGroupCount; mgIndex++)
                {
                    const auto &mg = sm.mappingGroups[mgIndex];
                    for (int eIndex = 0; eIndex < mg.entryCount; eIndex++)
                    {
                        if (idx >= MAX_ENTRIES)
                        {
                            // std::cerr << "ServoDrive: Too many PDO entries. Increase MAX_ENTRIES.\n";
                            log_warn( loggerMem_, "ServoDrive", 3301, "Too many PDO entries. Increase MAX_ENTRIES");
                            return false;
                        }
                        const auto &pme = mg.entries[eIndex];
                        uint16_t objIndex = hexStringToUint(pme.object_index.c_str());
                        uint8_t subIndex  = static_cast<uint8_t>(pme.subindex);

                        void *offsetPtr = getOffsetPointerByIndex(objIndex);
                        if (!offsetPtr)
                        {
                            // std::cerr << "[ServoDrive] Unsupported object_index=0x"
                            //           << std::hex << objIndex << std::dec << "\n";
                            log_error( loggerMem_, "ServoDrive", 3405, "Unsupported object_index");
                            return false;
                        }

                        domainRegs[idx].alias        = alias;
                        domainRegs[idx].position     = position;
                        domainRegs[idx].vendor_id    = vendorId;
                        domainRegs[idx].product_code = productCode;
                        domainRegs[idx].index        = objIndex;
                        domainRegs[idx].subindex     = subIndex;
                        domainRegs[idx].offset       = static_cast<unsigned int *>(offsetPtr);
                        domainRegs[idx].bit_position = 0;
                        idx++;
                    }
                }
            }

            domainRegs[idx] = {}; // terminator

            if (ecrt_domain_reg_pdo_entry_list(domain_, domainRegs))
            {
                // std::cerr << "ServoDrive: PDO entry registration failed.\n";
                if (loggerMem_) 
                {
                    log_warn(loggerMem_, "ServoDrive", 3302, "PDO entry registration failed.");
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

            // Read from EtherCAT domain memory -> servoTx_
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


            // Copy to rtLayout => servoBuffer.tx
            if (rtLayout_)
            {
                int frontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
                auto &txPdo  = rtLayout_->servoBuffer.buffer[frontIdx].tx[driveIndex_];

                txPdo.ctrl.statusWord          = servoTx_.ctrl.statusWord;
                txPdo.motion.positionActual    = servoTx_.motion.positionActual;
                txPdo.motion.velocityActual    = servoTx_.motion.velocityActual;
                txPdo.motion.torqueActual      = servoTx_.motion.torqueActual;
                txPdo.io.digitalInputs    = servoTx_.io.digitalInputs;
                txPdo.io.analogInput      = servoTx_.io.analogInput;
            }
            return true;
        }

        bool ServoDrive::writeOutputs(uint8_t* domainPd)
        {
            if (!domainPd)
            {
                return false;
            }

            // Read from rtLayout => servoBuffer.rx
            if (rtLayout_)
            {
                int frontIdx = rtLayout_->servoBuffer.frontIndex.load(std::memory_order_acquire);
                auto &rxPdo  = rtLayout_->servoBuffer.buffer[frontIdx].rx[driveIndex_];

                servoRx_.ctrl.controlWord       = rxPdo.ctrl.controlWord;
                servoRx_.motion.modeOfOperation   = rxPdo.motion.modeOfOperation;
                servoRx_.motion.targetTorque    = rxPdo.motion.targetTorque;
                servoRx_.motion.targetPosition  = rxPdo.motion.targetPosition;
            }

            // Write into EtherCAT domain memory
            EC_WRITE_U16(domainPd + servoOffsets_.controlword,
                         servoRx_.ctrl.controlWord);
            EC_WRITE_S8(domainPd + servoOffsets_.modes_of_operation,
                        servoRx_.motion.modeOfOperation);
            EC_WRITE_S16(domainPd + servoOffsets_.target_torque,
                         servoRx_.motion.targetTorque);
            EC_WRITE_S32(domainPd + servoOffsets_.target_position,
                         servoRx_.motion.targetPosition);
            return true;
        }

        void ServoDrive::handleState(const DriveUserSignals& /*signals*/)
        {
            // Additional logic for drive states if needed
        }

        uint16_t ServoDrive::hexStringToUint(const std::string& hexStr)
        {
            return static_cast<uint16_t>(std::strtoul(hexStr.c_str(), nullptr, 16));
        }

        void* ServoDrive::getOffsetPointerByIndex(uint16_t objectIndex)
        {
            // Associate object indices with servoOffsets_ fields
            // Inputs (Tx side):
            switch (objectIndex)
            {
            case 0x6041: return &servoOffsets_.statusword;           // statusWord
            case 0x6064: return &servoOffsets_.position_actual_value;
            case 0x606C: return &servoOffsets_.velocity_actual_value;
            case 0x6077: return &servoOffsets_.torque_actual_value;
            case 0x2600: return &servoOffsets_.digital_input_value;
            case 0x2081: return &servoOffsets_.analog_input_value;
            
            // Possibly 0x6061 => mode_of_operation_display

            // Outputs (Rx side):
            case 0x6040: return &servoOffsets_.controlword;
            case 0x6060: return &servoOffsets_.modes_of_operation;
            case 0x607A: return &servoOffsets_.target_position;
            case 0x6071: return &servoOffsets_.target_torque;

            default:
                return nullptr;
            }
        }

    } // namespace fieldbus
} // namespace hand_control
