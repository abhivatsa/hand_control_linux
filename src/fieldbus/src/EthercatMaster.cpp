#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <sched.h>
#include <sys/mman.h>

#include "fieldbus/EthercatMaster.h"
#include "fieldbus/drives/ServoDrive.h"

namespace hand_control
{
    namespace fieldbus
    {
        EthercatMaster::EthercatMaster(const std::string& paramServerShmName,
                                       size_t            paramServerShmSize,
                                       const std::string& rtDataShmName,
                                       size_t            rtDataShmSize,
                                       const std::string& loggerShmName,
                                       size_t            loggerShmSize)
            // We open ParameterServer in read-only mode (true), RTMemory read/write (false), etc.
            : configShm_(paramServerShmName, paramServerShmSize, true),
              rtDataShm_(rtDataShmName, rtDataShmSize, false),
              loggerShm_(loggerShmName, loggerShmSize, false)
        {
            configPtr_ = reinterpret_cast<const hand_control::merai::ParameterServer*>(
                configShm_.getPtr()
            );
            if (!configPtr_)
            {
                throw std::runtime_error(
                    "EthercatMaster: failed to map ParameterServer memory."
                );
            }

            std::cout<<"paramServerShmName : "<<paramServerShmName<<", paramServerShmSize : "<<paramServerShmSize<<std::endl;

            rtLayout_ = reinterpret_cast<hand_control::merai::RTMemoryLayout*>(
                rtDataShm_.getPtr()
            );
            if (!rtLayout_)
            {
                throw std::runtime_error(
                    "EthercatMaster: failed to map RTMemoryLayout memory."
                );
            }

            loggerMem_ = reinterpret_cast<hand_control::merai::multi_ring_logger_memory*>(
                loggerShm_.getPtr()
            );
            if (!loggerMem_)
            {
                throw std::runtime_error(
                    "EthercatMaster: failed to map Logger shared memory."
                );
            }

            hand_control::merai::log_info(
                loggerMem_,
                "Fieldbus",
                100,
                "EthercatMaster: Shared memories mapped"
            );
        }

        EthercatMaster::~EthercatMaster()
        {
            if (master_)
            {
                ecrt_master_deactivate(master_);
                ecrt_release_master(master_);
                master_ = nullptr;
            }

            hand_control::merai::log_info(
                loggerMem_,
                "Fieldbus",
                999,
                "EthercatMaster destructor called"
            );
        }

        bool EthercatMaster::initializeMaster()
        {
            if (!configPtr_)
            {
                std::cerr << "[Error] No valid ParameterServer pointer.\n";
                hand_control::merai::log_error(
                    loggerMem_,
                    "Fieldbus",
                    101,
                    "No valid ParameterServer pointer"
                );
                return false;
            }

            // Retrieve the cycle period for fieldbus from ParameterServer
            loopPeriodNs = configPtr_->startup.fieldbusLoopNs;

            master_ = ecrt_request_master(0);
            if (!master_)
            {
                std::cerr << "[Error] Failed to retrieve EtherCAT Master.\n";
                hand_control::merai::log_error(
                    loggerMem_,
                    "Fieldbus",
                    102,
                    "Failed to retrieve EtherCAT Master"
                );
                return false;
            }

            domain_ = ecrt_master_create_domain(master_);
            if (!domain_)
            {
                std::cerr << "[Error] Failed to create EtherCAT domain.\n";
                hand_control::merai::log_error(
                    loggerMem_,
                    "Fieldbus",
                    103,
                    "Failed to create EtherCAT domain"
                );
                return false;
            }

            std::cout<<"configPtr_->driveCount : "<<configPtr_->driveCount<<std::endl;

            if (configPtr_->driveCount <= 0)
            {
                std::cerr << "[Error] No drives found in ParameterServer.\n";
                hand_control::merai::log_error(
                    loggerMem_,
                    "Fieldbus",
                    104,
                    "No drives found in ParameterServer"
                );
                return false;
            }

            int driveCount = configPtr_->driveCount;
            if (driveCount > hand_control::merai::MAX_SERVO_DRIVES)
            {
                hand_control::merai::log_warn(
                    loggerMem_,
                    "Fieldbus",
                    105,
                    "driveCount exceeds MAX_SERVO_DRIVES, ignoring extras"
                );
                driveCount = hand_control::merai::MAX_SERVO_DRIVES;
            }

            // Create and configure each drive
            for (int i = 0; i < driveCount; ++i)
            {
                const hand_control::merai::DriveConfig& driveCfg = configPtr_->drives[i];

                uint16_t alias        = static_cast<uint16_t>(driveCfg.alias);
                uint16_t position     = static_cast<uint16_t>(driveCfg.position);
                uint32_t vendor_id    = driveCfg.vendor_id;
                uint32_t product_code = driveCfg.product_code;

                ec_slave_config_t* sc = ecrt_master_slave_config(
                    master_, alias, position, vendor_id, product_code
                );
                if (!sc)
                {
                    std::cerr << "[Error] Failed to get slave config for alias=" << alias
                              << ", position=" << position
                              << ", vendor_id=0x" << std::hex << vendor_id
                              << ", product_code=0x" << product_code << std::dec << "\n";

                    hand_control::merai::log_error(
                        loggerMem_,
                        "Fieldbus",
                        106,
                        "Failed to get slave config for a drive"
                    );
                    return false;
                }

                std::string driveType = driveCfg.type.c_str();
                if (driveType == "servo")
                {
                    drives_[i] = std::make_unique<ServoDrive>(
                        driveCfg,
                        sc,
                        domain_,
                        rtLayout_,
                        i,
                        loggerMem_
                    );
                }
                else
                {
                    std::cerr << "[Error] Unknown drive type: " << driveType << "\n";
                    hand_control::merai::log_error(
                        loggerMem_,
                        "Fieldbus",
                        107,
                        ("Unknown drive type: " + driveType).c_str()
                    );
                    return false;
                }
            }

            int ctr = 0;

            // Configure PDOs for all drives
            for (auto& drive : drives_)
            {

                std::cout<<"drive_cnt : "<<ctr<<std::endl;
                
                if (drive)
                {
                    if (!drive->configurePdos())
                    {
                        std::cerr << "[Error] Failed to configure PDOs.\n";
                        hand_control::merai::log_error(
                            loggerMem_,
                            "Fieldbus",
                            108,
                            "Failed to configure PDOs"
                        );
                        return false;
                    }
                }
            }

            std::cout << "[Info] EthercatMaster: Initialization complete.\n";
            hand_control::merai::log_info(
                loggerMem_,
                "Fieldbus",
                109,
                "EthercatMaster initialization complete"
            );
            return true;
        }

        void EthercatMaster::run()
        {
            if (ecrt_master_activate(master_))
            {
                std::cerr << "[Error] activating EtherCAT master. Aborting.\n";
                hand_control::merai::log_error(
                    loggerMem_,
                    "Fieldbus",
                    110,
                    "Failed to activate EtherCAT master"
                );
                return;
            }

            domainPd_ = ecrt_domain_data(domain_);
            if (!domainPd_)
            {
                std::cerr << "[Error] Failed to get domain data pointer.\n";
                hand_control::merai::log_error(
                    loggerMem_,
                    "Fieldbus",
                    111,
                    "Failed to get domain data pointer"
                );
                return;
            }

            running_ = true;
            hand_control::merai::log_info(
                loggerMem_,
                "Fieldbus",
                112,
                "Entering EthercatMaster cyclicTask"
            );
            cyclicTask();
            hand_control::merai::log_info(
                loggerMem_,
                "Fieldbus",
                113,
                "Exiting EthercatMaster cyclicTask"
            );
        }

        void EthercatMaster::stop()
        {
            running_ = false;
            hand_control::merai::log_warn(
                loggerMem_,
                "Fieldbus",
                200,
                "EthercatMaster stop() called, stopping RT loop"
            );
        }

        void EthercatMaster::cyclicTask()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, loopPeriodNs);

            while (running_)
            {
                if (ecrt_master_receive(master_) < 0)
                {
                    std::cerr << "[ERROR] ecrt_master_receive() failed. Breaking.\n";
                    hand_control::merai::log_error(
                        loggerMem_,
                        "Fieldbus",
                        201,
                        "ecrt_master_receive() failed"
                    );
                    break;
                }
                ecrt_domain_process(domain_);

                if (!domainPd_)
                {
                    std::cerr << "[ERROR] domainPd_ invalid. Breaking.\n";
                    hand_control::merai::log_error(
                        loggerMem_,
                        "Fieldbus",
                        202,
                        "domainPd_ invalid in cyclicTask"
                    );
                    break;
                }

                // Read inputs from all drives
                for (auto& drive : drives_)
                {
                    if (drive)
                    {
                        drive->readInputs(domainPd_);
                    }
                }

                if (!checkDomainState())
                {
                    hand_control::merai::log_error(
                        loggerMem_,
                        "Fieldbus",
                        203,
                        "Domain check failed, stopping loop"
                    );
                    break;
                }
                if (!checkMasterState())
                {
                    hand_control::merai::log_error(
                        loggerMem_,
                        "Fieldbus",
                        204,
                        "Master check failed, stopping loop"
                    );
                    break;
                }

                // Write outputs for all drives
                for (auto& drive : drives_)
                {
                    if (drive)
                    {
                        drive->writeOutputs(domainPd_);
                    }
                }

                // Queue the domain
                if (ecrt_domain_queue(domain_) < 0)
                {
                    std::cerr << "[ERROR] ecrt_domain_queue() failed.\n";
                    hand_control::merai::log_error(
                        loggerMem_,
                        "Fieldbus",
                        205,
                        "ecrt_domain_queue() failed"
                    );
                    break;
                }
                // Send master data
                if (ecrt_master_send(master_) < 0)
                {
                    std::cerr << "[ERROR] ecrt_master_send() failed.\n";
                    hand_control::merai::log_error(
                        loggerMem_,
                        "Fieldbus",
                        206,
                        "ecrt_master_send() failed"
                    );
                    break;
                }

                wait_rest_of_period(&pinfo);
            }
        }

        bool EthercatMaster::checkDomainState()
        {
            ec_domain_state_t ds;
            ecrt_domain_state(domain_, &ds);

            if (ds.working_counter != domainState_.working_counter)
            {
                std::cout << "Domain: WC changed to " << ds.working_counter << "\n";
            }
            if (ds.wc_state != domainState_.wc_state)
            {
                std::cout << "Domain: State changed to " << (int)ds.wc_state << "\n";
            }
            domainState_ = ds;
            return true;  // Minimal approach
        }

        bool EthercatMaster::checkMasterState()
        {
            ec_master_state_t ms;
            ecrt_master_state(master_, &ms);

            if (ms.al_states != masterState_.al_states)
            {
                std::cout << "AL states changed to: 0x"
                          << std::hex << (int)ms.al_states << std::dec << "\n";
            }
            masterState_ = ms;
            return true;
        }

        void EthercatMaster::inc_period(period_info* pinfo)
        {
            pinfo->next_period.tv_nsec += pinfo->period_ns;
            while (pinfo->next_period.tv_nsec >= 1000000000)
            {
                pinfo->next_period.tv_sec++;
                pinfo->next_period.tv_nsec -= 1000000000;
            }
        }

        void EthercatMaster::periodic_task_init(period_info* pinfo, long period_ns)
        {
            pinfo->period_ns = period_ns;
            clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
        }

        void EthercatMaster::wait_rest_of_period(period_info* pinfo)
        {
            inc_period(pinfo);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
        }
    } // namespace fieldbus
} // namespace hand_control
