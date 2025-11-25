#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>
#include <sched.h>
#include <sys/mman.h>

#include "fieldbus/EthercatMaster.h"
#include "fieldbus/drives/ServoDrive.h"
#include "fieldbus/drives/IODrive.h"

namespace seven_axis_robot
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
            configPtr_ = reinterpret_cast<const seven_axis_robot::merai::ParameterServer*>(configShm_.getPtr());
            if (!configPtr_)
            {
                throw std::runtime_error("EthercatMaster: failed to map ParameterServer memory.");
            }

            rtLayout_ = reinterpret_cast<seven_axis_robot::merai::RTMemoryLayout*>(rtDataShm_.getPtr());
            if (!rtLayout_)
            {
                throw std::runtime_error("EthercatMaster: failed to map RTMemoryLayout memory.");
            }

            loggerMem_ = reinterpret_cast<seven_axis_robot::merai::multi_ring_logger_memory*>(loggerShm_.getPtr());
            if (!loggerMem_)
            {
                throw std::runtime_error("EthercatMaster: failed to map Logger shared memory.");
            }

            seven_axis_robot::merai::log_info(loggerMem_, "Fieldbus", 100, "EthercatMaster: Shared memories mapped");
        }

        EthercatMaster::~EthercatMaster()
        {
            if (master_)
            {
                ecrt_master_deactivate(master_);
                ecrt_release_master(master_);
                master_ = nullptr;
            }

            log_info( loggerMem_, "Fieldbus", 999, "EthercatMaster destructor called" );
        }

        bool EthercatMaster::initializeMaster()
        {
            if (!configPtr_)
            {
                seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 101, "No valid ParameterServer pointer" );
                return false;
            }

            // loopPeriodNs defaults to 1 ms (configurable in future if needed).

            master_ = ecrt_request_master(0);
            if (!master_)
            {
                seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 102, "Failed to retrieve EtherCAT Master" );
                return false;
            }

            domain_ = ecrt_master_create_domain(master_);
            if (!domain_)
            {
                seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 103, "Failed to create EtherCAT domain");
                return false;
            }

            if (configPtr_->driveCount <= 0)
            {
                seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 104, "No drives found in ParameterServer");
                return false;
            }

            int driveCount = configPtr_->driveCount;

            // Create and configure each drive
            for (int i = 0; i < driveCount; ++i)
            {
                const seven_axis_robot::merai::DriveConfig& driveCfg = configPtr_->drives[i];

                uint16_t alias        = static_cast<uint16_t>(driveCfg.alias);
                uint16_t position     = static_cast<uint16_t>(driveCfg.position);
                uint32_t vendor_id    = driveCfg.vendor_id;
                uint32_t product_code = driveCfg.product_code;

                ec_slave_config_t* sc = ecrt_master_slave_config(
                    master_, alias, position, vendor_id, product_code
                );
                if (!sc)
                {
                    seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 106, "Failed to get slave config for a drive" );
                    return false;
                }

                switch (driveCfg.type)
                {
                    case seven_axis_robot::merai::DriveType::Servo:
                        drives_[i] = std::make_unique<ServoDrive>(
                            driveCfg,
                            sc,
                            domain_,
                            rtLayout_,
                            i,
                            loggerMem_
                        );
                        break;
                    case seven_axis_robot::merai::DriveType::Io:
                        drives_[i] = std::make_unique<IoDrive>(
                            driveCfg,
                            sc,
                            domain_,
                            rtLayout_,
                            i,
                            loggerMem_
                        );
                        break;
                    default:
                        seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 107, "Unknown drive type in config");
                        return false;
                }
            }

            // Configure PDOs for all drives
            for (auto& drive : drives_)
            {
                if (drive)
                {
                    drive->initialize();

                    if (!drive->configurePdos())
                    {
                        seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 108, "Failed to configure PDOs" );
                        return false;
                    }
                }
            }

            seven_axis_robot::merai::log_info( loggerMem_, "Fieldbus", 109, "EthercatMaster initialization complete" );
            return true;
        }

        void EthercatMaster::run()
        {
            if (ecrt_master_activate(master_))
            {
                seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 110, "Failed to activate EtherCAT master" );
                return;
            }

            domainPd_ = ecrt_domain_data(domain_);
            if (!domainPd_)
            {
                seven_axis_robot::merai::log_error( loggerMem_, "Fieldbus", 111, "Failed to get domain data pointer" );
                return;
            }

            running_ = true;
            seven_axis_robot::merai::log_info( loggerMem_, "Fieldbus", 112, "Entering EthercatMaster cyclicTask");
            cyclicTask();
            seven_axis_robot::merai::log_info( loggerMem_, "Fieldbus", 113, "Exiting EthercatMaster cyclicTask");
        }

        void EthercatMaster::stop()
        {
            running_ = false;
            seven_axis_robot::merai::log_warn( loggerMem_, "Fieldbus", 200, "EthercatMaster stop() called, stopping RT loop");
        }

        void EthercatMaster::cyclicTask()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, loopPeriodNs);

            while (running_)
            {
                if (ecrt_master_receive(master_) < 0)
                {
                    break;
                }
                ecrt_domain_process(domain_);

                if (!domainPd_)
                {
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
                    break;
                }
                if (!checkMasterState())
                {
                    break;
                }

                // Write outputs for all drives
                // int cnt = 0;
                for (auto& drive : drives_)
                {
                    if (drive)
                    {
                        // std::cout<<"cnt : "<<cnt<<std::endl;
                        drive->writeOutputs(domainPd_);
                        // cnt++;
                    }
                }

                // Queue the domain
                if (ecrt_domain_queue(domain_) < 0)
                {
                    break;
                }
                // Send master data
                if (ecrt_master_send(master_) < 0)
                {
                    break;
                }

                wait_rest_of_period(&pinfo);
            }
        }

        bool EthercatMaster::checkDomainState()
        {
            ec_domain_state_t ds;
            ecrt_domain_state(domain_, &ds);

            domainState_ = ds;
            return true;  // Minimal approach
        }

        bool EthercatMaster::checkMasterState()
        {
            ec_master_state_t ms;
            ecrt_master_state(master_, &ms);

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
} // namespace seven_axis_robot
