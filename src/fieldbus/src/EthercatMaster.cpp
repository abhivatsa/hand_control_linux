#include "fieldbus/EthercatMaster.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <iostream>
#include <sched.h>
#include <stdexcept>
#include <sys/mman.h>
#include <thread>
#include <time.h>

#include "fieldbus/drives/ServoDrive.h"
#include "fieldbus/drives/IODrive.h"
#include "merai/RTIpc.h"

namespace fieldbus
{

    EthercatMaster::EthercatMaster(const std::string& paramServerShmName,
                                   std::size_t        paramServerShmSize,
                                   const std::string& rtDataShmName,
                                   std::size_t        rtDataShmSize,
                                   const std::string& loggerShmName,
                                   std::size_t        loggerShmSize)
        // ParameterServer read-only; RT + Logger read/write
        : configShm_(paramServerShmName, paramServerShmSize, /*readOnly=*/true),
          rtDataShm_(rtDataShmName,      rtDataShmSize,      /*readOnly=*/false),
          loggerShm_(loggerShmName,      loggerShmSize,      /*readOnly=*/false)
    {
        configPtr_ = static_cast<const merai::ParameterServer*>(configShm_.getPtr());
        if (!configPtr_)
        {
            throw std::runtime_error("EthercatMaster: failed to map ParameterServer memory.");
        }
        if (configPtr_->magic != merai::PARAM_SERVER_MAGIC ||
            configPtr_->version != merai::PARAM_SERVER_VERSION)
        {
            throw std::runtime_error("EthercatMaster: ParameterServer magic/version mismatch.");
        }

        rtLayout_ = static_cast<merai::RTMemoryLayout*>(rtDataShm_.getPtr());
        if (!rtLayout_)
        {
            throw std::runtime_error("EthercatMaster: failed to map RTMemoryLayout memory.");
        }
        if (!merai::validate_rt_layout(rtLayout_))
        {
            throw std::runtime_error("EthercatMaster: RTMemoryLayout integrity check failed.");
        }

        loggerMem_ = static_cast<merai::multi_ring_logger_memory*>(loggerShm_.getPtr());
        if (!loggerMem_)
        {
            throw std::runtime_error("EthercatMaster: failed to map Logger shared memory.");
        }

        merai::log_info(loggerMem_, "Fieldbus", 100, "EthercatMaster: Shared memories mapped");
    }

    EthercatMaster::~EthercatMaster()
    {
        if (master_)
        {
            ecrt_master_deactivate(master_);
            ecrt_release_master(master_);
            master_ = nullptr;
        }

        if (loggerMem_)
        {
            merai::log_info(loggerMem_, "Fieldbus", 999, "EthercatMaster destructor called");
        }
    }

    bool EthercatMaster::initializeMaster()
    {
        if (!configPtr_)
        {
            merai::log_error(loggerMem_, "Fieldbus", 101, "No valid ParameterServer pointer");
            return false;
        }

        if (configPtr_->driveCount <= 0)
        {
            merai::log_error(loggerMem_, "Fieldbus", 104, "No drives found in ParameterServer");
            return false;
        }

        // Clamp to what RT layout supports, just in case.
        const int driveCount =
            std::min<int>(configPtr_->driveCount, static_cast<int>(merai::MAX_SERVO_DRIVES));

        if (driveCount != configPtr_->driveCount)
        {
            merai::log_warn(loggerMem_, "Fieldbus", 105,
                            "driveCount exceeds MAX_SERVO_DRIVES; some drives will be ignored");
        }

        master_ = ecrt_request_master(0);
        if (!master_)
        {
            merai::log_error(loggerMem_, "Fieldbus", 102, "Failed to retrieve EtherCAT Master");
            return false;
        }

        domain_ = ecrt_master_create_domain(master_);
        if (!domain_)
        {
            merai::log_error(loggerMem_, "Fieldbus", 103, "Failed to create EtherCAT domain");
            return false;
        }

        // Create and configure each supported drive
        for (int i = 0; i < driveCount; ++i)
        {
            const merai::DriveConfig& driveCfg = configPtr_->drives[i];

            const std::uint16_t alias        = driveCfg.alias;
            const std::uint16_t position     = driveCfg.position;
            const std::uint32_t vendor_id    = driveCfg.vendor_id;
            const std::uint32_t product_code = driveCfg.product_code;

            ec_slave_config_t* sc = ecrt_master_slave_config(
                master_, alias, position, vendor_id, product_code);

            if (!sc)
            {
                merai::log_error(loggerMem_, "Fieldbus", 106,
                                 "Failed to get slave config for a drive");
                return false;
            }

            switch (driveCfg.type)
            {
            case merai::DriveType::Servo:
                drives_[i] = std::make_unique<ServoDrive>(
                    driveCfg,
                    sc,
                    domain_,
                    loggerMem_);
                break;

            case merai::DriveType::Io:
                drives_[i] = std::make_unique<IoDrive>(
                    driveCfg,
                    sc,
                    domain_,
                    loggerMem_);
                break;

            default:
                merai::log_error(loggerMem_, "Fieldbus", 107, "Unknown drive type in config");
                return false;
            }
        }

        // Configure PDOs for all valid drives
        for (int i = 0; i < driveCount; ++i)
        {
            auto& drive = drives_[i];
            if (drive)
            {
                drive->initialize();

                if (!drive->configurePdos())
                {
                    merai::log_error(loggerMem_, "Fieldbus", 108, "Failed to configure PDOs");
                    return false;
                }
            }
        }

        merai::log_info(loggerMem_, "Fieldbus", 109, "EthercatMaster initialization complete");
        return true;
    }

    void EthercatMaster::run()
    {
        if (!master_ || !domain_)
        {
            merai::log_error(loggerMem_, "Fieldbus", 114,
                             "run() called before initializeMaster()");
            return;
        }

        if (ecrt_master_activate(master_))
        {
            merai::log_error(loggerMem_, "Fieldbus", 110, "Failed to activate EtherCAT master");
            return;
        }

        domainPd_ = ecrt_domain_data(domain_);
        if (!domainPd_)
        {
            merai::log_error(loggerMem_, "Fieldbus", 111,
                             "Failed to get domain data pointer");
            return;
        }

        running_ = true;
        merai::log_info(loggerMem_, "Fieldbus", 112,
                        "Entering EthercatMaster cyclicTask");
        cyclicTask();
        merai::log_info(loggerMem_, "Fieldbus", 113,
                        "Exiting EthercatMaster cyclicTask");
    }

    void EthercatMaster::stop()
    {
        running_ = false;
        merai::log_warn(loggerMem_, "Fieldbus", 200,
                        "EthercatMaster stop() called, stopping RT loop");
    }

    void EthercatMaster::cyclicTask()
    {
        assert(rtLayout_);
        period_info pinfo{};
        periodic_task_init(&pinfo, loopPeriodNs);

        const int driveCount =
            (configPtr_)
                ? std::min<int>(configPtr_->driveCount, static_cast<int>(merai::MAX_SERVO_DRIVES))
                : 0;

        while (running_)
        {
            // PHASE 1: Determine buffer index for this cycle (fieldbus owns servoTxBuffer)
            const int servoTxBackIdx = merai::back_index(rtLayout_->servoTxBuffer);
            auto&     txBuf          = rtLayout_->servoTxBuffer.buffer[servoTxBackIdx];

            FieldbusCycleOutputs cycleOutputs{
                txBuf.data(),
                static_cast<std::size_t>(driveCount)};

            // PHASE 2: Snapshot servoRxBuffer (commands from Control)
            merai::read_snapshot(rtLayout_->servoRxBuffer, servoRxShadow_);

            // Clear any commands beyond active driveCount
            for (int i = driveCount; i < static_cast<int>(merai::MAX_SERVO_DRIVES); ++i)
            {
                servoRxShadow_[i] = merai::ServoRxPdo{};
            }

            FieldbusCycleInputs cycleInputs{
                servoRxShadow_.data(),
                static_cast<std::size_t>(driveCount)};

            // PHASE 3: Receive latest EtherCAT data
            if (ecrt_master_receive(master_) < 0)
            {
                merai::log_error(loggerMem_, "Fieldbus", 210,
                                 "ecrt_master_receive failed");
                break;
            }

            ecrt_domain_process(domain_);

            if (!domainPd_)
            {
                merai::log_error(loggerMem_, "Fieldbus", 211,
                                 "domainPd_ is null in cyclicTask");
                break;
            }

            // PHASE 4: Read inputs from all drives (EtherCAT -> local Tx PDOs -> shared memory)
            for (int i = 0; i < driveCount; ++i)
            {
                auto& drive = drives_[i];
                if (!drive)
                {
                    continue;
                }

                drive->readInputs(domainPd_);

                if (cycleOutputs.txPdos &&
                    configPtr_->drives[i].type == merai::DriveType::Servo)
                {
                    auto* servo = static_cast<ServoDrive*>(drive.get());
                    cycleOutputs.txPdos[i] = servo->txPdo();
                }
            }

            (void)checkDomainState();
            (void)checkMasterState();

            // PHASE 5: Clear unused Tx slots to avoid stale data
            for (int i = driveCount; i < static_cast<int>(merai::MAX_SERVO_DRIVES); ++i)
            {
                txBuf[i] = merai::ServoTxPdo{};
            }

            // Publish the freshly written servo Tx data for consumers (control, etc.)
            merai::publish(rtLayout_->servoTxBuffer, servoTxBackIdx);

            // PHASE 6: Write outputs for all drives (commands -> EtherCAT)
            for (int i = 0; i < driveCount; ++i)
            {
                auto& drive = drives_[i];
                if (!drive)
                {
                    continue;
                }

                const auto& rxPdo = cycleInputs.rxPdos[i];

                if (configPtr_->drives[i].type == merai::DriveType::Servo)
                {
                    auto* servo = static_cast<ServoDrive*>(drive.get());
                    servo->writeOutputs(domainPd_, rxPdo);
                }
                else
                {
                    // IoDrive or other: uses its own writeOutputs variant
                    drive->writeOutputs(domainPd_);
                }
            }

            // Queue the domain
            if (ecrt_domain_queue(domain_) < 0)
            {
                merai::log_error(loggerMem_, "Fieldbus", 212,
                                 "ecrt_domain_queue failed");
                break;
            }

            // Send master data
            if (ecrt_master_send(master_) < 0)
            {
                merai::log_error(loggerMem_, "Fieldbus", 213,
                                 "ecrt_master_send failed");
                break;
            }

            wait_rest_of_period(&pinfo);
        }
    }

    bool EthercatMaster::checkDomainState()
    {
        ec_domain_state_t ds{};
        ecrt_domain_state(domain_, &ds);
        domainState_ = ds;
        // You can add state-change logging later if needed.
        return true;
    }

    bool EthercatMaster::checkMasterState()
    {
        ec_master_state_t ms{};
        ecrt_master_state(master_, &ms);
        masterState_ = ms;
        return true;
    }

    void EthercatMaster::inc_period(period_info* pinfo)
    {
        pinfo->next_period.tv_nsec += pinfo->period_ns;
        while (pinfo->next_period.tv_nsec >= 1000000000L)
        {
            pinfo->next_period.tv_sec++;
            pinfo->next_period.tv_nsec -= 1000000000L;
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
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                        &pinfo->next_period, nullptr);
    }

} // namespace fieldbus
