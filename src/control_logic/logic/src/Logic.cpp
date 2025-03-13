#include "logic/Logic.h"
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include "merai/Enums.h"  // for OrchestratorState, ControllerID, etc.

namespace hand_control
{
    namespace logic
    {
        // ----------------------------
        // Constructor / Destructor
        // ----------------------------
        Logic::Logic(const std::string& paramServerShmName,
                     std::size_t paramServerShmSize,
                     const std::string& rtDataShmName,
                     std::size_t rtDataShmSize,
                     const std::string& loggerShmName,
                     std::size_t loggerShmSize)
            : paramServerShm_(paramServerShmName, paramServerShmSize, true),
              rtDataShm_(rtDataShmName, rtDataShmSize, false),
              loggerShm_(loggerShmName, loggerShmSize, false),
              // SafetyManager constructor (paramServerPtr_, rtLayout_, ... ) will happen in init()
              safetyManager_(nullptr, nullptr, /*someHapticModel*/)
        {
            // 1) param server
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer*>(
                                  paramServerShm_.getPtr());
            if (!paramServerPtr_)
                throw std::runtime_error("[Logic] Failed to map ParameterServer memory.");

            // 2) RTMemoryLayout
            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout*>(rtDataShm_.getPtr());
            if (!rtLayout_)
                throw std::runtime_error("[Logic] Failed to map RTMemoryLayout memory.");

            // 3) Logger
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory*>(
                             loggerShm_.getPtr());
            if (!loggerMem_)
                throw std::runtime_error("[Logic] Failed to map logger memory.");

            std::cout << "[Logic] Shared memory attached.\n";
        }

        Logic::~Logic()
        {
            // Cleanup if needed
        }

        // ----------------------------
        // init()
        // ----------------------------
        bool Logic::init()
        {
            // 1) Orchestrator init
            if (!systemOrchestrator_.init())
            {
                std::cerr << "[Logic] SystemOrchestrator init failed.\n";
                return false;
            }

            // 2) Load Haptic Device Model from paramServer
            //    - This is new, similar to what Control.cpp does
            if (!hapticDeviceModel_.loadFromParameterServer(*paramServerPtr_))
            {
                std::cerr << "[Logic] HapticDeviceModel load failed.\n";
                return false;
            }

            // 3) Reinitialize SafetyManager if it needs the model
            //    - Example: safetyManager_ constructor
            safetyManager_ = SafetyManager(paramServerPtr_, rtLayout_, hapticDeviceModel_);
            if (!safetyManager_.init())
            {
                std::cerr << "[Logic] SafetyManager init failed.\n";
                return false;
            }

            std::cout << "[Logic] init complete.\n";
            return true;
        }

        void Logic::run()
        {
            std::cout << "[Logic] Entering main loop.\n";
            cyclicTask();
        }

        void Logic::requestStop()
        {
            stopRequested_.store(true, std::memory_order_relaxed);
        }

        // -------------------------------------------------
        // cyclicTask and aggregator I/O remain unchanged
        // -------------------------------------------------

        void Logic::cyclicTask()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, 10'000'000L); // 10 ms cycle

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // 1) Read aggregator inputs (user commands, drive feedback, etc.)
                UserCommandsData userCmds;
                DriveFeedbackData driveFdbk;
                ControllerFeedbackData ctrlFdbk;
                readUserCommandsFromAggregator(userCmds);
                readDriveFeedbackAggregator(driveFdbk);
                readControllerFeedbackAggregator(ctrlFdbk);

                // 2) Run SafetyManager checks
                bool isFaulted = safetyManager_.update(driveFdbk, userCmds, ctrlFdbk);

                // 3) Orchestrator
                systemOrchestrator_.update(isFaulted, userCmds.desiredMode);

                // 4) Write aggregator outputs
                writeDriveControlSignalsAggregator();

                bool switchWanted = systemOrchestrator_.wantsControllerSwitch();
                auto ctrlID       = systemOrchestrator_.desiredControllerId();
                writeControllerSwitchAggregator(switchWanted, ctrlID);

                writeUserFeedbackAggregator(isFaulted, systemOrchestrator_.currentState(), userCmds);

                // 5) Check for shutdown
                if (userCmds.shutdownRequest)
                {
                    std::cout << "[Logic] Shutdown requested by user.\n";
                    break;
                }

                // 6) Sleep
                wait_rest_of_period(&pinfo);
            }

            std::cout << "[Logic] Exiting main loop.\n";
        }

        void Logic::readUserCommandsFromAggregator(UserCommandsData& out)
        {
            int frontIdx = rtLayout_->userCommandsBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &cmdSlot = rtLayout_->userCommandsBuffer.buffer[frontIdx];

            out.eStop           = cmdSlot.eStop;
            out.resetFault      = cmdSlot.resetFault;
            out.shutdownRequest = cmdSlot.shutdownRequest;
            out.desiredMode     = cmdSlot.desiredMode;
        }

        void Logic::readDriveFeedbackAggregator(DriveFeedbackData& out)
        {
            int frontIdx = rtLayout_->driveFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &dfbSlot = rtLayout_->driveFeedbackBuffer.buffer[frontIdx];
            // fill out 'out'
        }

        void Logic::readControllerFeedbackAggregator(ControllerFeedbackData& out)
        {
            int frontIdx = rtLayout_->controllerFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &ctrlFdbkSlot = rtLayout_->controllerFeedbackBuffer.buffer[frontIdx];
            // fill out 'out'
        }

        void Logic::writeDriveControlSignalsAggregator()
        {
            size_t driveCount = paramServerPtr_->driveCount;
            int frontIdx = rtLayout_->driveControlSignalsBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;

            auto &signalsData = rtLayout_->driveControlSignalsBuffer.buffer[backIdx];

            for (size_t i = 0; i < driveCount; ++i)
            {
                auto &sig = signalsData.signals[i];
                sig.faultReset     = false;
                sig.allowOperation = false;
                sig.quickStop      = false;
                sig.forceDisable   = false;

                if (systemOrchestrator_.shouldEnableDrive(i))
                    sig.allowOperation = true;
                if (systemOrchestrator_.shouldForceDisableDrive(i))
                    sig.forceDisable = true;
                if (systemOrchestrator_.shouldQuickStopDrive(i))
                    sig.quickStop = true;
                if (systemOrchestrator_.shouldFaultResetDrive(i))
                    sig.faultReset = true;
            }

            rtLayout_->driveControlSignalsBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Logic::writeControllerSwitchAggregator(bool switchWanted,
                                                    hand_control::merai::ControllerID ctrlId)
        {
            int frontIdx = rtLayout_->controllerCommandsAggBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;

            auto &ctrlAgg = rtLayout_->controllerCommandsAggBuffer.buffer[backIdx];
            ctrlAgg.requestSwitch    = switchWanted;
            ctrlAgg.targetController = ctrlId;

            rtLayout_->controllerCommandsAggBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Logic::writeUserFeedbackAggregator(bool isFaulted,
                                                merai::OrchestratorState currentState,
                                                const UserCommandsData& userCmds)
        {
            int ufFrontIdx = rtLayout_->userFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            int ufBackIdx  = 1 - ufFrontIdx;
            auto &ufbk     = rtLayout_->userFeedbackBuffer.buffer[ufBackIdx];

            ufbk.faultActive  = isFaulted;
            ufbk.currentState = currentState;
            ufbk.desiredMode  = userCmds.desiredMode;

            rtLayout_->userFeedbackBuffer.frontIndex.store(ufBackIdx, std::memory_order_release);
        }

        // -------------------------------------------------
        // Scheduling Helpers
        // -------------------------------------------------
        void Logic::periodic_task_init(period_info* pinfo, long periodNs)
        {
            clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
            pinfo->period_ns = periodNs;
        }

        void Logic::inc_period(period_info* pinfo)
        {
            pinfo->next_period.tv_nsec += pinfo->period_ns;
            if (pinfo->next_period.tv_nsec >= 1'000'000'000L)
            {
                pinfo->next_period.tv_nsec -= 1'000'000'000L;
                pinfo->next_period.tv_sec++;
            }
        }

        void Logic::wait_rest_of_period(period_info* pinfo)
        {
            inc_period(pinfo);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
        }

    } // namespace logic
} // namespace hand_control
