#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

#include "control/Control.h"
#include "control/hardware_abstraction/MockHardwareAbstractionLayer.h"
#include "control/hardware_abstraction/RealHardwareAbstractionLayer.h"

// Example: If you have a GravityCompController:
#include "control/controllers/GravityCompController.h"

// Include your new Enums.h if needed explicitly
#include "merai/Enums.h"

namespace hand_control
{
    namespace control
    {
        // -------------------------------------------------------------------
        // Constructor
        // -------------------------------------------------------------------
        Control::Control(const std::string &paramServerShmName,
                         size_t paramServerShmSize,
                         const std::string &rtDataShmName,
                         size_t rtDataShmSize,
                         const std::string &loggerShmName,
                         size_t loggerShmSize)
            : paramServerShm_(paramServerShmName, paramServerShmSize, true),
              rtDataShm_(rtDataShmName, rtDataShmSize, false),
              loggerShm_(loggerShmName, loggerShmSize, false)
        {
            // 1) Attach & map ParameterServer
            paramServerPtr_ =
                reinterpret_cast<const hand_control::merai::ParameterServer*>(
                    paramServerShm_.getPtr());
            if (!paramServerPtr_)
            {
                throw std::runtime_error("[Control] Failed to map ParameterServer memory.");
            }

            // 2) Attach & map RTMemoryLayout
            rtLayout_ =
                reinterpret_cast<hand_control::merai::RTMemoryLayout*>(
                    rtDataShm_.getPtr());
            if (!rtLayout_)
            {
                throw std::runtime_error("[Control] Failed to map RTMemoryLayout memory.");
            }

            // 3) Attach & map Logger memory
            loggerMem_ =
                reinterpret_cast<hand_control::merai::multi_ring_logger_memory*>(
                    loggerShm_.getPtr());
            if (!loggerMem_)
            {
                throw std::runtime_error("[Control] Failed to map multi_ring_logger_memory.");
            }

            // Create HAL (mock or real)
            if (paramServerPtr_->startup.simulateMode)
            {
                std::cout << "[Control] Creating MockHardwareAbstractionLayer.\n";
                hal_ = std::make_unique<MockHardwareAbstractionLayer>(
                           rtLayout_, paramServerPtr_, loggerMem_);
            }
            else
            {
                std::cout << "[Control] Creating RealHardwareAbstractionLayer.\n";
                hal_ = std::make_unique<RealHardwareAbstractionLayer>(
                           rtLayout_, paramServerPtr_, loggerMem_);
            }

            // Create Managers
            manager_ = std::make_unique<ControllerManager>(paramServerPtr_);
            driveStateManager_ = std::make_unique<DriveStateManager>();
        }

        // -------------------------------------------------------------------
        // Destructor
        // -------------------------------------------------------------------
        Control::~Control()
        {
            // Cleanup if needed
        }

        // -------------------------------------------------------------------
        // init()
        // -------------------------------------------------------------------
        bool Control::init()
        {
            // 1) Build a 6-axis model from ParameterServer
            {
                std::cout << "[Control] Building 6-axis model from paramServer.\n";
                if (!hapticDeviceModel_.loadFromParameterServer(*paramServerPtr_))
                {
                    std::cerr << "[Control] loadFromParameterServer failed: not enough links/joints.\n";
                    return false;
                }
            }

            // 2) Init HAL
            if (!hal_->init())
            {
                std::cerr << "[Control] HAL init failed.\n";
                return false;
            }

            // 3) Register example controllers
            {
                auto gravityComp = std::make_shared<GravityCompController>(hapticDeviceModel_);
                if (!manager_->registerController(gravityComp))
                {
                    std::cerr << "[Control] Failed to register GravityCompController.\n";
                    return false;
                }
            }

            // 4) Init the manager
            if (!manager_->init())
            {
                std::cerr << "[Control] ControllerManager init failed.\n";
                return false;
            }

            std::cout << "[Control] init successful.\n";
            return true;
        }

        // -------------------------------------------------------------------
        // run()
        // -------------------------------------------------------------------
        void Control::run()
        {
            cyclicTask();
        }

        // -------------------------------------------------------------------
        // requestStop()
        // -------------------------------------------------------------------
        void Control::requestStop()
        {
            stopRequested_.store(true, std::memory_order_relaxed);
        }

        // -------------------------------------------------------------------
        // cyclicTask() - real-time loop (1 ms)
        // -------------------------------------------------------------------
        void Control::cyclicTask()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, 1'000'000L); // 1 ms = 1,000,000 ns

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // (1) Read from hardware
                hal_->read();

                // (2) Copy states -> shared memory
                copyJointStatesToSharedMemory();
                copyIoStatesToSharedMemory();

                // (3) Read aggregator commands from logic
                auto driveCmdEnum    = readDriveCommandAggregator();
                auto ctrlCmdAgg      = readControllerCommandAggregator();

                // (4) Apply aggregator commands
                applyDriveCommand(driveCmdEnum);
                applyControllerSwitch(ctrlCmdAgg);

                // (5) Sub-managers
                updateDriveStateManager();
                updateControllerManager();

                // (6) Copy updated commands -> hardware
                copyJointCommandsFromSharedMemory();
                copyIoCommandsFromSharedMemory();
                hal_->write();

                // (7) Possibly write aggregator feedback for logic (driveSummaryBuffer)
                writeDriveSummaryAggregator();

                // (8) Sleep until next cycle
                wait_rest_of_period(&pinfo);
            }

            // optionally disable drives upon exit
        }

        // -------------------------------------------------------------------
        // Scheduling Helpers
        // -------------------------------------------------------------------
        void Control::periodic_task_init(period_info *pinfo, long periodNs)
        {
            clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
            pinfo->period_ns = periodNs;
        }

        void Control::inc_period(period_info *pinfo)
        {
            pinfo->next_period.tv_nsec += pinfo->period_ns;
            if (pinfo->next_period.tv_nsec >= 1'000'000'000L)
            {
                pinfo->next_period.tv_nsec -= 1'000'000'000L;
                pinfo->next_period.tv_sec++;
            }
        }

        void Control::wait_rest_of_period(period_info *pinfo)
        {
            inc_period(pinfo);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
        }

        // -------------------------------------------------------------------
        // Copy Joint & I/O States to Shared Memory
        // -------------------------------------------------------------------
        void Control::copyJointStatesToSharedMemory()
        {
            int currentFront = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            int backIndex = 1 - currentFront;

            auto &backStates = rtLayout_->jointBuffer.buffer[backIndex].states;
            int jointCount = static_cast<int>(hal_->getJointCount());

            auto *halStates = hal_->getJointStatesPtr();
            for (int i = 0; i < jointCount; i++)
            {
                backStates[i].position = halStates[i].position;
                backStates[i].velocity = halStates[i].velocity;
                backStates[i].torque   = halStates[i].torque;
            }

            rtLayout_->jointBuffer.frontIndex.store(backIndex, std::memory_order_release);
        }

        void Control::copyIoStatesToSharedMemory()
        {
            int currentFront = rtLayout_->ioDataBuffer.frontIndex.load(std::memory_order_acquire);
            int backIndex = 1 - currentFront;

            auto &backIoStates = rtLayout_->ioDataBuffer.buffer[backIndex].states;
            size_t ioCount = hal_->getIoCount();
            auto *halIoStates = hal_->getIoStatesPtr();

            if (!halIoStates || ioCount == 0)
            {
                return;
            }

            for (size_t i = 0; i < ioCount; i++)
            {
                for (int ch = 0; ch < 8; ++ch)
                {
                    backIoStates[i].digitalInputs[ch] = halIoStates[i].digitalInputs[ch];
                }
                for (int ch = 0; ch < 2; ++ch)
                {
                    backIoStates[i].analogInputs[ch] = halIoStates[i].analogInputs[ch];
                }
            }

            rtLayout_->ioDataBuffer.frontIndex.store(backIndex, std::memory_order_release);
        }

        void Control::checkSafetyFlags()
        {
            // Possibly read from a separate aggregator or structure
            // and if something is triggered, handle it
        }

        // -------------------------------------------------------------------
        // Copy Commands from Shared Memory -> Hardware
        // -------------------------------------------------------------------
        void Control::copyJointCommandsFromSharedMemory()
        {
            int frontIndex = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &cmds = rtLayout_->jointBuffer.buffer[frontIndex].commands;

            int jointCount = static_cast<int>(hal_->getJointCount());
            auto *halCommands = hal_->getJointCommandsPtr();

            for (int i = 0; i < jointCount; i++)
            {
                halCommands[i].position = cmds[i].position;
                halCommands[i].velocity = cmds[i].velocity;
                halCommands[i].torque   = cmds[i].torque;
            }
        }

        void Control::copyIoCommandsFromSharedMemory()
        {
            int frontIndex = rtLayout_->ioDataBuffer.frontIndex.load(std::memory_order_acquire);
            auto &ioCmds = rtLayout_->ioDataBuffer.buffer[frontIndex].commands;

            size_t ioCount = hal_->getIoCount();
            auto *halIoCommands = hal_->getIoCommandsPtr();

            if (!halIoCommands || ioCount == 0)
            {
                return;
            }

            for (size_t i = 0; i < ioCount; i++)
            {
                for (int ch = 0; ch < 8; ++ch)
                {
                    halIoCommands[i].digitalOutputs[ch] = ioCmds[i].digitalOutputs[ch];
                }
                for (int ch = 0; ch < 2; ++ch)
                {
                    halIoCommands[i].analogOutputs[ch] = ioCmds[i].analogOutputs[ch];
                }
            }
        }

        // -------------------------------------------------------------------
        // Aggregator Helpers
        // -------------------------------------------------------------------
        hand_control::merai::DriveCommand Control::readDriveCommandAggregator()
        {
            // Read the aggregator buffer
            int frontIdx = rtLayout_->driveCommandBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &cmdAgg = rtLayout_->driveCommandBuffer.buffer[frontIdx];
            // Now cmdAgg.driveCommand is already an enum in the updated RTMemoryLayout
            return cmdAgg.driveCommand;
        }

        Control::ControllerCommandAggregated Control::readControllerCommandAggregator()
        {
            // NOTE: If your RTMemoryLayout's ControllerCommandAggregated uses an enum for targetController,
            // the code below changes accordingly. The snippet below is for the "fully enum" approach.

            ControllerCommandAggregated local{};
            int frontIdx = rtLayout_->controllerCommandsAggBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &ctrlAgg = rtLayout_->controllerCommandsAggBuffer.buffer[frontIdx];

            local.requestSwitch     = ctrlAgg.requestSwitch;
            local.targetController  = ctrlAgg.targetController;
            return local;
        }

        void Control::writeDriveSummaryAggregator()
        {
            bool anyFault = driveStateManager_->anyFaulted();
            int faultSev  = driveStateManager_->faultSeverity();

            int frontIdx  = rtLayout_->driveSummaryBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx   = 1 - frontIdx;
            auto &summary = rtLayout_->driveSummaryBuffer.buffer[backIdx];
            summary.anyFaulted    = anyFault;
            summary.faultSeverity = faultSev;

            rtLayout_->driveSummaryBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Control::applyDriveCommand(hand_control::merai::DriveCommand cmd)
        {
            // interpret the enumerated command => produce local user signals
            std::array<DriveUserSignals, MAX_SERVO_DRIVES> localSignals{};

            switch(cmd)
            {
                case hand_control::merai::DriveCommand::ENABLE_ALL:
                    for (auto &sig : localSignals)
                    {
                        sig.allowOperation = true;
                    }
                    break;

                case hand_control::merai::DriveCommand::DISABLE_ALL:
                    for (auto &sig : localSignals)
                    {
                        sig.forceDisable = true;
                    }
                    break;

                case hand_control::merai::DriveCommand::QUICK_STOP:
                    for (auto &sig : localSignals)
                    {
                        sig.quickStop = true;
                    }
                    break;

                case hand_control::merai::DriveCommand::FAULT_RESET:
                    for (auto &sig : localSignals)
                    {
                        sig.faultReset = true;
                    }
                    break;

                default: // NONE or unknown => do nothing
                    break;
            }

            // Now store localSignals into driveUserSignalsBuffer
            int frontIdx = rtLayout_->driveUserSignalsBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;
            auto &sigBuf = rtLayout_->driveUserSignalsBuffer.buffer[backIdx];

            for (int i = 0; i < MAX_SERVO_DRIVES; i++)
            {
                sigBuf.signals[i] = localSignals[i];
            }
            rtLayout_->driveUserSignalsBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Control::applyControllerSwitch(const ControllerCommandAggregated &ctrlCmd)
        {
            if (!ctrlCmd.requestSwitch)
                return;

            // We want to push this into the controllerUserCmdBuffer
            int cFrontIdx  = rtLayout_->controllerUserCmdBuffer.frontIndex.load(std::memory_order_acquire);
            int cBackIdx   = 1 - cFrontIdx;
            auto &ctrlBuf  = rtLayout_->controllerUserCmdBuffer.buffer[cBackIdx];

            // Indicate we want a switch
            ctrlBuf.commands[0].requestSwitch = true;

            // Instead of copying a string, just store the enum:
            ctrlBuf.commands[0].controllerId = ctrlCmd.targetController;

            rtLayout_->controllerUserCmdBuffer.frontIndex.store(cBackIdx, std::memory_order_release);
        }

        void Control::updateDriveStateManager()
        {
            // Read current signals
            int sigFrontIdx =
                rtLayout_->driveUserSignalsBuffer.frontIndex.load(std::memory_order_acquire);
            auto &sigBuf = rtLayout_->driveUserSignalsBuffer.buffer[sigFrontIdx];

            // Prepare feedback
            int fdbkBackIdx =
                1 - rtLayout_->driveFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            auto &fdbkBuf = rtLayout_->driveFeedbackBuffer.buffer[fdbkBackIdx];

            driveStateManager_->update(
                hal_->getDriveInputsPtr(),
                hal_->getDriveOutputsPtr(),
                sigBuf.signals.data(),
                fdbkBuf.feedback.data(),
                hal_->getDriveCount()
            );

            // Publish new feedback index
            rtLayout_->driveFeedbackBuffer.frontIndex.store(
                fdbkBackIdx, std::memory_order_release);
        }

        void Control::updateControllerManager()
        {
            // Get the user commands buffer
            int cmdFrontIdx =
                rtLayout_->controllerUserCmdBuffer.frontIndex.load(std::memory_order_acquire);
            auto &ctrlCmdBuf = rtLayout_->controllerUserCmdBuffer.buffer[cmdFrontIdx];

            // Prepare feedback
            int ctrlFdbkBackIdx =
                1 - rtLayout_->controllerFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            auto &ctrlFdbkBuf = rtLayout_->controllerFeedbackBuffer.buffer[ctrlFdbkBackIdx];

            manager_->update(
                hal_->getJointStatesPtr(),
                hal_->getJointCommandsPtr(),
                ctrlCmdBuf.commands.data(),
                ctrlFdbkBuf.feedback.data(),
                static_cast<int>(hal_->getJointCount()),
                0.001 // dt in seconds
            );

            // Publish new feedback index
            rtLayout_->controllerFeedbackBuffer.frontIndex.store(
                ctrlFdbkBackIdx, std::memory_order_release);
        }

    } // namespace control
} // namespace hand_control
