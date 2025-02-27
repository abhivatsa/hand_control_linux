#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

#include "control/Control.h"
#include "control/hardware_abstraction/MockHardwareAbstractionLayer.h"
#include "control/hardware_abstraction/RealHardwareAbstractionLayer.h"

// Example controllers
#include "control/controllers/GravityCompController.h"

namespace hand_control
{
    namespace control
    {

        // --------------------------------------------------------------------------------
        // Constructor
        // --------------------------------------------------------------------------------
        Control::Control(const std::string &paramServerShmName, size_t paramServerShmSize,
                         const std::string &rtDataShmName, size_t rtDataShmSize,
                         const std::string &loggerShmName, size_t loggerShmSize)
            : paramServerShm_(paramServerShmName, paramServerShmSize, /*readOnly=*/true), rtDataShm_(rtDataShmName, rtDataShmSize, /*readOnly=*/false), loggerShm_(loggerShmName, loggerShmSize, /*readOnly=*/false)
        {
            // 1) Attach & map ParameterServer
            paramServerPtr_ =
                reinterpret_cast<const hand_control::merai::ParameterServer *>(paramServerShm_.getPtr());
            if (!paramServerPtr_)
            {
                throw std::runtime_error("[Control] Failed to map ParameterServer memory.");
            }

            // 2) Attach & map RTMemoryLayout
            rtLayout_ =
                reinterpret_cast<hand_control::merai::RTMemoryLayout *>(rtDataShm_.getPtr());
            if (!rtLayout_)
            {
                throw std::runtime_error("[Control] Failed to map RTMemoryLayout memory.");
            }

            // 3) Attach & map Logger memory
            loggerMem_ =
                reinterpret_cast<hand_control::merai::multi_ring_logger_memory *>(loggerShm_.getPtr());
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

        // --------------------------------------------------------------------------------
        // Destructor
        // --------------------------------------------------------------------------------
        Control::~Control()
        {
            // Cleanup if needed
        }

        // --------------------------------------------------------------------------------
        // init()
        // --------------------------------------------------------------------------------
        bool Control::init()
        {
            // 1) Build a 6-axis model from ParameterServer
            {
                std::cout << "[Control] Building 6-axis model from paramServer.\n";

                // Load from paramServer
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

        // --------------------------------------------------------------------------------
        // run()
        // --------------------------------------------------------------------------------
        void Control::run()
        {
            cyclicTask();
        }

        // --------------------------------------------------------------------------------
        // requestStop()
        // --------------------------------------------------------------------------------
        void Control::requestStop()
        {
            stopRequested_.store(true, std::memory_order_relaxed);
        }

        // --------------------------------------------------------------------------------
        // Real-time loop
        // --------------------------------------------------------------------------------
        void Control::cyclicTask()
        {
            period_info pinfo;
            // Example: 1 ms cycle
            periodic_task_init(&pinfo, 1'000'000L);

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // 1) Read from hardware
                hal_->read();

                // 1a) Copy joint & I/O states -> shared memory
                copyJointStatesToSharedMemory();
                copyIoStatesToSharedMemory();

                // 1b) Check safety flags
                checkSafetyFlags();

                // (A) DriveStateManager
                {
                    int sigFrontIdx =
                        rtLayout_->driveUserSignalsBuffer.frontIndex.load(std::memory_order_acquire);
                    auto &sigBuf = rtLayout_->driveUserSignalsBuffer.buffer[sigFrontIdx];

                    int fdbkBackIdx =
                        1 - rtLayout_->driveFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
                    auto &fdbkBuf = rtLayout_->driveFeedbackBuffer.buffer[fdbkBackIdx];

                    driveStateManager_->update(
                        hal_->getDriveInputsPtr(),
                        hal_->getDriveOutputsPtr(),
                        sigBuf.signals.data(),
                        fdbkBuf.feedback.data(),
                        hal_->getDriveCount());

                    rtLayout_->driveFeedbackBuffer.frontIndex.store(
                        fdbkBackIdx, std::memory_order_release);
                }

                // // (B) ControllerManager
                // {
                //     int cmdFrontIdx =
                //         rtLayout_->controllerUserCmdBuffer.frontIndex.load(std::memory_order_acquire);
                //     auto &ctrlCmdBuf = rtLayout_->controllerUserCmdBuffer.buffer[cmdFrontIdx];

                //     int ctrlFdbkBackIdx =
                //         1 - rtLayout_->controllerFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
                //     auto &ctrlFdbkBuf = rtLayout_->controllerFeedbackBuffer.buffer[ctrlFdbkBackIdx];

                //     manager_->update(
                //         hal_->getJointStatesPtr(),
                //         hal_->getJointCommandsPtr(),
                //         ctrlCmdBuf.commands.data(),
                //         ctrlFdbkBuf.feedback.data(),
                //         static_cast<int>(hal_->getJointCount()),
                //         0.001 // dt in seconds (1 ms)
                //     );

                //     rtLayout_->controllerFeedbackBuffer.frontIndex.store(
                //         ctrlFdbkBackIdx, std::memory_order_release);
                // }

                // 3a) Copy updated commands from shared memory -> HAL
                copyJointCommandsFromSharedMemory();
                copyIoCommandsFromSharedMemory();

                // 4) Write to hardware
                hal_->write();

                // 5) Sleep until next cycle
                wait_rest_of_period(&pinfo);
            }

            // Optionally disable drives upon exit
            // driveStateManager_->disableAllDrives();
        }

        // --------------------------------------------------------------------------------
        // Scheduling Helpers
        // --------------------------------------------------------------------------------
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

        // --------------------------------------------------------------------------------
        // Copy Joint States TO Shared Memory
        // --------------------------------------------------------------------------------
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
                backStates[i].torque = halStates[i].torque;

                std::cout<<"joint pos: "<<i<<" : "<<halStates[i].position<<std::endl;
            }

            // Publish new data
            rtLayout_->jointBuffer.frontIndex.store(backIndex, std::memory_order_release);
        }

        // --------------------------------------------------------------------------------
        // Copy I/O States TO Shared Memory
        // --------------------------------------------------------------------------------
        void Control::copyIoStatesToSharedMemory()
        {
            int currentFront = rtLayout_->ioDataBuffer.frontIndex.load(std::memory_order_acquire);
            int backIndex = 1 - currentFront;

            auto &backIoStates = rtLayout_->ioDataBuffer.buffer[backIndex].states;
            size_t ioCount = hal_->getIoCount();

            auto *halIoStates = hal_->getIoStatesPtr();
            if (!halIoStates || ioCount == 0)
            {
                return; // No I/O
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

        // --------------------------------------------------------------------------------
        // Check Safety Flags
        // --------------------------------------------------------------------------------
        void Control::checkSafetyFlags()
        {
            // auto &sf = rtLayout_->safetyFeedback;
            // if (sf.limitExceeded || sf.overTemp || sf.eStop)
            // {
            //     std::cout << "[Control] Safety triggered: "
            //               << "limit=" << sf.limitExceeded
            //               << ", overTemp=" << sf.overTemp
            //               << ", eStop=" << sf.eStop
            //               << "\n";

            //     // Possibly disable drives or zero commands
            //     // driveStateManager_->forceDisableAllDrives();
            // }
        }

        // --------------------------------------------------------------------------------
        // Copy Joint Commands FROM Shared Memory
        // --------------------------------------------------------------------------------
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
                halCommands[i].torque = cmds[i].torque;
            }
        }

        // --------------------------------------------------------------------------------
        // Copy I/O Commands FROM Shared Memory
        // --------------------------------------------------------------------------------
        void Control::copyIoCommandsFromSharedMemory()
        {
            int frontIndex = rtLayout_->ioDataBuffer.frontIndex.load(std::memory_order_acquire);
            auto &ioCmds = rtLayout_->ioDataBuffer.buffer[frontIndex].commands;

            size_t ioCount = hal_->getIoCount();
            auto *halIoCommands = hal_->getIoCommandsPtr();

            if (!halIoCommands || ioCount == 0)
            {
                return; // No I/O
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

    } // namespace control
} // namespace hand_control
