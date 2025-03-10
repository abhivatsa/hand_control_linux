#include <stdexcept>   // for std::runtime_error
#include <time.h>      // clock_gettime, clock_nanosleep

#include "control/Control.h"
#include "control/hardware_abstraction/MockHardwareAbstractionLayer.h"
#include "control/hardware_abstraction/RealHardwareAbstractionLayer.h"
// Existing Gravity Controller:
#include "control/controllers/GravityCompController.h"
// Include your Homing Controller header:
#include "control/controllers/HomingController.h"

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
                throw std::runtime_error("Control: Failed to map ParameterServer memory.");
            }

            // 2) Attach & map RTMemoryLayout
            rtLayout_ =
                reinterpret_cast<hand_control::merai::RTMemoryLayout*>(
                    rtDataShm_.getPtr());
            if (!rtLayout_)
            {
                throw std::runtime_error("Control: Failed to map RTMemoryLayout memory.");
            }

            // 3) Attach & map Logger memory
            loggerMem_ =
                reinterpret_cast<hand_control::merai::multi_ring_logger_memory*>(
                    loggerShm_.getPtr());
            if (!loggerMem_)
            {
                throw std::runtime_error("Control: Failed to map multi_ring_logger_memory.");
            }

            // Create HAL (mock or real)
            if (paramServerPtr_->startup.simulateMode)
            {
                hal_ = std::make_unique<MockHardwareAbstractionLayer>(
                           rtLayout_, paramServerPtr_, loggerMem_);
            }
            else
            {
                hal_ = std::make_unique<RealHardwareAbstractionLayer>(
                           rtLayout_, paramServerPtr_, loggerMem_);
            }

            // Create the ControllerManager
            controllerManager_   = std::make_unique<ControllerManager>(paramServerPtr_);
            driveStateManager_   = std::make_unique<DriveStateManager>();
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
            // 1) Build a Haptic device model from ParameterServer
            if (!hapticDeviceModel_.loadFromParameterServer(*paramServerPtr_))
            {
                return false;
            }

            // 2) Init HAL
            if (!hal_->init())
            {
                return false;
            }

            // 3) Register controllers (ID-based)
            {
                // Example: GravityCompController => GRAVITY_COMP
                auto gravityComp = std::make_shared<GravityCompController>(hapticDeviceModel_);
                if (!controllerManager_->registerController(
                        hand_control::merai::ControllerID::GRAVITY_COMP,
                        gravityComp))
                {
                    return false;
                }

                // Example: HomingController => HOMING
                // You can load these homePositions from the ParameterServer if desired.
                double homePositions[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                auto homingCtrl = std::make_shared<HomingController>(homePositions, 6);
                if (!controllerManager_->registerController(
                        hand_control::merai::ControllerID::HOMING,
                        homingCtrl))
                {
                    return false;
                }

                // If you have other controllers, register them similarly:
                // auto eStopCtrl = std::make_shared<EStopController>();
                // controllerManager_->registerController(hand_control::merai::ControllerID::E_STOP, eStopCtrl);
                // ...
            }

            // 4) Init the controllerManager
            if (!controllerManager_->init())
            {
                return false;
            }

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
            periodic_task_init(&pinfo, 1'000'000L); // 1 ms cycle

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // (1) Read from hardware
                hal_->read();

                // (2) Copy states -> shared memory
                copyJointStatesToSharedMemory();

                // (3) Read aggregator for new controller switch
                auto ctrlCmdAgg = readControllerCommandAggregator();

                // (4) Possibly apply aggregator -> 'controllerCommandBuffer'
                applyControllerSwitch(ctrlCmdAgg);

                // (5) Sub-managers
                updateDriveStateManager();
                updateControllerManager(); // calls controllerManager_->update(...)

                // (6) Copy updated commands -> hardware
                copyJointCommandsFromSharedMemory();
                hal_->write();

                // (7) Sleep until next cycle
                wait_rest_of_period(&pinfo);
            }

            // optionally disable drives upon exit
        }

        // -------------------------------------------------------------------
        // Scheduling Helpers
        // -------------------------------------------------------------------
        void Control::periodic_task_init(period_info* pinfo, long periodNs)
        {
            clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
            pinfo->period_ns = periodNs;
        }

        void Control::inc_period(period_info* pinfo)
        {
            pinfo->next_period.tv_nsec += pinfo->period_ns;
            if (pinfo->next_period.tv_nsec >= 1'000'000'000L)
            {
                pinfo->next_period.tv_nsec -= 1'000'000'000L;
                pinfo->next_period.tv_sec++;
            }
        }

        void Control::wait_rest_of_period(period_info* pinfo)
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
            int backIndex    = 1 - currentFront;

            auto &backStates = rtLayout_->jointBuffer.buffer[backIndex].states;
            int jointCount   = static_cast<int>(hal_->getJointCount());
            auto *halStates  = hal_->getJointStatesPtr();

            for (int i = 0; i < jointCount; i++)
            {
                backStates[i].position = halStates[i].position;
                backStates[i].velocity = halStates[i].velocity;
                backStates[i].torque   = halStates[i].torque;
            }

            rtLayout_->jointBuffer.frontIndex.store(backIndex, std::memory_order_release);
        }

        void Control::checkSafetyFlags()
        {
            // Possibly read from a separate aggregator or structure
            // If triggered => handle it
        }

        // -------------------------------------------------------------------
        // Copy Commands from Shared Memory -> Hardware
        // -------------------------------------------------------------------
        void Control::copyJointCommandsFromSharedMemory()
        {
            int frontIndex = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &cmds     = rtLayout_->jointBuffer.buffer[frontIndex].commands;

            int jointCount     = static_cast<int>(hal_->getJointCount());
            auto *halCommands  = hal_->getJointCommandsPtr();

            for (int i = 0; i < jointCount; i++)
            {
                halCommands[i].position = cmds[i].position;
                halCommands[i].velocity = cmds[i].velocity;
                halCommands[i].torque   = cmds[i].torque;
            }
        }

        // -------------------------------------------------------------------
        // Aggregator Helpers
        // -------------------------------------------------------------------
        Control::ControllerCommandAggregated Control::readControllerCommandAggregator()
        {
            ControllerCommandAggregated local{};
            int frontIdx =
                rtLayout_->controllerCommandsAggBuffer.frontIndex.load(std::memory_order_acquire);
            const auto &agg = rtLayout_->controllerCommandsAggBuffer.buffer[frontIdx];

            local.requestSwitch    = agg.requestSwitch;
            local.targetController = agg.targetController;
            return local;
        }

        void Control::applyControllerSwitch(const ControllerCommandAggregated &ctrlCmd)
        {
            if (!ctrlCmd.requestSwitch)
                return;

            // Write aggregator => 'controllerCommandBuffer' for the manager
            int cFrontIdx =
                rtLayout_->controllerCommandBuffer.frontIndex.load(std::memory_order_acquire);
            int cBackIdx  = 1 - cFrontIdx;
            auto &ctrlBuf = rtLayout_->controllerCommandBuffer.buffer[cBackIdx];

            ctrlBuf.commands[0].requestSwitch = true;
            ctrlBuf.commands[0].controllerId  = ctrlCmd.targetController;

            rtLayout_->controllerCommandBuffer.frontIndex.store(cBackIdx, std::memory_order_release);
        }

        // -------------------------------------------------------------------
        // Drive State Manager
        // -------------------------------------------------------------------
        void Control::updateDriveStateManager()
        {
            int signalsFrontIdx =
                rtLayout_->driveControlSignalsBuffer.frontIndex.load(std::memory_order_acquire);
            auto &signalsBuf = rtLayout_->driveControlSignalsBuffer.buffer[signalsFrontIdx];

            int fdbkBackIdx =
                1 - rtLayout_->driveFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            auto &fdbkBuf = rtLayout_->driveFeedbackBuffer.buffer[fdbkBackIdx];

            driveStateManager_->update(
                hal_->getDriveInputsPtr(),
                hal_->getDriveOutputsPtr(),
                signalsBuf.signals.data(),
                fdbkBuf.feedback.data(),
                hal_->getDriveCount()
            );

            rtLayout_->driveFeedbackBuffer.frontIndex.store(
                fdbkBackIdx, std::memory_order_release);
        }

        // -------------------------------------------------------------------
        // ControllerManager Updates
        // -------------------------------------------------------------------
        void Control::updateControllerManager()
        {
            // We read the final aggregator => 'controllerCommandBuffer'
            int cmdFrontIdx =
                rtLayout_->controllerCommandBuffer.frontIndex.load(std::memory_order_acquire);
            auto &ctrlCmdBuf = rtLayout_->controllerCommandBuffer.buffer[cmdFrontIdx];

            // Manager feedback
            int fbkBackIdx =
                1 - rtLayout_->controllerFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            auto &ctrlFdbkBuf = rtLayout_->controllerFeedbackBuffer.buffer[fbkBackIdx];

            // The manager (renamed to controllerManager_) updates the active controller
            controllerManager_->update(
                hal_->getJointStatesPtr(),
                hal_->getJointCommandsPtr(),
                ctrlCmdBuf.commands.data(),
                ctrlFdbkBuf.feedback.data(),
                0.001 // dt in seconds
            );

            rtLayout_->controllerFeedbackBuffer.frontIndex.store(
                fbkBackIdx, std::memory_order_release);
        }

    } // namespace control
} // namespace hand_control
