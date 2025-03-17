#include <stdexcept>
#include <time.h>

#include "control/Control.h"
#include "control/hardware_abstraction/SimHAL.h" // Changed to SimHAL
#include "control/hardware_abstraction/RealHAL.h"
#include "control/controllers/GravityCompController.h"
#include "control/controllers/HomingController.h"
#include "merai/Enums.h"

namespace hand_control
{
    namespace control
    {
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
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer *>(
                paramServerShm_.getPtr());
            if (!paramServerPtr_)
                throw std::runtime_error("Failed to map ParameterServer memory.");

            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout *>(
                rtDataShm_.getPtr());
            if (!rtLayout_)
                throw std::runtime_error("Failed to map RTMemoryLayout memory.");

            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory *>(
                loggerShm_.getPtr());
            if (!loggerMem_)
                throw std::runtime_error("Failed to map multi_ring_logger_memory.");

            // Initialize HAL based on simulation mode
            if (paramServerPtr_->startup.simulateMode)
            {
                hal_ = std::make_unique<SimHAL>(rtLayout_, paramServerPtr_, loggerMem_);
            }
            else
            {
                hal_ = std::make_unique<RealHAL>(rtLayout_, paramServerPtr_, loggerMem_);
            }

            // IMPORTANT: Use accessor methods to retrieve pointers
            driveStateManager_ = std::make_unique<DriveStateManager>(
                hal_->getDriveOutputControlPtr(),
                hal_->getDriveInputControlPtr(),
                hal_->getDriveCount());

            controllerManager_ = std::make_unique<ControllerManager>(
                paramServerPtr_,
                hal_->getJointStatesPtr(),
                hal_->getJointCommandsPtr(),
                hal_->getJointCount());
        }

        Control::~Control()
        {
        }

        bool Control::init()
        {
            // 1) Load device model
            if (!hapticDeviceModel_.loadFromParameterServer(*paramServerPtr_))
                return false;

            // 2) Init HAL
            if (!hal_->init())
                return false;

            // 3) Initialize managers with pointers
            if (!driveStateManager_->init()) // No need to pass drive inputs/outputs, already done in constructor
                return false;

            if (!controllerManager_->init())
                return false;

            // 4) Register controllers with correct constructor signatures

            // GravityCompController now expects:
            //   (const HapticDeviceModel&, JointState*, JointCommand*, std::size_t)
            auto gravityComp = std::make_shared<GravityCompController>(
                hapticDeviceModel_,
                hal_->getJointStatesPtr(),
                hal_->getJointCommandsPtr(),
                hal_->getJointCount());
            if (!controllerManager_->registerController(merai::ControllerID::GRAVITY_COMP, gravityComp))
                return false;

            // HomingController now expects:
            //   (const double* homePositions, int numJoints, JointState*, JointCommand*)
            double homePositions[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            auto homingCtrl = std::make_shared<HomingController>(
                homePositions,
                6,
                hal_->getJointStatesPtr(),
                hal_->getJointCommandsPtr());
            if (!controllerManager_->registerController(merai::ControllerID::HOMING, homingCtrl))
                return false;

            // 5) Finalize manager setup
            if (!controllerManager_->init()) // Possibly re-initializing or finalizing sub-controllers
                return false;

            return true;
        }

        void Control::run()
        {
            cyclicTask();
        }

        void Control::requestStop()
        {
            stopRequested_.store(true, std::memory_order_relaxed);
        }

        void Control::cyclicTask()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, 1'000'000L);

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // 1. HAL input
                hal_->read();
                copyJointStatesToSharedMemory();

                // 2. Command input (e.g., from user or other modules)
                readControllerCommand(&ctrlCmd);
                readDriveCommand(&driveCmd);

                // 3. Drive state & control updates
                driveStateManager_->update(driveCmd.commands.data(), driveFdbk.status.data());
                controllerManager_->update(ctrlCmd, ctrlFdbk, 0.001);

                // 4. Process outputs
                copyJointCommandsFromSharedMemory();
                writeDriveFeedback(driveFdbk);
                writeControllerFeedback(ctrlFdbk);

                // 5. HAL output
                hal_->write();

                wait_rest_of_period(&pinfo);
            }
        }

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
            }

            rtLayout_->jointBuffer.frontIndex.store(backIndex, std::memory_order_release);
        }

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

        void Control::readControllerCommand(hand_control::merai::ControllerCommand *outCmd)
        {
            int frontIdx = rtLayout_->controllerCommandBuffer.frontIndex.load(std::memory_order_acquire);
            *outCmd = rtLayout_->controllerCommandBuffer.buffer[frontIdx];
        }

        void Control::readDriveCommand(hand_control::merai::DriveCommandData *outDriveCmd)
        {
            int frontIdx = rtLayout_->driveCommandBuffer.frontIndex.load(std::memory_order_acquire);
            *outDriveCmd = rtLayout_->driveCommandBuffer.buffer[frontIdx];
        }

        void Control::writeDriveFeedback(const hand_control::merai::DriveFeedbackData &feedback)
        {
            size_t driveCount = paramServerPtr_->driveCount;
            int frontIdx = rtLayout_->driveFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx = 1 - frontIdx;

            auto &feedbackData = rtLayout_->driveFeedbackBuffer.buffer[backIdx];

            for (size_t i = 0; i < driveCount; ++i)
            {
                feedbackData.status[i] = feedback.status[i];
            }

            rtLayout_->driveFeedbackBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        void Control::writeControllerFeedback(const hand_control::merai::ControllerFeedback &feedback)
        {
            int frontIdx = rtLayout_->controllerFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx = 1 - frontIdx;

            rtLayout_->controllerFeedbackBuffer.buffer[backIdx] = feedback;
            rtLayout_->controllerFeedbackBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

    } // namespace control
} // namespace hand_control
