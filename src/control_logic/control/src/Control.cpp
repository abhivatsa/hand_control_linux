#include <stdexcept>
#include <ctime>

#include "control/Control.h"
#include "control/hardware_abstraction/SimHAL.h"
#include "control/hardware_abstraction/RealHAL.h"

// Example controllers
#include "control/controllers/GravityCompController.h"
#include "control/controllers/HomingController.h"

#include "merai/Enums.h"  // for e.g. DriveCommand, ControllerID

namespace seven_axis_robot
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
            // 1) ParameterServer mapping
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer*>(
                paramServerShm_.getPtr());
            if (!paramServerPtr_)
            {
                throw std::runtime_error("Failed to map ParameterServer memory.");
            }

            // 2) RTMemoryLayout mapping
            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout*>(
                rtDataShm_.getPtr());
            if (!rtLayout_)
            {
                throw std::runtime_error("Failed to map RTMemoryLayout memory.");
            }

            // 3) Logger memory
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory*>(
                loggerShm_.getPtr());
            if (!loggerMem_)
            {
                throw std::runtime_error("Failed to map multi_ring_logger_memory.");
            }

            // 4) Initialize HAL (simulate or real)
            // TODO: Wire simulate/real selection from GUI/runtime input instead of config.
            const bool simulateMode = false;
            if (simulateMode)
            {
                hal_ = std::make_unique<SimHAL>(rtLayout_, paramServerPtr_, loggerMem_);
            }
            else
            {
                hal_ = std::make_unique<RealHAL>(rtLayout_, paramServerPtr_, loggerMem_);
            }

            // 5) Create DriveStateManager with joint-level control/feedback pointers
            driveStateManager_ = std::make_unique<DriveStateManager>(
                hal_->getJointControlCommandPtr(),
                hal_->getJointControlFeedbackPtr(),
                hal_->getDriveCount() // or getJointCount(), if they match
            );

            // 6) Create ControllerManager with motion-level pointers
            controllerManager_ = std::make_unique<ControllerManager>(
                paramServerPtr_,
                // The manager presumably wants motion feedback + motion commands:
                hal_->getJointMotionFeedbackPtr(),
                hal_->getJointMotionCommandPtr(),
                hal_->getDriveCount() // or getJointCount()
            );
        }

        Control::~Control()
        {
            // Cleanup if needed
        }

        bool Control::init()
        {
            // 1) Load device model from param server
            if (!hapticDeviceModel_.loadFromParameterServer(*paramServerPtr_))
                return false;

            // 2) Initialize HAL
            if (!hal_->init())
                return false;

            // 3) Initialize the drive manager & controller manager
            if (!driveStateManager_->init())
                return false;
            if (!controllerManager_->init())
                return false;

            // 4) Register controllers (GravityComp, Homing, etc.)
            auto gravityComp = std::make_shared<GravityCompController>(
                hapticDeviceModel_,
                hal_->getJointMotionFeedbackPtr(),  // motion feedback
                hal_->getJointMotionCommandPtr(),   // motion commands
                hal_->getDriveCount()
            );
            if (!controllerManager_->registerController(merai::ControllerID::GRAVITY_COMP, gravityComp))
                return false;

            // Example for Homing
            double homePositions[7] = {-0.82, 1.336, 0.0, 0.4724, -0.504, 0.0, 0.0};
            auto homingCtrl = std::make_shared<HomingController>(
                homePositions,
                hal_->getDriveCount(),
                hal_->getJointMotionFeedbackPtr(),
                hal_->getJointMotionCommandPtr()
            );
            if (!controllerManager_->registerController(merai::ControllerID::HOMING, homingCtrl))
                return false;

            // Possibly do a final init if your manager needs it
            if (!controllerManager_->init())
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

        //----------------------------------------------------------------------------
        // The main real-time loophal_->getDriveCount()
        //----------------------------------------------------------------------------
        void Control::cyclicTask()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, 1'000'000L); // e.g., 1 ms

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                // 1) HAL read -> local feedback arrays
                hal_->read();

                // 2) Copy local feedback arrays -> shared memory
                copyJointFeedbackToSharedMemory();

                // 3) Read any user/drive commands from shared memory
                readControllerCommand(&ctrlCmd);
                readDriveCommand(&driveCmd);

                // std::cout<<"Drive Command : "<<int(driveCmd.commands[0])<<std::endl;

                // 4) Update drive states
                driveStateManager_->update(driveCmd.commands.data(), driveFdbk.status.data());

                // 5) Update controllers
                controllerManager_->update(ctrlCmd, ctrlFdbk, 0.001);

                // std::cout<<"Control Word : "<<hal_->getJointControlCommandPtr()[0].controlWord<<std::endl;

                // 6) Copy joint commands from shared memory -> local command arrays
                // copyJointCommandsFromSharedMemory();

                // 7) Write drive/controller feedback to shared memory
                writeDriveFeedback(driveFdbk);
                writeControllerFeedback(ctrlFdbk);

                // 8) HAL write -> real hardware or sim
                hal_->write();

                // 9) Sleep until next period
                wait_rest_of_period(&pinfo);
            }
        }

        //----------------------------------------------------------------------------
        // Periodic helpers
        //----------------------------------------------------------------------------
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

        //----------------------------------------------------------------------------
        // Copy from HAL's joint feedback arrays -> shared memory
        //----------------------------------------------------------------------------
        void Control::copyJointFeedbackToSharedMemory()
        {
            int currentFront = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            int backIndex    = 1 - currentFront;

            auto &backFeedbackArray = rtLayout_->jointBuffer.buffer[backIndex].feedback;

            // Number of joints
            int jointCount = static_cast<int>(hal_->getDriveCount()); // or getJointCount()

            // Pointers from HAL
            auto *ctrlFbk   = hal_->getJointControlFeedbackPtr();
            auto *motionFbk = hal_->getJointMotionFeedbackPtr();
            auto *ioFbk     = hal_->getJointFeedbackIOPtr();

            for (int i = 0; i < jointCount; i++)
            {
                // Control feedback
                backFeedbackArray[i].control.statusWord = ctrlFbk[i].statusWord;

                // Motion feedback
                backFeedbackArray[i].motion.positionActual = motionFbk[i].positionActual;
                backFeedbackArray[i].motion.velocityActual = motionFbk[i].velocityActual;
                backFeedbackArray[i].motion.torqueActual   = motionFbk[i].torqueActual;

                // I/O feedback
                backFeedbackArray[i].io.digitalInputClutch = ioFbk[i].digitalInputClutch;
                backFeedbackArray[i].io.digitalInputThumb  = ioFbk[i].digitalInputThumb;
                backFeedbackArray[i].io.analogInputPinch   = ioFbk[i].analogInputPinch;
            }

            // Publish the 'back' index
            rtLayout_->jointBuffer.frontIndex.store(backIndex, std::memory_order_release);
        }

        //----------------------------------------------------------------------------
        // Copy from shared memory's joint commands -> HAL's local command arrays
        //----------------------------------------------------------------------------
        void Control::copyJointCommandsFromSharedMemory()
        {
            int frontIndex = rtLayout_->jointBuffer.frontIndex.load(std::memory_order_acquire);
            auto &cmdArray = rtLayout_->jointBuffer.buffer[frontIndex].commands;

            int jointCount = static_cast<int>(hal_->getDriveCount());

            // Pointers from HAL
            auto *ctrlCmd   = hal_->getJointControlCommandPtr();
            auto *motionCmd = hal_->getJointMotionCommandPtr();

            for (int i = 0; i < jointCount; i++)
            {
                // Control command
                // ctrlCmd[i].controlWord = cmdArray[i].control.controlWord;

                // Motion command
                motionCmd[i].targetPosition   = cmdArray[i].motion.targetPosition;
                motionCmd[i].targetTorque     = cmdArray[i].motion.targetTorque;
                motionCmd[i].modeOfOperation  = cmdArray[i].motion.modeOfOperation;

                // IO commands if any
                // if (ioCmd) { ... }
            }
        }

        //----------------------------------------------------------------------------
        // Read ControllerCommand from shared memory
        //----------------------------------------------------------------------------
        void Control::readControllerCommand(seven_axis_robot::merai::ControllerCommand *outCmd)
        {
            int frontIdx = rtLayout_->controllerCommandBuffer.frontIndex.load(std::memory_order_acquire);
            *outCmd = rtLayout_->controllerCommandBuffer.buffer[frontIdx];
        }

        // Read DriveCommandData from shared memory
        void Control::readDriveCommand(seven_axis_robot::merai::DriveCommandData *outDriveCmd)
        {
            int frontIdx = rtLayout_->driveCommandBuffer.frontIndex.load(std::memory_order_acquire);
            *outDriveCmd = rtLayout_->driveCommandBuffer.buffer[frontIdx];
        }

        //----------------------------------------------------------------------------
        // Write DriveFeedbackData to shared memory
        //----------------------------------------------------------------------------
        void Control::writeDriveFeedback(const seven_axis_robot::merai::DriveFeedbackData &feedback)
        {
            size_t driveCount = paramServerPtr_->driveCount;
            int frontIdx = rtLayout_->driveFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;

            auto &feedbackData = rtLayout_->driveFeedbackBuffer.buffer[backIdx];

            for (size_t i = 0; i < driveCount; ++i)
            {
                feedbackData.status[i] = feedback.status[i];
            }

            rtLayout_->driveFeedbackBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

        // Write ControllerFeedback to shared memory
        void Control::writeControllerFeedback(const seven_axis_robot::merai::ControllerFeedback &feedback)
        {
            int frontIdx = rtLayout_->controllerFeedbackBuffer.frontIndex.load(std::memory_order_acquire);
            int backIdx  = 1 - frontIdx;

            rtLayout_->controllerFeedbackBuffer.buffer[backIdx] = feedback;
            rtLayout_->controllerFeedbackBuffer.frontIndex.store(backIdx, std::memory_order_release);
        }

    } // namespace control
} // namespace seven_axis_robot
