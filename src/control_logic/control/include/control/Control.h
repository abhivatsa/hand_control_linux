#pragma once

#include <memory>
#include <atomic>
#include <ctime> // for timespec, clock_gettime

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"
#include "merai/Enums.h"

#include "control/hardware_abstraction/BaseHAL.h"
#include "control/DriveStateManager.h"
#include "control/ControllerManager.h"

#include "hand_control_core/robotics_lib/haptic_device/HapticDeviceModel.h"

namespace hand_control
{
    namespace control
    {
        class Control
        {
        public:
            Control(const std::string &paramServerShmName, size_t paramServerShmSize,
                    const std::string &rtDataShmName, size_t rtDataShmSize,
                    const std::string &loggerShmName, size_t loggerShmSize);

            ~Control();

            bool init();
            void run();
            void requestStop();

        private:
            // Main periodic loop
            void cyclicTask();

            // Periodic task helper struct and methods
            struct period_info
            {
                struct timespec next_period;
                long period_ns;
            };

            void periodic_task_init(period_info *pinfo, long period_ns);
            void inc_period(period_info *pinfo);
            void wait_rest_of_period(period_info *pinfo);

            // Copy data between HAL and shared memory
            void copyJointFeedbackToSharedMemory();
            void copyJointCommandsFromSharedMemory();

            // Reading/writing additional commands/feedback
            void readControllerCommand(hand_control::merai::ControllerCommand *outCmd);
            void readDriveCommand(hand_control::merai::DriveCommandData *outDriveCmd);
            void writeDriveFeedback(const hand_control::merai::DriveFeedbackData &feedback);
            void writeControllerFeedback(const hand_control::merai::ControllerFeedback &feedback);

        private:
            // Local copies of user/drive/controller commands & feedback
            hand_control::merai::ControllerCommand  ctrlCmd;
            hand_control::merai::DriveCommandData   driveCmd;
            hand_control::merai::DriveFeedbackData  driveFdbk;
            hand_control::merai::ControllerFeedback ctrlFdbk;

            // HAL and managers
            std::unique_ptr<BaseHAL> hal_;
            std::unique_ptr<ControllerManager>  controllerManager_;
            std::unique_ptr<DriveStateManager>  driveStateManager_;

            std::atomic_bool stopRequested_{false};

            // Shared memory & param server references
            hand_control::merai::RAII_SharedMemory paramServerShm_;
            const hand_control::merai::ParameterServer* paramServerPtr_ = nullptr;

            hand_control::merai::RAII_SharedMemory rtDataShm_;
            hand_control::merai::RTMemoryLayout* rtLayout_ = nullptr;

            hand_control::merai::RAII_SharedMemory loggerShm_;
            hand_control::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            // Device model (for controllers like GravityCompController)
            hand_control::robotics::haptic_device::HapticDeviceModel hapticDeviceModel_;
        };

    } // namespace control
} // namespace hand_control
