#pragma once

#include <atomic>
#include <cstddef>
#include <memory> // for std::unique_ptr
#include <time.h>

// merai / Common headers
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/RAII_SharedMemory.h"
#include "merai/SharedLogger.h"
#include "merai/Enums.h"

// StateMachine (replacing old SystemOrchestrator)
#include "logic/StateMachine.h"

// Safety Manager
#include "logic/SafetyManager.h"

// HapticDeviceModel
#include "robotics_lib/haptic_device/HapticDeviceModel.h"

namespace seven_axis_robot
{
    namespace logic
    {
        class Logic
        {
        public:
            Logic(const std::string &paramServerShmName,
                  std::size_t paramServerShmSize,
                  const std::string &rtDataShmName,
                  std::size_t rtDataShmSize,
                  const std::string &loggerShmName,
                  std::size_t loggerShmSize);

            ~Logic();

            bool init();
            void run();
            void requestStop();

        private:
            void cyclicTask();

            // ---------------------------------------------------
            // Bridge / Aggregator I/O methods
            // ---------------------------------------------------
            void readUserCommands(seven_axis_robot::merai::UserCommands &out);
            void readDriveFeedback(seven_axis_robot::merai::DriveFeedbackData &out);
            void readControllerFeedback(seven_axis_robot::merai::ControllerFeedback &out);

            void writeDriveCommands(seven_axis_robot::merai::DriveCommandData &in);
            void writeControllerCommand(seven_axis_robot::merai::ControllerCommand &in);
            void writeUserFeedback(seven_axis_robot::merai::AppState currentState);

            // ---------------------------------------------------
            // Periodic scheduling helpers
            // ---------------------------------------------------
            struct period_info
            {
                struct timespec next_period;
                long period_ns;
            };
            void periodic_task_init(period_info *pinfo, long periodNs);
            void inc_period(period_info *pinfo);
            void wait_rest_of_period(period_info *pinfo);

        private:
            std::atomic<bool> stopRequested_{false};

            bool isFaulted        = false;
            bool isHomingCompleted= false;

            // Shared Memory for ParameterServer
            seven_axis_robot::merai::RAII_SharedMemory paramServerShm_;
            const seven_axis_robot::merai::ParameterServer *paramServerPtr_ = nullptr;

            // Shared Memory for real-time layout
            seven_axis_robot::merai::RAII_SharedMemory rtDataShm_;
            seven_axis_robot::merai::RTMemoryLayout *rtLayout_ = nullptr;

            // Shared Memory for Logger
            seven_axis_robot::merai::RAII_SharedMemory loggerShm_;
            seven_axis_robot::merai::multi_ring_logger_memory *loggerMem_ = nullptr;

            // The new StateMachine (replacing old SystemOrchestrator)
            // StateMachine stateMachine_;

            // SafetyManager now stored in a unique_ptr
            std::unique_ptr<SafetyManager> safetyManager_;
            std::unique_ptr<StateMachine> stateMachine_;

            // Temp aggregator structures
            seven_axis_robot::merai::UserCommands userCmds;
            seven_axis_robot::merai::DriveFeedbackData driveFdbk;
            seven_axis_robot::merai::ControllerFeedback ctrlFdbk;

            // Haptic device model
            seven_axis_robot::robotics::haptic_device::HapticDeviceModel hapticDeviceModel_;
        };
    } // namespace logic
} // namespace seven_axis_robot
