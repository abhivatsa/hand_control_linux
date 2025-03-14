#pragma once

#include <atomic>
#include <cstddef>
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

namespace hand_control
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
            // Bridge I/O methods (formerly "aggregator" I/O)
            // ---------------------------------------------------
            void readUserCommands(hand_control::merai::UserCommands &out);
            void readDriveFeedback(hand_control::merai::DriveFeedbackData &out);
            void readControllerFeedback(hand_control::merai::ControllerFeedbackData &out);

            void writeDriveCommands();
            void writeControllerCommand(bool switchWanted,
                                               hand_control::merai::ControllerID ctrlId);
            void writeUserFeedbackToBridge(bool isFaulted,
                                           merai::AppState currentState,
                                           const hand_control::merai::UserCommands &userCmds);

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

            bool isFaulted;
            bool isHomingCompleted;

            // Shared Memory for ParameterServer
            merai::RAII_SharedMemory paramServerShm_;
            const merai::ParameterServer *paramServerPtr_ = nullptr;

            // Shared Memory for real-time layout
            merai::RAII_SharedMemory rtDataShm_;
            merai::RTMemoryLayout *rtLayout_ = nullptr;

            // Shared Memory for Logger
            merai::RAII_SharedMemory loggerShm_;
            merai::multi_ring_logger_memory *loggerMem_ = nullptr;

            // The new "StateMachine" (replacing old "SystemOrchestrator")
            StateMachine stateMachine_;

            // SafetyManager
            SafetyManager safetyManager_;

            hand_control::merai::UserCommands userCmds;
            hand_control::merai::DriveFeedbackData driveFdbk;
            hand_control::merai::ControllerFeedbackData ctrlFdbk;

            // Haptic device model
            hand_control::robotics::haptic_device::HapticDeviceModel hapticDeviceModel_;
        };
    } // namespace logic
} // namespace hand_control
