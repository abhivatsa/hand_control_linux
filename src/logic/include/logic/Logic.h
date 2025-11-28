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

#include "robotics_lib/RobotModel.h"

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
            void readUserCommands(merai::UserCommands &out);
            void readDriveFeedback(merai::DriveFeedbackData &out);
            void readControllerFeedback(merai::ControllerFeedback &out);

            void writeDriveCommands(merai::DriveCommandData &in);
            void writeControllerCommand(merai::ControllerCommand &in);
            void writeUserFeedback(merai::AppState currentState);

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
            merai::RAII_SharedMemory paramServerShm_;
            const merai::ParameterServer *paramServerPtr_ = nullptr;

            // Shared Memory for real-time layout
            merai::RAII_SharedMemory rtDataShm_;
            merai::RTMemoryLayout *rtLayout_ = nullptr;

            // Shared Memory for Logger
            merai::RAII_SharedMemory loggerShm_;
            merai::multi_ring_logger_memory *loggerMem_ = nullptr;

            // Robot model (for safety/logic if needed)
            robot_lib::RobotModel robotModel_;

            // The new StateMachine (replacing old SystemOrchestrator)
            // StateMachine stateMachine_;

            // SafetyManager now stored in a unique_ptr
            std::unique_ptr<SafetyManager> safetyManager_;
            std::unique_ptr<StateMachine> stateMachine_;

            // Temp aggregator structures
            merai::UserCommands userCmds;
            merai::DriveFeedbackData driveFdbk;
            merai::ControllerFeedback ctrlFdbk;

        };
    } // namespace logic
