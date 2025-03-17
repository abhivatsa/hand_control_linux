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
        /**
         * @brief The Logic class orchestrates high-level logic, bridging user commands,
         *        drive feedback, controller feedback, and a state machine, plus safety checks.
         */
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

            /**
             * @brief init
             *  - Initializes the state machine
             *  - Loads the HapticDeviceModel from paramServer
             *  - Re-initializes the SafetyManager with pointers
             *  - Returns true if all steps succeed
             */
            bool init();

            /**
             * @brief run
             *  - Starts the main cyclicTask loop
             */
            void run();

            /**
             * @brief requestStop
             *  - Signals the run() loop to exit gracefully
             */
            void requestStop();

        private:
            /**
             * @brief cyclicTask
             *  - Real-time loop running every ~10 ms
             *  - Reads aggregator data from shared memory
             *  - Runs safety checks & state machine
             *  - Writes commands & user feedback
             */
            void cyclicTask();

            // ---------------------------------------------------
            // Bridge / Aggregator I/O methods
            // ---------------------------------------------------
            void readUserCommands(hand_control::merai::UserCommands &out);
            void readDriveFeedback(hand_control::merai::DriveFeedbackData &out);
            void readControllerFeedback(hand_control::merai::ControllerFeedback &out);

            void writeDriveCommands(hand_control::merai::DriveCommandData &in);
            void writeControllerCommand(hand_control::merai::ControllerCommand &in);
            void writeUserFeedback(hand_control::merai::AppState currentState);

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

            // In-cycle flags
            bool isFaulted = false;
            bool isHomingCompleted = false;

            // Shared Memory for ParameterServer
            hand_control::merai::RAII_SharedMemory paramServerShm_;
            const hand_control::merai::ParameterServer *paramServerPtr_ = nullptr;

            // Shared Memory for real-time layout
            hand_control::merai::RAII_SharedMemory rtDataShm_;
            hand_control::merai::RTMemoryLayout *rtLayout_ = nullptr;

            // Shared Memory for Logger
            hand_control::merai::RAII_SharedMemory loggerShm_;
            hand_control::merai::multi_ring_logger_memory *loggerMem_ = nullptr;

            // StateMachine (replacing old SystemOrchestrator)
            StateMachine stateMachine_;

            // SafetyManager
            SafetyManager safetyManager_;

            // Temp aggregator structures
            hand_control::merai::UserCommands userCmds;
            hand_control::merai::DriveFeedbackData driveFdbk;
            hand_control::merai::ControllerFeedback ctrlFdbk;

            // Haptic device model
            hand_control::robotics::haptic_device::HapticDeviceModel hapticDeviceModel_;
        };

    } // namespace logic
} // namespace hand_control
