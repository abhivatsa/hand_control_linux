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

// Orchestrator (logic layer)
#include "logic/SystemOrchestrator.h"

// The aggregator data structs
#include "logic/LogicTypes.h"

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
            Logic(const std::string& paramServerShmName, 
                  std::size_t paramServerShmSize,
                  const std::string& rtDataShmName, 
                  std::size_t rtDataShmSize,
                  const std::string& loggerShmName, 
                  std::size_t loggerShmSize);

            ~Logic();

            bool init();
            void run();
            void requestStop();

        private:
            void cyclicTask();

            // Helper aggregator I/O
            void readUserCommandsFromAggregator(UserCommandsData& out);
            void readDriveFeedbackAggregator(DriveFeedbackData& out);
            void readControllerFeedbackAggregator(ControllerFeedbackData& out);

            void writeDriveControlSignalsAggregator();
            void writeControllerSwitchAggregator(bool switchWanted,
                                                 hand_control::merai::ControllerID ctrlId);
            void writeUserFeedbackAggregator(bool isFaulted,
                                             merai::OrchestratorState currentState,
                                             const UserCommandsData& userCmds);

            // Periodic scheduling
            struct period_info
            {
                struct timespec next_period;
                long period_ns;
            };
            void periodic_task_init(period_info* pinfo, long periodNs);
            void inc_period(period_info* pinfo);
            void wait_rest_of_period(period_info* pinfo);

        private:
            std::atomic<bool> stopRequested_{false};

            merai::RAII_SharedMemory paramServerShm_;
            const merai::ParameterServer* paramServerPtr_ = nullptr;

            merai::RAII_SharedMemory rtDataShm_;
            merai::RTMemoryLayout* rtLayout_ = nullptr;

            merai::RAII_SharedMemory loggerShm_;
            merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            // Orchestrator
            SystemOrchestrator systemOrchestrator_;

            // Our SafetyManager
            SafetyManager safetyManager_;

            // -------------------------------------------------
            // NEW: Haptic device model for logic usage
            // -------------------------------------------------
            hand_control::robotics::haptic_device::HapticDeviceModel hapticDeviceModel_;
        };
    } // namespace logic
} // namespace hand_control
