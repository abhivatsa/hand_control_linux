#pragma once

#include <memory>
#include <atomic>
#include <string>
#include <time.h> // for timespec

// merai / Common headers
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"

// Control-layer includes
#include "control/hardware_abstraction/IHardwareAbstractionLayer.h"
#include "control/DriveStateManager.h"
#include "control/ControllerManager.h"

// Robot-specific includes (hand control)
#include "robotics_lib/haptic_device/HapticDeviceModel.h"

namespace hand_control
{
    namespace control
    {
        // Example enum for DriveCommand if you want to store an integer in aggregator
        enum class DriveCommand : int
        {
            NONE        = 0,
            ENABLE_ALL  = 1,
            DISABLE_ALL = 2,
            QUICK_STOP  = 3,
            FAULT_RESET = 4,
            // etc.
        };

        /**
         * @brief Main control class that attaches to shared memory segments (ParameterServer,
         *        RTMemoryLayout, and Logger) to run the real-time control loop.
         */
        class Control
        {
        public:
            /**
             * @brief Constructor attaches to:
             *  - ParameterServer (paramServerShmName)
             *  - RTMemoryLayout (rtDataShmName)
             *  - multi_ring_logger_memory (loggerShmName)
             *
             * Throws std::runtime_error on any SHM attach failure.
             */
            Control(const std::string &paramServerShmName, size_t paramServerShmSize,
                    const std::string &rtDataShmName, size_t rtDataShmSize,
                    const std::string &loggerShmName, size_t loggerShmSize);

            ~Control();

            /**
             * @brief Initializes the control system, including:
             *        - Building a 6-axis model from ParameterServer data
             *        - Initializing hardware abstraction (mock or real)
             *        - Registering & initializing controllers
             * @return true if initialization succeeds, false otherwise.
             */
            bool init();

            /**
             * @brief Starts the main (blocking) loop of the control application.
             *        This typically runs at a fixed period (e.g., 1ms).
             */
            void run();

            /**
             * @brief Signals the control loop to stop gracefully.
             *        Once stopped, run() will exit its loop.
             */
            void requestStop();

        private:
            /**
             * @brief The real-time control loop, typically scheduled with a period
             *        (e.g., 1ms). Reads hardware, updates states, runs controllers,
             *        and writes new commands.
             */
            void cyclicTask();

            /**
             * @brief A small helper struct for handling periodic tasks.
             */
            struct period_info
            {
                struct timespec next_period;
                long period_ns;
            };

            // Periodic scheduling helpers
            void periodic_task_init(period_info *pinfo, long period_ns);
            void inc_period(period_info *pinfo);
            void wait_rest_of_period(period_info *pinfo);

            // Copy methods for joint and I/O data
            void copyJointStatesToSharedMemory();
            void copyIoStatesToSharedMemory();
            void checkSafetyFlags();
            void copyJointCommandsFromSharedMemory();
            void copyIoCommandsFromSharedMemory();

            // ==============================
            // Aggregator read/write helpers
            // ==============================

            /**
             * @brief readDriveCommandAggregator
             *  Reads the integer from driveCommandBuffer, converting to DriveCommand enum
             */
            DriveCommand readDriveCommandAggregator();

            /**
             * @brief readControllerCommandAggregator
             *  Reads from controllerCommandsAggBuffer (requestSwitch + name)
             */
            struct ControllerCommandAggregated
            {
                bool requestSwitch = false;
                char targetControllerName[64];
            };
            ControllerCommandAggregated readControllerCommandAggregator();

            /**
             * @brief writeDriveSummaryAggregator
             *  Writes whether any fault and severity for the logic to read
             */
            void writeDriveSummaryAggregator();

            /**
             * @brief applyDriveCommand
             *  Interprets the enumerated DriveCommand, sets local signals or user signals buffer
             */
            void applyDriveCommand(DriveCommand cmd);

            /**
             * @brief applyControllerSwitch
             *  If requestSwitch is true, fill the controllerUserCmdBuffer or call manager
             */
            void applyControllerSwitch(const ControllerCommandAggregated &ctrlCmd);

            /**
             * @brief sub-manager updates
             */
            void updateDriveStateManager();
            void updateControllerManager();

        private:
            // HAL (real or mock)
            std::unique_ptr<IHardwareAbstractionLayer> hal_;

            // Managers
            std::unique_ptr<ControllerManager> manager_;
            std::unique_ptr<DriveStateManager> driveStateManager_;

            // Flag to stop the loop
            std::atomic_bool stopRequested_{false};

            // SHM for ParameterServer
            hand_control::merai::RAII_SharedMemory paramServerShm_;
            const hand_control::merai::ParameterServer *paramServerPtr_ = nullptr;

            // SHM for RTMemoryLayout
            hand_control::merai::RAII_SharedMemory rtDataShm_;
            hand_control::merai::RTMemoryLayout *rtLayout_ = nullptr;

            // SHM for Logger
            hand_control::merai::RAII_SharedMemory loggerShm_;
            hand_control::merai::multi_ring_logger_memory *loggerMem_ = nullptr;

            hand_control::robotics::haptic_device::HapticDeviceModel hapticDeviceModel_;
        };

    } // namespace control
} // namespace hand_control
