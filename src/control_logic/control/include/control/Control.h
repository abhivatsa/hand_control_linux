#pragma once

#include <memory>
#include <atomic>
#include <time.h> // for timespec

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"
#include "merai/Enums.h"
#include "control/hardware_abstraction/IHardwareAbstractionLayer.h"
#include "control/DriveStateManager.h"
#include "control/ControllerManager.h"
#include "hand_control_core/robotics_lib/haptic_device/HapticDeviceModel.h"

namespace hand_control
{
    namespace control
    {
        /**
         * @brief Main control class that runs the real-time loop.
         */
        class Control
        {
        public:
            Control(const std::string &paramServerShmName, size_t paramServerShmSize,
                    const std::string &rtDataShmName,      size_t rtDataShmSize,
                    const std::string &loggerShmName,      size_t loggerShmSize);

            ~Control();

            /**
             * @brief Initializes the control system.
             */
            bool init();

            /**
             * @brief Starts the control loop (blocking).
             */
            void run();

            /**
             * @brief Requests a stop, causing run() to exit.
             */
            void requestStop();

        private:
            void cyclicTask();

            struct period_info
            {
                struct timespec next_period;
                long period_ns;
            };

            void periodic_task_init(period_info* pinfo, long period_ns);
            void inc_period(period_info* pinfo);
            void wait_rest_of_period(period_info* pinfo);

            void copyJointStatesToSharedMemory();
            void checkSafetyFlags();
            void copyJointCommandsFromSharedMemory();

            struct ControllerCommandAggregated
            {
                bool requestSwitch = false;
                hand_control::merai::ControllerID targetController = hand_control::merai::ControllerID::NONE;
            };

            ControllerCommandAggregated readControllerCommandAggregator();
            void applyControllerSwitch(const ControllerCommandAggregated& ctrlCmd);

            void updateDriveStateManager();
            void updateControllerManager();

        private:
            std::unique_ptr<IHardwareAbstractionLayer> hal_;
            std::unique_ptr<ControllerManager> controllerManager_;
            std::unique_ptr<DriveStateManager> driveStateManager_;

            std::atomic_bool stopRequested_{false};

            hand_control::merai::RAII_SharedMemory paramServerShm_;
            const hand_control::merai::ParameterServer* paramServerPtr_ = nullptr;

            hand_control::merai::RAII_SharedMemory rtDataShm_;
            hand_control::merai::RTMemoryLayout* rtLayout_ = nullptr;

            hand_control::merai::RAII_SharedMemory loggerShm_;
            hand_control::merai::multi_ring_logger_memory* loggerMem_ = nullptr;

            hand_control::robotics::haptic_device::HapticDeviceModel hapticDeviceModel_;
        };

    } // namespace control
} // namespace hand_control
