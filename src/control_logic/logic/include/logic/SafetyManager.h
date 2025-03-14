#pragma once

#include <cstddef> // for size_t
#include <cstdint>

// Include the Haptic Device model & math libs
#include "robotics_lib/haptic_device/HapticDeviceModel.h"
#include "robotics_lib/haptic_device/HapticDeviceKinematics.h"
#include "robotics_lib/haptic_device/HapticDeviceDynamics.h"

// Shared memory structures
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/Enums.h" // for SystemMode, OrchestratorState, etc.

// Aggregator Data
#include "logic/Logic.h" // or wherever UserCommandsData, DriveFeedbackData, ControllerFeedbackData are defined

namespace hand_control
{
    namespace logic
    {
        /**
         * @brief A manager that checks system safety conditions each cycle (eStop, joint limits,
         *        motion constraints, and more). If a fault occurs, sets an internal flag.
         */
        class SafetyManager
        {
        public:
            /**
             * @brief Construct a SafetyManager referencing the parameter server, the RTMemoryLayout,
             *        and the haptic device model. 
             *
             * @param paramServerPtr  For reading joint limits, gear ratios, etc.
             * @param rtLayout        For accessing aggregator data if needed (joint states, etc.).
             * @param model           A reference to the haptic device model (6 DOF or otherwise).
             */
            SafetyManager(const hand_control::merai::ParameterServer* paramServerPtr,
                          hand_control::merai::RTMemoryLayout*        rtLayout,
                          const hand_control::robotics::haptic_device::HapticDeviceModel& model);

            ~SafetyManager() = default;

            /**
             * @brief init()
             *  - Reads joint limits / gear ratio / offsets from paramServer.
             *  - Initializes kinematics_ and dynamics_ if needed.
             *  - Sets initial state to not faulted.
             */
            bool init();

            /**
             * @brief update()
             *  - Checks aggregator data for user eStop, drive faults, controller errors, 
             *    joint position limits, torque limits, etc.
             *  - If any condition fails, sets faulted_ = true.
             *  - If userCmds.resetFault is set, tries to clear the fault (if safe).
             *
             * @param driveFdbk   The drive feedback aggregator data (drive faults, etc.).
             * @param userCmds    The user commands aggregator data (eStop, resetFault, shutdown, etc.).
             * @param ctrlFdbk    The controller feedback aggregator data (e.g. if a controller signals error).
             * @return bool       True if the system is currently faulted, false otherwise.
             */
            bool update(const hand_control::merai::DriveFeedbackData&    driveFdbk,
                        const hand_control::merai::UserCommands&         userCmds,
                        const hand_control::merai::ControllerFeedbackData& ctrlFdbk);

            /**
             * @brief isFaulted()
             *  - Returns whether the manager has detected a fault condition.
             */
            bool isFaulted() const;

            /**
             * @brief forceFault()
             *  - Manually triggers a fault.
             */
            void forceFault();

            /**
             * @brief clearFault()
             *  - Clears the fault condition if it's safe to do so.
             */
            void clearFault();

            bool HomingStatus();

        private:
            // ----------------------------------
            // Internal helpers
            // ----------------------------------

            /**
             * @brief checkJointLimits()
             *  - Reads the jointBuffer from rtLayout_ to get current positions,
             *    compares them with joint min/max from the paramServer, sets fault if out of range.
             */
            void checkJointLimits();

            /**
             * @brief checkTorqueOrKinematicLimits()
             *  - Optionally use HapticDeviceKinematics / HapticDeviceDynamics to check if 
             *    torques or positions violate any limit. E.g., inverse dynamics > max torque limit,
             *    or kinematic singularity detection, etc.
             */
            void checkTorqueOrKinematicLimits();

        private:
            const hand_control::merai::ParameterServer* paramServerPtr_ = nullptr;
            hand_control::merai::RTMemoryLayout*        rtLayout_       = nullptr;

            // Reference to your haptic device model
            const hand_control::robotics::haptic_device::HapticDeviceModel& model_;

            // Local kinematics/dynamics if needed
            hand_control::robotics::haptic_device::HapticDeviceKinematics kinematics_;
            hand_control::robotics::haptic_device::HapticDeviceDynamics   dynamics_;

            // Basic fault flag
            bool faulted_ = false;

            // Example: number of drives / joints
            std::size_t driveCount_ = 0;
            static constexpr int MAX_JOINTS = 6;

            // Example min/max joint angle arrays
            double jointMin_[MAX_JOINTS];
            double jointMax_[MAX_JOINTS];

            // Possibly other threshold arrays: torqueLimit_, velocityLimit_, etc.
        };

    } // namespace logic
} // namespace hand_control
