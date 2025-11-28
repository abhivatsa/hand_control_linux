#pragma once

#include <cstddef> // for size_t
#include <cstdint>

// Shared memory structures
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/Enums.h"
#include "robotics_lib/RobotModel.h"

// The data structures from RTMemoryLayout
// e.g., DriveFeedbackData, UserCommands, ControllerFeedback
// which hold status arrays, eStop flags, etc.

    namespace logic
    {
        /**
         * @brief A manager that checks system safety conditions each cycle (eStop, joint limits,
         *        motion constraints, etc.). If a fault occurs, sets an internal flag.
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
             * @param model           Reference to the robot model (kinematics/dynamics).
             */
            SafetyManager(const merai::ParameterServer* paramServerPtr,
                          merai::RTMemoryLayout*        rtLayout,
                          const robot_lib::RobotModel&  model);

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
             * @param driveFdbk   The drive feedback aggregator data (drive status).
             * @param userCmds    The user commands aggregator data (eStop, resetFault, etc.).
             * @param ctrlFdbk    The controller feedback aggregator data (controller state).
             * @return bool       True if the system is currently faulted, false otherwise.
             */
            bool update(const merai::DriveFeedbackData& driveFdbk,
                        const merai::UserCommands&      userCmds,
                        const merai::ControllerFeedback& ctrlFdbk);

            /**
             * @brief isFaulted()
             *  - Returns whether the manager has detected a fault condition.
             */
            bool isFaulted() const;

            bool isHomingCompleted();

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

        private:
            // ----------------------------------
            // Internal helpers
            // ----------------------------------

            /**
             * @brief checkJointLimits()
             *  - Reads jointFeedbackBuffer from rtLayout_ to get current positions,
             *    compares them with joint min/max from the paramServer, sets fault if out of range.
             */
            void checkJointLimits();

            /**
             * @brief checkTorqueOrKinematicLimits()
             *  - Optionally use kinematics_ / dynamics_ to check if torques or positions 
             *    violate any limit. E.g., inverse dynamics > max torque limit,
             *    or kinematic singularity detection, etc.
             */
            void checkTorqueOrKinematicLimits();

        private:
            const merai::ParameterServer* paramServerPtr_ = nullptr;
            merai::RTMemoryLayout*        rtLayout_       = nullptr;

            // Robot model reference (for future kinematic/dynamic checks)
            const robot_lib::RobotModel&  model_;

            // Basic fault flag
            bool faulted_ = false;

            // Example: number of drives / joints
            std::size_t driveCount_ = 0;
            static constexpr int MAX_JOINTS = 7;

            // Example min/max joint angle arrays
            double jointMin_[MAX_JOINTS];
            double jointMax_[MAX_JOINTS];
        };

    } // namespace logic
