#pragma once

#include <cstddef>  // size_t
#include <cstdint>

#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/Enums.h"
#include "robotics_lib/RobotModel.h"

namespace logic
{
    /**
     * @brief Checks safety conditions each logic cycle:
     *  - user e-stop
     *  - drive FAULT states
     *  - (optional) joint limits / torque / kinematics
     *
     * It maintains a latched fault flag. Once a fault is latched, it will stay
     * active until resetFault is requested *and* it is safe to clear.
     */
    class SafetyManager
    {
    public:
        SafetyManager(const merai::ParameterServer* paramServerPtr,
                      merai::RTMemoryLayout*        rtLayout,
                      const robot_lib::RobotModel&  model);

        ~SafetyManager() = default;

        /**
         * @brief Initialize safety thresholds from ParameterServer.
         *
         * Reads joint limits, clamps drive count, and clears the latched fault.
         */
        bool init();

        /**
         * @brief Evaluate safety for this cycle.
         *
         * - Latches a fault if:
         *    * userCmds.eStop is true
         *    * any drive reports DriveStatus::FAULT
         *    * (future) joint/torque/kinematic violations
         *
         * - If a fault is already latched, only clears it if:
         *    * userCmds.resetFault is true
         *    * e-stop is *not* pressed
         *    * no drive is currently in DriveStatus::FAULT
         *
         * @return true if system is currently faulted (latched), false otherwise.
         */
        bool update(const merai::DriveFeedbackData&  driveFdbk,
                    const merai::UserCommands&       userCmds,
                    const merai::ControllerFeedback& ctrlFdbk);

        /// Returns whether a fault is currently latched.
        bool isFaulted() const { return faulted_; }

        /// Manually latch a fault (e.g. from other modules).
        void forceFault() { faulted_ = true; }

        /// Clears the latched fault unconditionally (internal use).
        void clearFault() { faulted_ = false; }

    private:
        // Joint position limit check (optional, can be enabled when needed).
        void checkJointLimits();

        // Placeholder for more advanced torque/kinematic checks.
        void checkTorqueOrKinematicLimits();

    private:
        const merai::ParameterServer* paramServerPtr_ = nullptr;
        merai::RTMemoryLayout*        rtLayout_       = nullptr;

        // Robot model reference (for future kinematic/dynamic checks).
        const robot_lib::RobotModel&  model_;

        // Latched fault flag.
        bool faulted_ = false;

        // Number of drives / joints actually in use.
        std::size_t driveCount_ = 0;
        static constexpr int MAX_JOINTS = 7;

        // Joint limits [rad].
        double jointMin_[MAX_JOINTS]{};
        double jointMax_[MAX_JOINTS]{};
    };

} // namespace logic
