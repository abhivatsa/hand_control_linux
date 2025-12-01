#pragma once

#include <span>
#include <chrono>
#include <cstdint>

#include "merai/RTMemoryLayout.h"

namespace control
{
    using TimePoint = std::chrono::steady_clock::time_point;

    struct ControlCycleInputs
    {
        // From hardware (via HAL::read())
        std::span<const merai::JointControlFeedback> jointControlFbk;
        std::span<const merai::JointMotionFeedback>  jointMotionFbk;
        std::span<const merai::JointFeedbackIO>      jointIoFbk;

        // From higher-level logic (shared memory snapshots)
        merai::DriveCommandData   driveCmd;
        merai::ControllerCommand  ctrlCmd;
    };

    struct ControlCycleOutputs
    {
        // Commands that will be sent to hardware (consumed by HAL::write())
        std::span<merai::JointControlCommand> jointControlCmd;
        std::span<merai::JointMotionCommand>  jointMotionCmd;

        // Feedback that will be written back to shared memory
        merai::DriveFeedbackData  driveFbk;
        merai::ControllerFeedback ctrlFbk;
    };

} // namespace control
