#include <cmath>

#include "control/hardware_abstraction/SimHAL.h"
#include "merai/RTIpc.h"

namespace control
{
    SimHAL::SimHAL(
        merai::RTMemoryLayout *rtLayout,
        const merai::ParameterServer *paramServerPtr,
        merai::multi_ring_logger_memory *loggerMem)
        : rtLayout_(rtLayout),
          paramServerPtr_(paramServerPtr),
          loggerMem_(loggerMem)
    {
        if (!rtLayout_)
        {
            merai::log_warn(loggerMem_, "Control", 2300, "[SimHAL] Warning: Null rtLayout");
        }
        if (!paramServerPtr_)
        {
            merai::log_warn(loggerMem_, "Control", 2301, "[SimHAL] Warning: Null ParameterServer");
        }
    }

    bool SimHAL::init()
    {
        // 1) Determine how many drives from paramServerPtr_
        if (paramServerPtr_)
        {
            driveCount_ = paramServerPtr_->driveCount;
        }
        // Cap at MAX_SERVO_DRIVES
        if (driveCount_ > merai::MAX_SERVO_DRIVES)
        {
            driveCount_ = merai::MAX_SERVO_DRIVES;
        }

        // 2) Clear local arrays
        for (int i = 0; i < driveCount_; ++i)
        {
            // Control commands / feedback
            localJointControlCommand_[i] = {};
            localJointControlFeedback_[i] = {};

            // Motion commands / feedback
            localJointMotionCommand_[i] = {};
            localJointMotionFeedback_[i] = {};

            // I/O
            localJointFeedbackIO_[i] = {};
        }

        merai::log_info(loggerMem_, "Control", 2302, "[SimHAL] init() complete");
        return true;
    }

    bool SimHAL::read()
    {
        // In a real system, you'd fetch servo Tx data from shared memory here.
        // In simulation, we can just generate or update these feedback values as needed.
        simulateDriveStateTransitions();
        simulateJointIOChanges();

        return true;
    }

    bool SimHAL::write()
    {
        // In a real system, you'd convert local joint commands to servo Rx data.
        // In simulation, we simply note that new commands have arrived, and possibly
        // use them to update the next read cycle's feedback.

        // For instance, if you wanted to simulate motion:
        //   - read localJointMotionCommand_ (like targetPosition)
        //   - update localJointMotionFeedback_ to reflect a new position each cycle
        //   - or simply store the commands for future read() usage

        return true;
    }

    void SimHAL::simulateDriveStateTransitions()
    {
        // Example: If a control word is set in localJointControlCommand_, update localJointControlFeedback_.
        for (int i = 0; i < driveCount_; ++i)
        {
            // If controlWord is non-zero, we might say it's "enabled"
            if (localJointControlCommand_[i].controlWord != 0)
            {
                // e.g., set statusWord to some "enabled" bit
                localJointControlFeedback_[i].statusWord = 0x0001;
            }
            else
            {
                localJointControlFeedback_[i].statusWord = 0x0000;
            }

            // You could also interpret "modeOfOperation" from localJointMotionCommand_ and simulate
            // different behaviors in localJointMotionFeedback_.
        }
    }

    void SimHAL::simulateJointIOChanges()
    {
        // Example: Toggle digital input states or simulate analog input changes.
        for (int i = 0; i < driveCount_; ++i)
        {
            // For demonstration, just alternate the digitalInputClutch flag each cycle
            localJointFeedbackIO_[i].digitalInputClutch =
                !localJointFeedbackIO_[i].digitalInputClutch;

            // Simulate an analog input with a sine wave (for demonstration)
            double angle = static_cast<double>(i) * 0.5; // or use a time-based increment
            localJointFeedbackIO_[i].analogInputPinch = std::sin(angle);

            // Possibly read localJointMotionCommand_[i] and do something with it
            // to produce a feedback effect in localJointMotionFeedback_[i].
        }
    }

    bool SimHAL::publishJointFeedbackToShm()
    {
        if (!rtLayout_)
        {
            return false;
        }

        int backIdx = merai::back_index(rtLayout_->jointFeedbackBuffer);
        auto& feedbackArray = rtLayout_->jointFeedbackBuffer.buffer[backIdx];
        feedbackArray.fill(merai::JointFeedbackData{});

        for (int i = 0; i < driveCount_; ++i)
        {
            feedbackArray[i].control = localJointControlFeedback_[i];
            feedbackArray[i].motion = localJointMotionFeedback_[i];
            feedbackArray[i].io = localJointFeedbackIO_[i];
        }

        merai::publish(rtLayout_->jointFeedbackBuffer, backIdx);
        return true;
    }

} // namespace control
