#include <cmath>

#include "control/hardware_abstraction/SimHAL.h"
#include "merai/RTIpc.h"

namespace control
{
    SimHAL::SimHAL(merai::RTMemoryLayout           *rtLayout,
                   const merai::ParameterServer    *paramServerPtr,
                   merai::multi_ring_logger_memory *loggerMem)
        : rtLayout_(rtLayout),
          paramServerPtr_(paramServerPtr),
          loggerMem_(loggerMem)
    {
        if (!rtLayout_)
        {
            merai::log_warn(loggerMem_, "Control", 2300,
                            "[SimHAL] Warning: Null rtLayout");
        }
        if (!paramServerPtr_)
        {
            merai::log_warn(loggerMem_, "Control", 2301,
                            "[SimHAL] Warning: Null ParameterServer");
        }
    }

    bool SimHAL::init()
    {
        // 1) Determine drive count from ParameterServer
        if (paramServerPtr_)
        {
            driveCount_ = paramServerPtr_->driveCount;
        }

        if (driveCount_ > merai::MAX_SERVO_DRIVES)
        {
            driveCount_ = merai::MAX_SERVO_DRIVES;
        }

        // 2) Clear local arrays
        for (int i = 0; i < driveCount_; ++i)
        {
            localJointControlCommand_[i]   = {};
            localJointControlFeedback_[i]  = {};
            localJointMotionCommand_[i]    = {};
            localJointMotionFeedback_[i]   = {};
            localJointFeedbackIO_[i]       = {};
            localJointCommandIO_[i]        = {};
        }

        merai::log_info(loggerMem_, "Control", 2302,
                        "[SimHAL] init() complete");
        return true;
    }

    bool SimHAL::read()
    {
        // No fieldbus here: just simulate based on last commands.
        simulateDriveStateTransitions();
        simulateJointIOChanges();

        // If you want, you could also integrate motion here:
        // - use localJointMotionCommand_ to update localJointMotionFeedback_
        //   with a simple kinematic model at 1 kHz.

        return true;
    }

    bool SimHAL::write()
    {
        // In sim mode we don’t need to touch shared servo PDOs at all.
        // Control writes into localJoint*Command_ via spans, and read()
        // uses those to update feedback. Nothing else required here.
        return true;
    }

    void SimHAL::simulateDriveStateTransitions()
    {
        // Simple: if controlWord != 0, treat drive as "enabled-ish"
        for (int i = 0; i < driveCount_; ++i)
        {
            if (localJointControlCommand_[i].controlWord != 0)
            {
                localJointControlFeedback_[i].statusWord = 0x0001; // READY_TO_SWITCH_ON-ish
            }
            else
            {
                localJointControlFeedback_[i].statusWord = 0x0000;
            }

            // Optional: basic motion simulation (example: position follows target)
            // Here we just keep motionFeedback as-is, but you could do:
            //
            // double alpha = 0.05; // simple low-pass
            // localJointMotionFeedback_[i].positionActual =
            //     (1.0 - alpha) * localJointMotionFeedback_[i].positionActual +
            //     alpha * localJointMotionCommand_[i].targetPosition;
            //
            // etc.
        }
    }

    void SimHAL::simulateJointIOChanges()
    {
        // Simple fake I/O: toggle a bit and wiggle an analog value
        for (int i = 0; i < driveCount_; ++i)
        {
            // Flip clutch input each cycle
            localJointFeedbackIO_[i].digitalInputClutch =
                !localJointFeedbackIO_[i].digitalInputClutch;

            // Thumb stays false for now
            localJointFeedbackIO_[i].digitalInputThumb = false;

            // Fake analog pinch sensor
            double angle = static_cast<double>(i) * 0.5;
            localJointFeedbackIO_[i].analogInputPinch = std::sin(angle);
        }
    }

    bool SimHAL::publishJointFeedbackToShm()
    {
        if (!rtLayout_)
        {
            return false;
        }

        int backIdx = merai::back_index(rtLayout_->jointFeedbackBuffer);
        auto &feedbackArray = rtLayout_->jointFeedbackBuffer.buffer[backIdx];

        feedbackArray.fill(merai::JointFeedbackData{});

        for (int i = 0; i < driveCount_; ++i)
        {
            feedbackArray[i].control = localJointControlFeedback_[i];
            feedbackArray[i].motion  = localJointMotionFeedback_[i];
            feedbackArray[i].io      = localJointFeedbackIO_[i];
        }

        merai::publish(rtLayout_->jointFeedbackBuffer, backIdx);
        return true;
    }

} // namespace control
