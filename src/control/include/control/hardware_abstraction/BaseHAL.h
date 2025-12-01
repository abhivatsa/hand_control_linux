#pragma once

#include <cstddef>
#include <span>

#include "merai/RTMemoryLayout.h"

namespace control
{
    /**
     * @brief The BaseHAL interface defines the contract for
     *        initializing, reading, and writing hardware (or simulation).
     *
     * Provides pointer-based and span-based accessors for:
     *   - JointControlCommand / JointControlFeedback
     *   - JointMotionCommand  / JointMotionFeedback
     *   - JointCommandIO      / JointFeedbackIO
     */
    class BaseHAL
    {
    public:
        virtual ~BaseHAL() = default;

        // Lifecycle
        virtual bool init()  = 0;
        virtual bool read()  = 0;
        virtual bool write() = 0;

        // Pointer accessors (legacy / direct access)
        virtual merai::JointControlCommand  *getJointControlCommandPtr()   = 0;
        virtual merai::JointControlFeedback *getJointControlFeedbackPtr()  = 0;

        virtual merai::JointMotionCommand   *getJointMotionCommandPtr()    = 0;
        virtual merai::JointMotionFeedback  *getJointMotionFeedbackPtr()   = 0;

        virtual merai::JointFeedbackIO      *getJointFeedbackIOPtr()       = 0;
        virtual merai::JointCommandIO       *getJointCommandIOPtr()        = 0;

        // Metadata
        virtual std::size_t getDriveCount() const = 0;

        // Span-based accessors for pipeline
        virtual std::span<const merai::JointControlFeedback> jointControlFeedback() const = 0;
        virtual std::span<const merai::JointMotionFeedback>  jointMotionFeedback()  const = 0;
        virtual std::span<const merai::JointFeedbackIO>      jointIOFeedback()      const = 0;

        virtual std::span<merai::JointControlCommand> jointControlCommand() = 0;
        virtual std::span<merai::JointMotionCommand>  jointMotionCommand()  = 0;

        // Publish joint feedback into RT SHM
        virtual bool publishJointFeedbackToShm() = 0;
    };

} // namespace control
