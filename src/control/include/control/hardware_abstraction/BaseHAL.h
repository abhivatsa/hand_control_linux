#pragma once

#include <cstddef>  // for size_t
#include "merai/RTMemoryLayout.h" // For definitions like JointControlCommand, JointMotionCommand, etc.

namespace seven_axis_robot
{
    namespace control
    {
        /**
         * @brief The BaseHAL interface defines the essential contract
         *        for initializing, reading from, and writing commands to hardware (or simulated hardware).
         *
         * In the new architecture, we have separate data structures for:
         *   - Control-level commands/feedback: JointControlCommand / JointControlFeedback
         *   - Motion-level commands/feedback:  JointMotionCommand / JointMotionFeedback
         *   - IO-level commands/feedback:      JointCommandIO / JointFeedbackIO
         *
         * Each derived HAL (RealHAL, SimHAL) will provide pointers to these arrays,
         * which the rest of the system can use for reading/writing data in real-time.
         */
        class BaseHAL
        {
        public:
            virtual ~BaseHAL() = default;

            // --------------------------------------------------
            // Lifecycle
            // --------------------------------------------------

            /**
             * @brief Initialize the hardware or simulation layer.
             * @return True on success, false on error.
             */
            virtual bool init() = 0;

            /**
             * @brief Read fresh data from hardware or simulation into local buffers.
             * @return True on success, false on error.
             */
            virtual bool read() = 0;

            /**
             * @brief Write the current joint commands (control or motion) out to hardware or simulation.
             * @return True on success, false on error.
             */
            virtual bool write() = 0;

            // --------------------------------------------------
            // Joint Control (CiA-402) Access
            // --------------------------------------------------

            /**
             * @brief Provides access to the array of JointControlCommand (controlWord, etc.)
             * @return Pointer to the first element of an array sized getDriveCount().
             */
            virtual seven_axis_robot::merai::JointControlCommand* getJointControlCommandPtr() = 0;

            /**
             * @brief Provides access to the array of JointControlFeedback (statusWord, etc.)
             * @return Pointer to the first element of an array sized getDriveCount().
             */
            virtual seven_axis_robot::merai::JointControlFeedback* getJointControlFeedbackPtr() = 0;

            // --------------------------------------------------
            // Joint Motion Access
            // --------------------------------------------------

            /**
             * @brief Provides access to the array of JointMotionCommand (position, torque, mode).
             * @return Pointer to the first element of an array sized getDriveCount().
             */
            virtual seven_axis_robot::merai::JointMotionCommand* getJointMotionCommandPtr() = 0;

            /**
             * @brief Provides access to the array of JointMotionFeedback (positionActual, velocityActual, torqueActual).
             * @return Pointer to the first element of an array sized getDriveCount().
             */
            virtual seven_axis_robot::merai::JointMotionFeedback* getJointMotionFeedbackPtr() = 0;

            // --------------------------------------------------
            // Joint IO Access (optional)
            // --------------------------------------------------

            /**
             * @brief Accessor for the array of JointFeedbackIO (digital/analog inputs, etc.).
             * @return Pointer to an array sized getDriveCount(), or nullptr if not used.
             */
            virtual seven_axis_robot::merai::JointFeedbackIO* getJointFeedbackIOPtr() = 0;

            /**
             * @brief Accessor for the array of JointCommandIO (digital/analog outputs, etc.).
             * @return Pointer to an array sized getDriveCount(), or nullptr if not used.
             */
            virtual seven_axis_robot::merai::JointCommandIO* getJointCommandIOPtr() = 0;

            // --------------------------------------------------
            // Metadata
            // --------------------------------------------------

            /**
             * @brief Returns the number of drives (or joints) this hardware layer manages.
             * @return The drive/joint count.
             */
            virtual size_t getDriveCount() const = 0;
        };

    } // namespace control
} // namespace seven_axis_robot
