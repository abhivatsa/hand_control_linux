#pragma once

#include <cstddef>  // for size_t
#include "merai/RTMemoryLayout.h"   // hand_control::merai::JointState, JointCommand, IoState, IoCommand

namespace hand_control
{
    namespace control
    {
        /**
         * @brief The BaseHAL interface defines a generic contract
         *        for initializing, reading from, and writing commands to hardware (or simulated hardware).
         */
        class BaseHAL
        {
        public:
            virtual ~BaseHAL() = default;

            /**
             * @brief Initialize hardware or simulation layer.
             * @return True on success, false on error.
             */
            virtual bool init() = 0;

            /**
             * @brief Read fresh data from hardware or simulation into local buffers.
             * @return True on success, false on error.
             */
            virtual bool read() = 0;

            /**
             * @brief Write current commands (joint or drive) out to hardware or simulation.
             * @return True on success, false on error.
             */
            virtual bool write() = 0;

            /**
             * @brief Accessor for the local array of joint states in SI units.
             * @return Pointer to array of JointState structures.
             */
            virtual hand_control::merai::JointState* getJointStatesPtr() = 0;

            /**
             * @brief Accessor for the local array of joint commands in SI units.
             * @return Pointer to array of JointCommand structures.
             */
            virtual hand_control::merai::JointCommand* getJointCommandsPtr() = 0;

            /**
             * @brief Get the number of joints or actuators this hardware layer manages.
             * @return The count of joints.
             */
            virtual size_t getJointCount() const = 0;

            // --------------------------------------------------
            // Drive data: Simulated or Real Drive I/O
            // --------------------------------------------------

            /**
             * @brief Accessor for the local array of drive input controls.
             * @return Pointer to the array of drive input control data.
             */
            virtual hand_control::merai::ServoTxControl* getDriveInputControlPtr() = 0;

            /**
             * @brief Accessor for the local array of drive output controls.
             * @return Pointer to the array of drive output control data.
             */
            virtual hand_control::merai::ServoRxControl* getDriveOutputControlPtr() = 0;

            /**
             * @brief Get the number of drives the hardware layer manages.
             * @return The count of drives.
             */
            virtual size_t getDriveCount() const = 0;

            // --------------------------------------------------
            // Joint I/O access
            // --------------------------------------------------

            /**
             * @brief Accessor for the local array of joint I/O.
             * @return Pointer to array of JointIO structures.
             */
            virtual hand_control::merai::JointIO* getJointIOPtr() = 0;
        };
    } // namespace control
} // namespace hand_control
