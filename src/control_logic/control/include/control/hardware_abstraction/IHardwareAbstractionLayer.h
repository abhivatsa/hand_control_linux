#pragma once

#include <cstddef>  // for size_t
#include "merai/RTMemoryLayout.h"   // motion_control::merai::JointState, JointCommand, IoState, IoCommand
#include "control/hardware_abstraction/DriveData.h"  // motion_control::control::DriveInput, DriveOutput

namespace motion_control
{
    namespace control
    {
        /**
         * @brief The IHardwareAbstractionLayer interface defines a generic contract
         *        for initializing, reading from, and writing commands to hardware (or simulated hardware).
         */
        class IHardwareAbstractionLayer
        {
        public:
            virtual ~IHardwareAbstractionLayer() = default;

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
            virtual motion_control::merai::JointState* getJointStatesPtr() = 0;

            /**
             * @brief Accessor for the local array of joint commands in SI units.
             * @return Pointer to array of JointCommand structures.
             */
            virtual motion_control::merai::JointCommand* getJointCommandsPtr() = 0;

            /**
             * @brief Get the number of joints or actuators this hardware layer manages.
             * @return The count of joints.
             */
            virtual size_t getJointCount() const = 0;

            // --------------------------------------------------
            // Optional: I/O data handling methods
            // --------------------------------------------------
            virtual motion_control::merai::IoState* getIoStatesPtr()
            {
                return nullptr;
            }

            virtual motion_control::merai::IoCommand* getIoCommandsPtr()
            {
                return nullptr;
            }

            virtual size_t getIoCount() const
            {
                return 0;
            }

            // --------------------------------------------------
            // Optional: Drive data for CiA 402 logic
            // --------------------------------------------------
            virtual motion_control::control::DriveInput* getDriveInputsPtr()
            {
                return nullptr;
            }

            virtual motion_control::control::DriveOutput* getDriveOutputsPtr()
            {
                return nullptr;
            }

            virtual size_t getDriveCount() const
            {
                return 0;
            }
        };
    } // namespace control
} // namespace motion_control
