#pragma once

#include <cstddef>
#include <cstdint>

// control/hardware_abstraction/DriveData.h defines motion_control::control::DriveInput, DriveOutput
#include "control/hardware_abstraction/DriveData.h"

// merai/RTMemoryLayout.h (or a similar header) for motion_control::merai::DriveUserSignals, DriveFeedback
#include "merai/RTMemoryLayout.h"

namespace motion_control
{
    namespace control
    {
        /**
         * @brief Enumerated states used internally by the manager
         *        to interpret statusWord (CiA 402).
         */
        enum class DriveState
        {
            Fault,
            SwitchOnDisabled,
            NotReadyToSwitchOn,
            ReadyToSwitchOn,
            SwitchedOn,
            OperationEnabled,
            QuickStopActive,
            FaultReactionActive,
            Unknown
        };

        /**
         * @brief The DriveStateManager handles CiA 402 transitions
         *        by decoding the statusWord and writing controlWord.
         *        It also sets minimal feedback bits for logic to read.
         */
        class DriveStateManager
        {
        public:
            DriveStateManager() = default;

            bool init();

            /**
             * @brief update
             *   - Reads driveInputs[i].statusWord -> decode
             *   - Reads userSignals[i] for faultReset, allowOperation, etc.
             *   - Writes controlWord to driveOutputs[i]
             *   - Sets feedback bits (faultActive, operationEnabled, etc.)
             *
             * @param driveInputs    Array of DriveInput
             * @param driveOutputs   Array of DriveOutput
             * @param userSignals    Array of DriveUserSignals
             * @param feedback       Array of DriveFeedback for logic to read
             * @param driveCount     Number of drives
             */
            void update(const motion_control::control::DriveInput* driveInputs,
                        motion_control::control::DriveOutput* driveOutputs,
                        const motion_control::merai::DriveUserSignals* userSignals,
                        motion_control::merai::DriveFeedback* feedback,
                        std::size_t driveCount);

        private:
            /**
             * @brief decodeStatusword interprets the bits of statusWord to find the drive's CiA 402 state.
             */
            DriveState decodeStatusword(uint16_t statusWord);
        };
    } // namespace control
} // namespace motion_control
