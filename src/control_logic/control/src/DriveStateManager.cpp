#include <iostream>
#include "control/DriveStateManager.h"

// For convenience, bring in the CiA-402 control word definitions
namespace
{
    // Control word definitions
    constexpr uint16_t CW_FAULT_RESET       = 0x0080;
    constexpr uint16_t CW_SHUTDOWN          = 0x0006;
    constexpr uint16_t CW_SWITCH_ON         = 0x0007;
    constexpr uint16_t CW_ENABLE_OPERATION  = 0x000F;
    constexpr uint16_t CW_QUICK_STOP        = 0x000B;
    constexpr uint16_t CW_DISABLE_VOLTAGE   = 0x0000;
}

namespace hand_control
{
    namespace control
    {
        DriveStateManager::DriveStateManager(
            hand_control::merai::ServoRxControl* driveOutputControlPtr,
            hand_control::merai::ServoTxControl* driveInputControlPtr,
            std::size_t driveCount)
            : driveOutputControlPtr_(driveOutputControlPtr),
              driveInputControlPtr_(driveInputControlPtr),
              driveCount_(driveCount)
        {
        }

        bool DriveStateManager::init()
        {
            // Initialization logic, if any
            // e.g., check that pointers are not null, set defaults, etc.
            return (driveOutputControlPtr_ && driveInputControlPtr_);
        }

        hand_control::merai::DriveStatus DriveStateManager::decodeStatusword(uint16_t statusWord)
        {
            // Based on CiA 402 bit definitions:
            bool fault            = (statusWord & 0x0008) != 0;  // bit 3
            bool switchOnDisabled = (statusWord & 0x0040) != 0;  // bit 6
            bool readyToSwitchOn  = (statusWord & 0x0001) != 0;  // bit 0
            bool switchedOn       = (statusWord & 0x0002) != 0;  // bit 1
            bool opEnabled        = (statusWord & 0x0004) != 0;  // bit 2
            bool quickStop        = (statusWord & 0x0020) != 0;  // bit 5

            if (fault)
                return hand_control::merai::DriveStatus::FAULT;
            if (switchOnDisabled)
                return hand_control::merai::DriveStatus::SWITCH_ON_DISABLED;
            if (!readyToSwitchOn && !switchedOn && !opEnabled)
                return hand_control::merai::DriveStatus::NOT_READY_TO_SWITCH_ON;
            if (readyToSwitchOn && !switchedOn && !opEnabled)
                return hand_control::merai::DriveStatus::READY_TO_SWITCH_ON;
            if (readyToSwitchOn && switchedOn && !opEnabled)
                return hand_control::merai::DriveStatus::SWITCHED_ON;
            if (readyToSwitchOn && switchedOn && opEnabled)
            {
                if (quickStop)
                    return hand_control::merai::DriveStatus::QUICK_STOP;
                else
                    return hand_control::merai::DriveStatus::OPERATION_ENABLED;
            }
            return hand_control::merai::DriveStatus::NOT_READY_TO_SWITCH_ON;
        }

        void DriveStateManager::update(const hand_control::merai::DriveCommand* driveCommands,
                                       hand_control::merai::DriveStatus* driveStatus)
        {
            if (!driveOutputControlPtr_ || !driveInputControlPtr_ || !driveCommands || !driveStatus)
            {
                // In production code, handle null pointers (log error, etc.)
                return;
            }

            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                // Decode the status word for each drive
                uint16_t sw = driveInputControlPtr_[i].statusWord;
                driveStatus[i] = decodeStatusword(sw);

                // Decide what control word to send based on the drive status + command
                hand_control::merai::DriveCommand cmd = driveCommands[i];
                uint16_t controlWord = CW_DISABLE_VOLTAGE; // default

                switch (driveStatus[i])
                {
                    case hand_control::merai::DriveStatus::FAULT:
                        if (cmd == hand_control::merai::DriveCommand::FAULT_RESET)
                            controlWord = CW_FAULT_RESET;
                        else
                            controlWord = CW_DISABLE_VOLTAGE;
                        break;

                    case hand_control::merai::DriveStatus::SWITCH_ON_DISABLED:
                        controlWord = CW_SHUTDOWN;
                        break;

                    case hand_control::merai::DriveStatus::NOT_READY_TO_SWITCH_ON:
                        controlWord = CW_DISABLE_VOLTAGE;
                        break;

                    case hand_control::merai::DriveStatus::READY_TO_SWITCH_ON:
                        controlWord = CW_SWITCH_ON;
                        break;

                    case hand_control::merai::DriveStatus::SWITCHED_ON:
                        if (cmd == hand_control::merai::DriveCommand::ALLOW_OPERATION)
                            controlWord = CW_ENABLE_OPERATION;
                        else
                            controlWord = CW_SWITCH_ON;
                        break;

                    case hand_control::merai::DriveStatus::OPERATION_ENABLED:
                        if (cmd == hand_control::merai::DriveCommand::ALLOW_OPERATION)
                            controlWord = CW_ENABLE_OPERATION;
                        else if (cmd == hand_control::merai::DriveCommand::FORCE_DISABLE)
                            controlWord = CW_SWITCH_ON;
                        else
                            controlWord = CW_ENABLE_OPERATION;
                        break;

                    case hand_control::merai::DriveStatus::QUICK_STOP:
                        if (cmd == hand_control::merai::DriveCommand::ALLOW_OPERATION)
                            controlWord = CW_SWITCH_ON;
                        else
                            controlWord = CW_QUICK_STOP;
                        break;

                    default:
                        controlWord = CW_DISABLE_VOLTAGE;
                        break;
                }

                // Write controlWord to the corresponding output control
                driveOutputControlPtr_[i].controlWord = controlWord;
            }
        }

    } // namespace control
} // namespace hand_control
