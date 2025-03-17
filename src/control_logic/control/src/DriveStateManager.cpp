#include <iostream>
#include "control/DriveStateManager.h"

namespace hand_control
{
    namespace control
    {
        // Control word definitions
        static constexpr uint16_t CW_FAULT_RESET       = 0x0080;
        static constexpr uint16_t CW_SHUTDOWN          = 0x0006;
        static constexpr uint16_t CW_SWITCH_ON         = 0x0007;
        static constexpr uint16_t CW_ENABLE_OPERATION  = 0x000F;
        static constexpr uint16_t CW_QUICK_STOP        = 0x000B;
        static constexpr uint16_t CW_DISABLE_VOLTAGE   = 0x0000;

        DriveStateManager::DriveStateManager(
            std::array<hand_control::merai::ServoRxControl, hand_control::merai::MAX_SERVO_DRIVES>& driveOutputControl,
            std::array<hand_control::merai::ServoTxControl, hand_control::merai::MAX_SERVO_DRIVES>& driveInputControl,
            std::size_t driveCount)
            : driveOutputControl_(driveOutputControl),
              driveInputControl_(driveInputControl),
              driveCount_(driveCount)
        {
        }

        bool DriveStateManager::init()
        {
            // Initialization logic, if any, can be added here.
            return true;
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
            {
                return hand_control::merai::DriveStatus::FAULT;
            }
            if (switchOnDisabled)
            {
                return hand_control::merai::DriveStatus::SWITCH_ON_DISABLED;
            }
            if (!readyToSwitchOn && !switchedOn && !opEnabled)
            {
                return hand_control::merai::DriveStatus::NOT_READY_TO_SWITCH_ON;
            }
            if (readyToSwitchOn && !switchedOn && !opEnabled)
            {
                return hand_control::merai::DriveStatus::READY_TO_SWITCH_ON;
            }
            if (readyToSwitchOn && switchedOn && !opEnabled)
            {
                return hand_control::merai::DriveStatus::SWITCHED_ON;
            }
            if (readyToSwitchOn && switchedOn && opEnabled)
            {
                if (quickStop)
                {
                    return hand_control::merai::DriveStatus::QUICK_STOP;
                }
                else
                {
                    return hand_control::merai::DriveStatus::OPERATION_ENABLED;
                }
            }
            return hand_control::merai::DriveStatus::NOT_READY_TO_SWITCH_ON;
        }

        void DriveStateManager::update(const hand_control::merai::DriveCommand* driveCommands,
                                       hand_control::merai::DriveStatus* driveStatus)
        {
            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                // Directly decode the status word into a DriveStatus.
                uint16_t sw = driveInputControl_[i].statusWord;
                driveStatus[i] = decodeStatusword(sw);

                // Decide what control word to send based on the decoded status and the drive command.
                hand_control::merai::DriveCommand cmd = driveCommands[i];
                uint16_t controlWord = CW_DISABLE_VOLTAGE; // default

                switch (driveStatus[i])
                {
                    case hand_control::merai::DriveStatus::FAULT:
                        if (cmd == hand_control::merai::DriveCommand::FAULT_RESET)
                        {
                            controlWord = CW_FAULT_RESET;
                        }
                        else
                        {
                            controlWord = CW_DISABLE_VOLTAGE;
                        }
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
                        {
                            controlWord = CW_ENABLE_OPERATION;
                        }
                        else
                        {
                            controlWord = CW_SWITCH_ON;
                        }
                        break;

                    case hand_control::merai::DriveStatus::OPERATION_ENABLED:
                        if (cmd == hand_control::merai::DriveCommand::ALLOW_OPERATION)
                        {
                            controlWord = CW_ENABLE_OPERATION;
                        }
                        else if (cmd == hand_control::merai::DriveCommand::FORCE_DISABLE)
                        {
                            controlWord = CW_SWITCH_ON;
                        }
                        else
                        {
                            controlWord = CW_ENABLE_OPERATION;
                        }
                        break;

                    case hand_control::merai::DriveStatus::QUICK_STOP:
                        if (cmd == hand_control::merai::DriveCommand::ALLOW_OPERATION)
                        {
                            controlWord = CW_SWITCH_ON;
                        }
                        else
                        {
                            controlWord = CW_QUICK_STOP;
                        }
                        break;

                    default:
                        controlWord = CW_DISABLE_VOLTAGE;
                        break;
                }

                driveOutputControl_[i].controlWord = controlWord;
            }
        }
    } // namespace control
} // namespace hand_control
