#include <iostream> 
#include <stdexcept>
#include "control/DriveStateManager.h"

namespace hand_control
{
    namespace control
    {
        static constexpr uint16_t CW_FAULT_RESET       = 0x0080;
        static constexpr uint16_t CW_SHUTDOWN         = 0x0006;
        static constexpr uint16_t CW_SWITCH_ON        = 0x0007;
        static constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
        static constexpr uint16_t CW_QUICK_STOP       = 0x000B;
        static constexpr uint16_t CW_DISABLE_VOLTAGE  = 0x0000;

        bool DriveStateManager::init()
        {
            // If you have any initialization logic, place it here.
            return true;
        }

        DriveState DriveStateManager::decodeStatusword(uint16_t statusWord)
        {
            // (Example logic for reference)
            bool fault           = (statusWord & 0x0008) != 0;  // bit 3
            bool switchOnDisabled= (statusWord & 0x0040) != 0;  // bit 6
            bool readyToSwitchOn = (statusWord & 0x0001) != 0;  // bit 0
            bool switchedOn      = (statusWord & 0x0002) != 0;  // bit 1
            bool opEnabled       = (statusWord & 0x0004) != 0;  // bit 2
            bool quickStop       = (statusWord & 0x0020) != 0;  // bit 5

            if (fault)
            {
                return DriveState::Fault;
            }
            if (switchOnDisabled)
            {
                return DriveState::SwitchOnDisabled;
            }
            if (!readyToSwitchOn && !switchedOn && !opEnabled)
            {
                return DriveState::NotReadyToSwitchOn;
            }
            if (readyToSwitchOn && !switchedOn && !opEnabled)
            {
                return DriveState::ReadyToSwitchOn;
            }
            if (readyToSwitchOn && switchedOn && !opEnabled)
            {
                return DriveState::SwitchedOn;
            }
            if (readyToSwitchOn && switchedOn && opEnabled)
            {
                if (quickStop)
                {
                    return DriveState::QuickStopActive;
                }
                else
                {
                    return DriveState::OperationEnabled;
                }
            }
            return DriveState::Unknown;
        }

        void DriveStateManager::update(const hand_control::control::DriveInput* driveInputs,
                                       hand_control::control::DriveOutput* driveOutputs,
                                       const hand_control::merai::DriveControlSignals* controlSignals,
                                       hand_control::merai::DriveFeedback* feedback,
                                       std::size_t driveCount)
        {
            for (std::size_t i = 0; i < driveCount; ++i)
            {
                // Decode to a high-level DriveState
                uint16_t sw = driveInputs[i].statusWord;
                DriveState ds = decodeStatusword(sw);

                // Clear the feedback booleans for this drive
                feedback[i].fault            = false;
                feedback[i].switchOnDisabled = false;
                feedback[i].readyToSwitchOn  = false;
                feedback[i].switchedOn       = false;
                feedback[i].operationEnabled = false;
                feedback[i].quickStop        = false;

                // Fill feedback based on the enumerated state
                switch (ds)
                {
                case DriveState::Fault:
                    feedback[i].fault = true;
                    break;
                case DriveState::SwitchOnDisabled:
                    feedback[i].switchOnDisabled = true;
                    break;
                case DriveState::NotReadyToSwitchOn:
                    // Possibly do nothing here, or set a boolean if you want to track it
                    break;
                case DriveState::ReadyToSwitchOn:
                    feedback[i].readyToSwitchOn = true;
                    break;
                case DriveState::SwitchedOn:
                    feedback[i].switchedOn = true;
                    break;
                case DriveState::OperationEnabled:
                    feedback[i].operationEnabled = true;
                    break;
                case DriveState::QuickStopActive:
                    feedback[i].quickStop = true;
                    break;
                default:
                    // unknown or fault reaction active, do nothing special
                    break;
                }

                // Decide what controlWord to send, based on DriveState + user signals
                const auto& sig = controlSignals[i];
                uint16_t controlWord = CW_DISABLE_VOLTAGE; // default

                switch (ds)
                {
                case DriveState::Fault:
                    if (sig.faultReset)
                    {
                        controlWord = CW_FAULT_RESET;
                    }
                    else
                    {
                        controlWord = CW_DISABLE_VOLTAGE;
                    }
                    break;

                case DriveState::SwitchOnDisabled:
                    controlWord = CW_SHUTDOWN;
                    break;

                case DriveState::NotReadyToSwitchOn:
                    controlWord = CW_DISABLE_VOLTAGE;
                    break;

                case DriveState::ReadyToSwitchOn:
                    controlWord = CW_SWITCH_ON;
                    break;

                case DriveState::SwitchedOn:
                    if (sig.allowOperation)
                    {
                        controlWord = CW_ENABLE_OPERATION;
                    }
                    else
                    {
                        controlWord = CW_SWITCH_ON;
                    }
                    break;

                case DriveState::OperationEnabled:
                    if (sig.quickStop)
                    {
                        controlWord = CW_QUICK_STOP;
                    }
                    else if (!sig.allowOperation || sig.forceDisable)
                    {
                        controlWord = CW_SWITCH_ON;
                    }
                    else
                    {
                        controlWord = CW_ENABLE_OPERATION;
                    }
                    break;

                case DriveState::QuickStopActive:
                    if (!sig.quickStop)
                    {
                        controlWord = CW_SWITCH_ON;
                    }
                    else
                    {
                        controlWord = CW_QUICK_STOP;
                    }
                    break;

                case DriveState::FaultReactionActive:
                    controlWord = CW_DISABLE_VOLTAGE;
                    break;

                default:
                    controlWord = CW_DISABLE_VOLTAGE;
                    break;
                }

                driveOutputs[i].controlWord = controlWord;
            }
        }

    } // namespace control
} // namespace hand_control
