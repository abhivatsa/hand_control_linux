#include <iostream>  // Optional debug prints
#include <stdexcept>

#include "control/DriveStateManager.h"

namespace motion_control
{
    namespace control
    {
        static constexpr uint16_t CW_FAULT_RESET       = 0x0080;
        static constexpr uint16_t CW_SHUTDOWN          = 0x0006;
        static constexpr uint16_t CW_SWITCH_ON         = 0x0007;
        static constexpr uint16_t CW_ENABLE_OPERATION  = 0x000F;
        static constexpr uint16_t CW_QUICK_STOP        = 0x000B;
        static constexpr uint16_t CW_DISABLE_VOLTAGE   = 0x0000;

        bool DriveStateManager::init()
        {
            return true;
        }

        DriveState DriveStateManager::decodeStatusword(uint16_t statusWord)
        {
            bool fault            = (statusWord & 0x0008) != 0;  // bit 3
            bool switchOnDisabled = (statusWord & 0x0040) != 0;  // bit 6
            bool readyToSwitchOn  = (statusWord & 0x0001) != 0;  // bit 0
            bool switchedOn       = (statusWord & 0x0002) != 0;  // bit 1
            bool opEnabled        = (statusWord & 0x0004) != 0;  // bit 2
            bool quickStop        = (statusWord & 0x0020) != 0;  // bit 5

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

        void DriveStateManager::update(const DriveInput*       driveInputs,
                                       DriveOutput*            driveOutputs,
                                       const DriveUserSignals* userSignals,
                                       DriveFeedback*          feedback,
                                       std::size_t             driveCount)
        {
            for (std::size_t i = 0; i < driveCount; ++i)
            {
                DriveState ds = decodeStatusword(driveInputs[i].statusWord);
                const auto& sig = userSignals[i];

                uint16_t controlWord = CW_DISABLE_VOLTAGE;

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

                // minimal feedback
                feedback[i].faultActive       = (ds == DriveState::Fault);
                feedback[i].operationEnabled  = (ds == DriveState::OperationEnabled);
            }
        }
    } // namespace control
} // namespace motion_control
