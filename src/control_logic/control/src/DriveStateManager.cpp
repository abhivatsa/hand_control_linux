#include <iostream>
#include "control/DriveStateManager.h"

// For convenience, bring in the CiA-402 control word definitions
namespace
{
    // Control word definitions (CiA 402)
    constexpr uint16_t CW_FAULT_RESET       = 0x0080;
    constexpr uint16_t CW_SHUTDOWN          = 0x0006;
    constexpr uint16_t CW_SWITCH_ON         = 0x0007;
    constexpr uint16_t CW_ENABLE_OPERATION  = 0x000F;
    constexpr uint16_t CW_QUICK_STOP        = 0x000B;
    constexpr uint16_t CW_DISABLE_VOLTAGE   = 0x0000;
}

namespace seven_axis_robot
{
    namespace control
    {
        DriveStateManager::DriveStateManager(
            seven_axis_robot::merai::JointControlCommand* jointCommandPtr,
            seven_axis_robot::merai::JointControlFeedback* jointFeedbackPtr,
            std::size_t driveCount)
            : jointCommandPtr_(jointCommandPtr),
              jointFeedbackPtr_(jointFeedbackPtr),
              driveCount_(driveCount)
        {
        }

        bool DriveStateManager::init()
        {
            // Check pointers
            if (!jointCommandPtr_ || !jointFeedbackPtr_ || driveCount_ == 0)
            {
                std::cerr << "[DriveStateManager] Invalid pointers or drive count.\n";
                return false;
            }
            return true;
        }

        seven_axis_robot::merai::DriveStatus 
        DriveStateManager::decodeStatusword(uint16_t statusWord)
        {
            // Based on CiA 402 bit definitions
            bool fault            = (statusWord & 0x0008) != 0;  // bit 3
            bool switchOnDisabled = (statusWord & 0x0040) != 0;  // bit 6
            bool readyToSwitchOn  = (statusWord & 0x0001) != 0;  // bit 0
            bool switchedOn       = (statusWord & 0x0002) != 0;  // bit 1
            bool opEnabled        = (statusWord & 0x0004) != 0;  // bit 2
            bool quickStop        = (statusWord & 0x0020) != 0;  // bit 5

            if (fault)
                return seven_axis_robot::merai::DriveStatus::FAULT;
            if (switchOnDisabled)
                return seven_axis_robot::merai::DriveStatus::SWITCH_ON_DISABLED;
            if (!readyToSwitchOn && !switchedOn && !opEnabled)
                return seven_axis_robot::merai::DriveStatus::NOT_READY_TO_SWITCH_ON;
            if (readyToSwitchOn && !switchedOn && !opEnabled)
                return seven_axis_robot::merai::DriveStatus::READY_TO_SWITCH_ON;
            if (readyToSwitchOn && switchedOn && !opEnabled)
                return seven_axis_robot::merai::DriveStatus::SWITCHED_ON;
            if (readyToSwitchOn && switchedOn && opEnabled)
            {
                if (!quickStop)
                    return seven_axis_robot::merai::DriveStatus::QUICK_STOP;
                else
                    return seven_axis_robot::merai::DriveStatus::OPERATION_ENABLED;
            }
            return seven_axis_robot::merai::DriveStatus::NOT_READY_TO_SWITCH_ON;
        }

        void DriveStateManager::update(const seven_axis_robot::merai::DriveCommand* driveCommands,
                                       seven_axis_robot::merai::DriveStatus* driveStatus)
        {
            if (!jointCommandPtr_ || !jointFeedbackPtr_ || !driveCommands || !driveStatus)
            {
                // Handle null pointers
                return;
            }

            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                // 1) Decode the status word (CiA-402) from JointControlFeedback
                uint16_t sw = jointFeedbackPtr_[i].statusWord;
                driveStatus[i] = decodeStatusword(sw);

                // 2) Decide on the new control word based on drive status + input command
                seven_axis_robot::merai::DriveCommand cmd = driveCommands[i];
                uint16_t controlWord = CW_DISABLE_VOLTAGE; // default

                // std::cout<<"Joint Data - i "<<i<<std::endl;
                // std::cout<<"Drive Command : "<<int(cmd)<<std::endl;

                switch (driveStatus[i])
                {
                case seven_axis_robot::merai::DriveStatus::FAULT:
                    // std::cout<<"Drive state Fault, cmd == seven_axis_robot::merai::DriveCommand::FAULT_RESET : "<<(cmd == seven_axis_robot::merai::DriveCommand::FAULT_RESET)<<std::endl;
                    if (cmd == seven_axis_robot::merai::DriveCommand::FAULT_RESET)
                        controlWord = CW_FAULT_RESET;
                    else
                        controlWord = CW_DISABLE_VOLTAGE;
                    break;

                case seven_axis_robot::merai::DriveStatus::SWITCH_ON_DISABLED:
                // std::cout<<"Drive state SWITCH_ON_DISABLED, DriveStatus::SWITCH_ON_DISABLED, control word Shutdown "<<std::endl;
                    controlWord = CW_SHUTDOWN;
                    break;

                case seven_axis_robot::merai::DriveStatus::NOT_READY_TO_SWITCH_ON:
                // std::cout<<"Drive state NOT_READY_TO_SWITCH_ON"<<std::endl;
                    controlWord = CW_DISABLE_VOLTAGE;
                    break;

                case seven_axis_robot::merai::DriveStatus::READY_TO_SWITCH_ON:
                // std::cout<<"Drive state READY_TO_SWITCH_ON"<<std::endl;
                    controlWord = CW_SWITCH_ON;
                    break;

                case seven_axis_robot::merai::DriveStatus::SWITCHED_ON:
                
                    if (cmd == seven_axis_robot::merai::DriveCommand::ALLOW_OPERATION)
                    {
                        std::cout<<"command recieved for operation enabled"<<std::endl;
                        controlWord = CW_ENABLE_OPERATION;
                    }
                    else
                        controlWord = CW_SWITCH_ON;
                    break;

                case seven_axis_robot::merai::DriveStatus::OPERATION_ENABLED:
                std::cout<<"Drive state OPERATION_ENABLED"<<std::endl;
                    if (cmd == seven_axis_robot::merai::DriveCommand::ALLOW_OPERATION)
                        controlWord = CW_ENABLE_OPERATION;
                    else if (cmd == seven_axis_robot::merai::DriveCommand::FORCE_DISABLE)
                        controlWord = CW_SWITCH_ON;
                    else
                        controlWord = CW_ENABLE_OPERATION;
                    break;

                case seven_axis_robot::merai::DriveStatus::QUICK_STOP:
                // std::cout<<"Drive state QUICK_STOP"<<std::endl;
                    if (cmd == seven_axis_robot::merai::DriveCommand::ALLOW_OPERATION)
                        controlWord = CW_SWITCH_ON;
                    else
                        controlWord = CW_QUICK_STOP;
                    break;

                default:
                    controlWord = CW_DISABLE_VOLTAGE;
                    break;
                }

                // std::cout<<"************* controlWord : "<<controlWord<<std::endl;

                // 3) Write the computed control word into JointControlCommand
                jointCommandPtr_[i].controlWord = controlWord;
            }
        }

    } // namespace control
} // namespace seven_axis_robot
