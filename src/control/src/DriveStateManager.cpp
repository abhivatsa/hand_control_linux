#include "control/DriveStateManager.h"
#include "merai/RTIpc.h"

// For convenience, bring in the CiA-402 control word definitions
namespace
{
    // Control word definitions (CiA 402)
    constexpr uint16_t CW_FAULT_RESET = 0x0080;
    constexpr uint16_t CW_SHUTDOWN = 0x0006;
    constexpr uint16_t CW_SWITCH_ON = 0x0007;
    constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
    constexpr uint16_t CW_QUICK_STOP = 0x000B;
    constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
}

namespace control
{
    DriveStateManager::DriveStateManager(
        std::size_t driveCount,
        merai::multi_ring_logger_memory *loggerMem)
        : driveCount_(driveCount),
          loggerMem_(loggerMem)
    {
    }

    bool DriveStateManager::init()
    {
        // Check pointers
        if (driveCount_ == 0)
        {
            merai::log_error(loggerMem_, "Control", 2000, "[DriveStateManager] Invalid drive count");
            return false;
        }
        return true;
    }

    merai::DriveStatus
    DriveStateManager::decodeStatusword(uint16_t statusWord)
    {
        // Based on CiA 402 bit definitions
        bool fault = (statusWord & 0x0008) != 0;            // bit 3
        bool switchOnDisabled = (statusWord & 0x0040) != 0; // bit 6
        bool readyToSwitchOn = (statusWord & 0x0001) != 0;  // bit 0
        bool switchedOn = (statusWord & 0x0002) != 0;       // bit 1
        bool opEnabled = (statusWord & 0x0004) != 0;        // bit 2
        bool quickStopBitSet = (statusWord & 0x0020) != 0;  // bit 5 (0 = quick stop active)

        if (fault)
            return merai::DriveStatus::FAULT;
        if (switchOnDisabled)
            return merai::DriveStatus::SWITCH_ON_DISABLED;
        if (!readyToSwitchOn && !switchedOn && !opEnabled)
            return merai::DriveStatus::NOT_READY_TO_SWITCH_ON;
        if (readyToSwitchOn && !switchedOn && !opEnabled)
            return merai::DriveStatus::READY_TO_SWITCH_ON;
        if (readyToSwitchOn && switchedOn && !opEnabled)
            return merai::DriveStatus::SWITCHED_ON;
        if (readyToSwitchOn && switchedOn && opEnabled)
        {
            if (!quickStopBitSet)
                return merai::DriveStatus::QUICK_STOP;
            else
                return merai::DriveStatus::OPERATION_ENABLED;
        }
        return merai::DriveStatus::NOT_READY_TO_SWITCH_ON;
    }

    void DriveStateManager::update(const ControlCycleInputs &in,
                                   ControlCycleOutputs &out)
    {
        if (in.jointControlFbk.size() < driveCount_ ||
            out.jointControlCmd.size() < driveCount_)
        {
            // Handle size mismatch
            return;
        }

        out.driveFbk.status.fill(merai::DriveStatus::NOT_READY_TO_SWITCH_ON);

        for (std::size_t i = 0; i < driveCount_; ++i)
        {
            // 1) Decode the status word (CiA-402) from JointControlFeedback
            uint16_t sw = in.jointControlFbk[i].statusWord;
            auto status = decodeStatusword(sw);
            out.driveFbk.status[i] = status;

            // 2) Decide on the new control word based on drive status + input command
            merai::DriveCommand cmd = in.driveCmd.commands[i];
            uint16_t controlWord = CW_DISABLE_VOLTAGE; // default

            switch (status)
            {
            case merai::DriveStatus::FAULT:
                if (cmd == merai::DriveCommand::FAULT_RESET)
                    controlWord = CW_FAULT_RESET;
                else
                    controlWord = CW_DISABLE_VOLTAGE;
                break;

            case merai::DriveStatus::SWITCH_ON_DISABLED:
                controlWord = CW_SHUTDOWN;
                break;

            case merai::DriveStatus::NOT_READY_TO_SWITCH_ON:
                controlWord = CW_DISABLE_VOLTAGE;
                break;

            case merai::DriveStatus::READY_TO_SWITCH_ON:
                controlWord = CW_SWITCH_ON;
                break;

            case merai::DriveStatus::SWITCHED_ON:

                if (cmd == merai::DriveCommand::ALLOW_OPERATION)
                {
                    controlWord = CW_ENABLE_OPERATION;
                }
                else
                {
                    controlWord = CW_SWITCH_ON;
                }
                break;

            case merai::DriveStatus::OPERATION_ENABLED:
                if (cmd == merai::DriveCommand::ALLOW_OPERATION)
                    controlWord = CW_ENABLE_OPERATION;
                else if (cmd == merai::DriveCommand::FORCE_DISABLE)
                    controlWord = CW_SWITCH_ON;
                else
                    controlWord = CW_ENABLE_OPERATION;
                break;

            case merai::DriveStatus::QUICK_STOP:
                if (cmd == merai::DriveCommand::ALLOW_OPERATION)
                    controlWord = CW_SWITCH_ON;
                else
                    controlWord = CW_QUICK_STOP;
                break;

            default:
                controlWord = CW_DISABLE_VOLTAGE;
                break;
            }
            // 3) Write the computed control word into JointControlCommand
            out.jointControlCmd[i].controlWord = controlWord;
        }
    }

} // namespace control
