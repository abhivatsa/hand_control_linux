#include "logic/StateMachine.h"
#include <iostream>

namespace seven_axis_robot
{
    namespace logic
    {
        StateMachine::StateMachine(const seven_axis_robot::merai::ParameterServer *paramServerPtr)
            : paramServerPtr_(paramServerPtr)
        {
        }

        bool StateMachine::init()
        {
            if (!paramServerPtr_)
            {
                // Possibly log an error or handle differently in RT
                return false;
            }

            // 1) read driveCount (or jointCount) from paramServer
            driveCount_ = paramServerPtr_->driveCount;
            if (driveCount_ > MAX_JOINTS)
            {
                driveCount_ = MAX_JOINTS;
            }

            currentState_ = merai::AppState::INIT;
            return true;
        }

        StateManagerOutput StateMachine::update(bool faultActive, bool isHomingCompleted,
                                                const seven_axis_robot::merai::DriveFeedbackData &driveFdbk,
                                                const seven_axis_robot::merai::UserCommands &userCmds,
                                                const seven_axis_robot::merai::ControllerFeedback &ctrlFdbk)
        {
            StateManagerOutput output;

            // 1) Handle faults first
            if (faultActive)
            {

                currentState_ = merai::AppState::FAULT;

                for (int i = 0; i < driveCount_; i++)
                {
                    driveCmd_.commands[i] = seven_axis_robot::merai::DriveCommand::FORCE_DISABLE;
                }

                ctrlCmd_.requestSwitch = true;
                ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::E_STOP;

                state_transistion = true;

                std::cout << "[StateMachine] Entering FAULT (unrecoverable)\n";
            }

            // 2) State transitions based on current state and homing status
            switch (currentState_)
            {
            case merai::AppState::INIT:

                // Checking state of all Drive to Determine if any drive is not in Operation Enabled
                all_drive_operation_enable = true;
                all_drives_switched_on = true;

                for (int i = 0; i < driveCount_; i++)
                {

                    if ((driveFdbk.status[i] != seven_axis_robot::merai::DriveStatus::SWITCHED_ON) && (driveFdbk.status[i] != seven_axis_robot::merai::DriveStatus::OPERATION_ENABLED))
                    {

                        if ((driveFdbk.status[i] != seven_axis_robot::merai::DriveStatus::SWITCH_ON_DISABLED) && (driveFdbk.status[i] != seven_axis_robot::merai::DriveStatus::READY_TO_SWITCH_ON))
                        {
                            driveCmd_.commands[i] = seven_axis_robot::merai::DriveCommand::FORCE_DISABLE;
                        }
                        else
                        {
                            driveCmd_.commands[i] = seven_axis_robot::merai::DriveCommand::SWITCH_ON;
                        }

                        all_drives_switched_on = false;
                    }
                }

                if (all_drives_switched_on == true)
                {                    
                    for (int i = 0; i < driveCount_; i++)
                    {
                        if (driveFdbk.status[i] != seven_axis_robot::merai::DriveStatus::OPERATION_ENABLED)
                        {
                            driveCmd_.commands[i] = seven_axis_robot::merai::DriveCommand::ALLOW_OPERATION;
                            all_drive_operation_enable = false;
                        }
                    }

                    if (all_drive_operation_enable == true)// && ctrlFdbk.feedbackState != seven_axis_robot::merai::ControllerFeedbackState::SWITCH_COMPLETED)
                    {
                        currentState_ = merai::AppState::HOMING;
                        ctrlCmd_.requestSwitch = true;
                        ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::HOMING;
                        state_transistion = true;
                        break;
                    }
                }

                if (state_transistion == true)
                {
                    ctrlCmd_.requestSwitch = true;
                    ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::NONE;
                    state_transistion = false;
                }
                else
                {
                    ctrlCmd_.requestSwitch = false;
                    ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::NONE;
                }

                break;

            case merai::AppState::HOMING:

                std::cout << "App State HOMING, "<<isHomingCompleted<< std::endl;

                if (isHomingCompleted)
                {
                    ctrlCmd_.requestSwitch = true;
                    ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::GRAVITY_COMP;
                    state_transistion = true;
                    currentState_ = merai::AppState::ACTIVE;
                    break;
                }

                if (state_transistion == true)
                {
                    ctrlCmd_.requestSwitch = true;
                    ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::HOMING;
                    state_transistion = false;
                }
                else{
                    ctrlCmd_.requestSwitch = false;
                    ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::HOMING;
                }

                break;

            case merai::AppState::ACTIVE:

                std::cout << "App State ACTIVE" << std::endl;

                break;

            case merai::AppState::FAULT:

                std::cout << "App State Fault" << std::endl;
                // Remain in FAULT until the user resets the fault.
                // if (state_transistion == true)
                // {
                //     ctrlCmd_.requestSwitch = true;
                //     ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::E_STOP;
                //     state_transistion = false;

                //     for (int i = 0; i < driveCount_; i++)
                //     {
                //         driveCmd_.commands[i] = seven_axis_robot::merai::DriveCommand::FORCE_DISABLE;
                //     }
                // }

                all_drives_switch_on_disabled = true;

                for (int i = 0; i < driveCount_; i++)
                {
                    if (driveFdbk.status[i] == seven_axis_robot::merai::DriveStatus::FAULT){
                        driveCmd_.commands[i] = seven_axis_robot::merai::DriveCommand::FAULT_RESET;
                        all_drives_switch_on_disabled = false;
                    }
                    else if (driveFdbk.status[i] != seven_axis_robot::merai::DriveStatus::SWITCH_ON_DISABLED)
                    {
                        driveCmd_.commands[i] = seven_axis_robot::merai::DriveCommand::FORCE_DISABLE;
                        all_drives_switch_on_disabled = false;
                    }

                }

                if (all_drives_switch_on_disabled)
                {
                    ctrlCmd_.requestSwitch = true;
                    ctrlCmd_.controllerId = seven_axis_robot::merai::ControllerID::NONE;
                    currentState_ = merai::AppState::INIT;
                }
                break;

            default:
                break;
            }

            // 3) Set the output app state
            output.appState = currentState_;
            // 4) Compute drive and controller commands.
            //    Replace the default construction with your actual command logic as needed.
            output.driveCmd = driveCmd_;
            output.ctrlCmd = ctrlCmd_;

            return output;
        }
    } // namespace logic
} // namespace seven_axis_robot
