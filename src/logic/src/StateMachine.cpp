#include "logic/StateMachine.h"

namespace logic
{
    using merai::AppState;
    using merai::ControllerID;
    using merai::DriveCommand;
    using merai::DriveStatus;

    StateMachine::StateMachine(const merai::ParameterServer*    paramServerPtr,
                               merai::multi_ring_logger_memory* loggerMem)
        : paramServerPtr_(paramServerPtr),
          loggerMem_(loggerMem)
    {
    }

    bool StateMachine::init()
    {
        if (!paramServerPtr_)
        {
            return false;
        }

        // Number of drives from config, capped by MAX_JOINTS (7-DOF arm)
        driveCount_ = static_cast<std::size_t>(paramServerPtr_->driveCount);
        if (driveCount_ > MAX_JOINTS)
        {
            driveCount_ = MAX_JOINTS;
        }

        currentState_ = AppState::INIT;
        driveCmd_.commands.fill(DriveCommand::NONE);
        ctrlCmd_ = merai::ControllerCommand{};

        return true;
    }

    StateManagerOutput StateMachine::update(bool                            faultActive,
                                            bool                            trajectoryActive,
                                            const merai::DriveFeedbackData& driveFdbk,
                                            const merai::UserCommands&      userCmds,
                                            const merai::ControllerFeedback& ctrlFdbk)
    {
        StateManagerOutput output{};

        // Start each cycle from safe defaults.
        driveCmd_.commands.fill(DriveCommand::NONE);
        ctrlCmd_ = merai::ControllerCommand{};

        const bool eStopRequested = userCmds.eStop;

        // Keep a copy to detect state changes for logging.
        AppState prevState = currentState_;

        // --- Top-level overrides: ESTOP > FAULT > normal --------------------
        if (eStopRequested)
        {
            if (currentState_ != AppState::ESTOP)
            {
                merai::log_error(loggerMem_, "Logic", 3001,
                                 "[StateMachine] ESTOP requested -> entering ESTOP");
            }
            currentState_ = AppState::ESTOP;
        }
        else if (faultActive && currentState_ != AppState::ESTOP)
        {
            if (currentState_ != AppState::FAULT)
            {
                merai::log_error(loggerMem_, "Logic", 3002,
                                 "[StateMachine] Fault detected -> entering FAULT");
            }
            currentState_ = AppState::FAULT;
        }

        // --- State-specific logic -------------------------------------------
        switch (currentState_)
        {
        // --------------------------------------------------------------------
        case AppState::INIT:
        {
            // Bring drives from various CiA-402 states up to OPERATION_ENABLED.
            bool allOpEnabled = true;

            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                const auto status = driveFdbk.status[i];

                switch (status)
                {
                case DriveStatus::FAULT:
                    // Try to clear fault first.
                    driveCmd_.commands[i] = DriveCommand::FAULT_RESET;
                    allOpEnabled = false;
                    break;

                case DriveStatus::NOT_READY_TO_SWITCH_ON:
                case DriveStatus::SWITCH_ON_DISABLED:
                case DriveStatus::READY_TO_SWITCH_ON:
                    // Ask DriveStateManager to move up the CiA-402 ladder.
                    driveCmd_.commands[i] = DriveCommand::SWITCH_ON;
                    allOpEnabled = false;
                    break;

                case DriveStatus::SWITCHED_ON:
                    // Next step toward operation enabled.
                    driveCmd_.commands[i] = DriveCommand::ALLOW_OPERATION;
                    allOpEnabled = false;
                    break;

                case DriveStatus::OPERATION_ENABLED:
                    // Good, do nothing for this joint.
                    break;

                default:
                    // Unknown / unexpected → keep disabled.
                    driveCmd_.commands[i] = DriveCommand::FORCE_DISABLE;
                    allOpEnabled = false;
                    break;
                }
            }

            // Once everything is operation enabled, move to READY
            // and ask Control to switch to GRAVITY_COMP as the default
            // holding controller.
            if (allOpEnabled && !faultActive && !eStopRequested)
            {
                currentState_ = AppState::READY;
                ctrlCmd_.requestSwitch = true;
                ctrlCmd_.controllerId  = ControllerID::GRAVITY_COMP;
            }
            break;
        }

        // --------------------------------------------------------------------
        case AppState::READY:
        {
            // Drives are OK and no motion is running.
            // Default controller: GRAVITY_COMP (safe zero-motion posture).
            if (trajectoryActive && !faultActive && !eStopRequested)
            {
                // A trajectory/jog just started -> go ACTIVE.
                currentState_ = AppState::ACTIVE;
                // We assume controller switch to JOINT_TRAJECTORY / JOINT_JOG
                // is triggered elsewhere (e.g. by a planner orchestration).
                // No extra ctrlCmd_ here.
            }
            else
            {
                // Keep or restore GRAVITY_COMP as holding controller.
                ctrlCmd_.requestSwitch = true;
                ctrlCmd_.controllerId  = ControllerID::GRAVITY_COMP;
            }
            break;
        }

        // --------------------------------------------------------------------
        case AppState::ACTIVE:
        {
            // ACTIVE is meant for "motion running" (trajectory or jog).
            // If trajectoryActive drops, we go back to READY and gravity comp.
            if (!trajectoryActive || faultActive || eStopRequested)
            {
                // Motion finished/aborted, or we must stop due to fault/eStop.
                currentState_ = AppState::READY;

                if (!faultActive && !eStopRequested)
                {
                    ctrlCmd_.requestSwitch = true;
                    ctrlCmd_.controllerId  = ControllerID::GRAVITY_COMP;
                }
            }
            else
            {
                // Trajectory still running; leave ctrlCmd_ as default
                // (no new controller switch).
            }
            break;
        }

        // --------------------------------------------------------------------
        case AppState::FAULT:
        {
            // Keep drives safe. Allow exit to INIT once user resets and
            // all drives are in SWITCH_ON_DISABLED and no faultActive / no eStop.
            bool allSwitchOnDisabled = true;

            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                const auto status = driveFdbk.status[i];

                if (status == DriveStatus::FAULT)
                {
                    // Try to clear drive faults when user asks for reset.
                    if (userCmds.resetFault)
                    {
                        driveCmd_.commands[i] = DriveCommand::FAULT_RESET;
                    }
                    else
                    {
                        driveCmd_.commands[i] = DriveCommand::FORCE_DISABLE;
                    }
                    allSwitchOnDisabled = false;
                }
                else if (status == DriveStatus::SWITCH_ON_DISABLED)
                {
                    // good, leave it
                }
                else
                {
                    // Anything else in FAULT state we force disable.
                    driveCmd_.commands[i] = DriveCommand::FORCE_DISABLE;
                    allSwitchOnDisabled   = false;
                }
            }

            // While in FAULT, keep E_STOP controller as a safe fallback.
            ctrlCmd_.requestSwitch = true;
            ctrlCmd_.controllerId  = ControllerID::E_STOP;

            // Exit FAULT only if:
            //  - user requested reset,
            //  - no faults currently reported,
            //  - all drives are SWITCH_ON_DISABLED,
            //  - e-stop not pressed.
            if (userCmds.resetFault &&
                !faultActive &&
                allSwitchOnDisabled &&
                !eStopRequested)
            {
                currentState_ = AppState::INIT;
            }
            break;
        }

        // --------------------------------------------------------------------
        case AppState::ESTOP:
        {
            // Hard stop: force-disable all drives every cycle.
            for (std::size_t i = 0; i < driveCount_; ++i)
            {
                driveCmd_.commands[i] = DriveCommand::FORCE_DISABLE;
            }

            ctrlCmd_.requestSwitch = true;
            ctrlCmd_.controllerId  = ControllerID::E_STOP;

            // Only leave ESTOP if:
            //  - eStop released,
            //  - user asked to reset,
            //  - no active fault.
            if (!eStopRequested && userCmds.resetFault && !faultActive)
            {
                currentState_ = AppState::INIT;
            }
            break;
        }

        default:
            break;
        }

        // Log state transitions once.
        if (currentState_ != prevState)
        {
            const int code = 3000 + static_cast<int>(currentState_);
            merai::log_info(loggerMem_, "Logic", code,
                            "[StateMachine] AppState changed");
        }

        // Fill output
        output.appState = currentState_;
        output.driveCmd = driveCmd_;
        output.ctrlCmd  = ctrlCmd_;
        return output;
    }

} // namespace logic
