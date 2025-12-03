#pragma once

#include <cstddef>

#include "merai/Enums.h"
#include "merai/RTMemoryLayout.h"
#include "merai/ParameterServer.h"
#include "merai/SharedLogger.h"

namespace logic
{
    // Output from StateMachine::update: high-level app state + commands.
    struct StateManagerOutput
    {
        merai::DriveCommandData   driveCmd{};
        merai::ControllerCommand  ctrlCmd{};
        merai::AppState           appState{merai::AppState::INIT};
    };

    /**
     * @brief High-level app state machine.
     *
     * States (merai::AppState):
     *   INIT  : SHM OK, drives not yet fully enabled.
     *   READY : Drives OPERATION_ENABLED, safe holding controller (e.g. GRAVITY_COMP).
     *   ACTIVE: Some motion running (trajectory or jog).
     *   FAULT : Fault latched (drive/safety/controller).
     *   ESTOP : E-stop asserted (hardware or user command).
     *
     * Inputs:
     *   - faultActive      : from SafetyManager (true if any fault condition).
     *   - trajectoryActive : true if control is currently executing a trajectory/jog.
     *   - driveFdbk        : CiA-402 drive states (decoded by DriveStateManager).
     *   - userCmds         : e-stop, reset, shutdown, mode.
     *   - ctrlFdbk         : which controller is active / switch status.
     *
     * Outputs:
     *   - DriveCommandData  : CiA-402 commands (FORCE_DISABLE, FAULT_RESET, SWITCH_ON, ALLOW_OPERATION).
     *   - ControllerCommand : controller switch requests (GRAVITY_COMP, E_STOP, etc.).
     *   - AppState          : current high-level state.
     */
    class StateMachine
    {
    public:
        StateMachine(const merai::ParameterServer*    paramServerPtr,
                     merai::multi_ring_logger_memory* loggerMem);

        bool init();

        StateManagerOutput update(bool                            faultActive,
                                  bool                            trajectoryActive,
                                  const merai::DriveFeedbackData& driveFdbk,
                                  const merai::UserCommands&      userCmds,
                                  const merai::ControllerFeedback& ctrlFdbk);

    private:
        const merai::ParameterServer*    paramServerPtr_ = nullptr;
        merai::multi_ring_logger_memory* loggerMem_      = nullptr;

        merai::AppState         currentState_ = merai::AppState::INIT;
        merai::DriveCommandData driveCmd_{};
        merai::ControllerCommand ctrlCmd_{};

        std::size_t                     driveCount_ = 0;
        static constexpr std::size_t    MAX_JOINTS = 7;  // cap for arm joints
    };

} // namespace logic
