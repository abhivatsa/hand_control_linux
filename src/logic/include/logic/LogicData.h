// logic/include/logic/LogicData.h
namespace logic {

struct LogicCycleInputs {
    merai::UserCommands      userCmds;
    merai::DriveFeedbackData driveFdbk;
    merai::ControllerFeedback ctrlFdbk;
    // later: PlannerResult, JointTrajectoryStatus, etc.
};

struct LogicCycleOutputs {
    merai::DriveCommandData  driveCmd;
    merai::ControllerCommand ctrlCmd;
    merai::AppState          appState;
    // later: PlannerRequest, extra GUI feedback, etc.
};

} // namespace logic
