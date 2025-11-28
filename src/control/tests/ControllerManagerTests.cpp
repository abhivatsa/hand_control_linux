#include <memory>
#include <span>
#include <chrono>
#include "control/ControllerManager.h"
#include "control/ControlData.h"
#include "control/controllers/BaseController.h"

using merai::ControllerCommand;
using merai::ControllerFeedback;
using merai::ControllerID;
using merai::ControllerSwitchResult;

using control::ControlCycleInputs;
using control::ControlCycleOutputs;

namespace
{
    class DummyController : public control::BaseController
    {
    public:
        explicit DummyController(bool startSuccess = true) : startSuccess_(startSuccess) {}

        bool init() override
        {
            initCount++;
            state_ = control::ControllerState::INIT;
            return true;
        }

        bool start(std::span<const merai::JointMotionFeedback> /*motionFbk*/,
                   std::span<merai::JointMotionCommand> /*motionCmd*/) override
        {
            startCount++;
            if (!startSuccess_)
            {
                state_ = control::ControllerState::STOPPED;
                return false;
            }
            state_ = control::ControllerState::RUNNING;
            return true;
        }

        void update(std::span<const merai::JointMotionFeedback> /*motionFbk*/,
                    std::span<merai::JointMotionCommand> /*motionCmd*/,
                    double /*dt*/) override
        {
            updateCount++;
        }

        void stop() override
        {
            stopCount++;
            state_ = control::ControllerState::STOPPED;
        }

        void teardown() override { teardownCount++; }

        int initCount = 0;
        int startCount = 0;
        int stopCount = 0;
        int updateCount = 0;
        int teardownCount = 0;

    private:
        bool startSuccess_ = true;
    };
}

int main()
{
    merai::ParameterServer ps{};
    ps.driveCount = 2;
    ps.jointCount = 2;

    merai::JointMotionFeedback feedback[merai::MAX_SERVO_DRIVES]{};
    merai::JointMotionCommand commands[merai::MAX_SERVO_DRIVES]{};
    merai::JointControlFeedback controlFeedback[merai::MAX_SERVO_DRIVES]{};
    merai::JointControlCommand controlCommands[merai::MAX_SERVO_DRIVES]{};
    merai::JointFeedbackIO ioFeedback[merai::MAX_SERVO_DRIVES]{};
    merai::multi_ring_logger_memory dummyLogger{};

    control::ControllerManager mgr(
        &ps,
        2,
        &dummyLogger,
        0.001);

    auto okController = std::make_shared<DummyController>(true);
    auto failingController = std::make_shared<DummyController>(false);

    if (!mgr.registerController(ControllerID::GRAVITY_COMP, okController, 8))
    {
        return 1;
    }
    if (!mgr.registerController(ControllerID::HOMING, failingController, 10))
    {
        return 1;
    }

    control::ControllerManager::ModeCompatibilityPolicy policy{};
    policy.allowedModes = {8};
    mgr.setModeCompatibilityPolicy(policy);

    if (!mgr.init())
    {
        return 1;
    }

    ControllerFeedback feedbackOut{};
    ControllerCommand command{};
    command.requestSwitch = true;
    command.controllerId = ControllerID::GRAVITY_COMP;

    ControlCycleInputs in{
        .jointControlFbk = std::span<const merai::JointControlFeedback>(controlFeedback, 2),
        .jointMotionFbk = std::span<const merai::JointMotionFeedback>(feedback, 2),
        .jointIoFbk = std::span<const merai::JointFeedbackIO>(ioFeedback, 2),
        .driveCmd = {},
        .ctrlCmd = command,
        .timestamp = std::chrono::steady_clock::now(),
    };

    ControlCycleOutputs out{
        .jointControlCmd = std::span<merai::JointControlCommand>(controlCommands, 2),
        .jointMotionCmd = std::span<merai::JointMotionCommand>(commands, 2),
    };

    mgr.update(in, out);
    feedbackOut = out.ctrlFbk;

    if (feedbackOut.switchResult != ControllerSwitchResult::SUCCEEDED ||
        feedbackOut.activeControllerId != ControllerID::GRAVITY_COMP ||
        okController->startCount != 1)
    {
        return 1;
    }

    ControllerCommand idle{};
    in.ctrlCmd = idle;
    in.timestamp = std::chrono::steady_clock::now();
    mgr.update(in, out);
    feedbackOut = out.ctrlFbk;
    if (okController->updateCount == 0)
    {
        return 1;
    }

    ControllerCommand incompatible{};
    incompatible.requestSwitch = true;
    incompatible.controllerId = ControllerID::HOMING;
    in.ctrlCmd = incompatible;
    in.timestamp = std::chrono::steady_clock::now();
    mgr.update(in, out);
    feedbackOut = out.ctrlFbk;
    if (feedbackOut.switchResult != ControllerSwitchResult::FAILED ||
        feedbackOut.activeControllerId != ControllerID::GRAVITY_COMP)
    {
        return 1;
    }
    if (okController->stopCount != 0)
    {
        return 1;
    }

    control::ControllerManager::ModeCompatibilityPolicy relaxedPolicy{};
    relaxedPolicy.allowedModes = {8, 10};
    mgr.setModeCompatibilityPolicy(relaxedPolicy);

    ControllerCommand failing{};
    failing.requestSwitch = true;
    failing.controllerId = ControllerID::HOMING;
    in.ctrlCmd = failing;
    in.timestamp = std::chrono::steady_clock::now();
    mgr.update(in, out);
    feedbackOut = out.ctrlFbk;
    if (feedbackOut.switchResult != ControllerSwitchResult::FAILED ||
        feedbackOut.activeControllerId != ControllerID::NONE ||
        okController->stopCount != 1)
    {
        return 1;
    }

    return 0;
}
