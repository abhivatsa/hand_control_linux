#include <memory>
#include "control/ControllerManager.h"
#include "control/controllers/BaseController.h"

using seven_axis_robot::merai::ControllerCommand;
using seven_axis_robot::merai::ControllerFeedback;
using seven_axis_robot::merai::ControllerID;
using seven_axis_robot::merai::ControllerSwitchResult;

namespace
{
    class DummyController : public seven_axis_robot::control::BaseController
    {
    public:
        explicit DummyController(bool startSuccess = true) : startSuccess_(startSuccess) {}

        bool init() override
        {
            initCount++;
            state_ = ControllerState::INIT;
            return true;
        }

        bool start() override
        {
            startCount++;
            if (!startSuccess_)
            {
                state_ = ControllerState::STOPPED;
                return false;
            }
            state_ = ControllerState::RUNNING;
            return true;
        }

        void update(double /*dt*/) override
        {
            updateCount++;
        }

        void stop() override
        {
            stopCount++;
            state_ = ControllerState::STOPPED;
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
    seven_axis_robot::merai::ParameterServer ps{};
    ps.driveCount = 2;
    ps.jointCount = 2;

    seven_axis_robot::merai::JointMotionFeedback feedback[seven_axis_robot::merai::MAX_SERVO_DRIVES]{};
    seven_axis_robot::merai::JointMotionCommand commands[seven_axis_robot::merai::MAX_SERVO_DRIVES]{};
    seven_axis_robot::merai::multi_ring_logger_memory dummyLogger{};

    seven_axis_robot::control::ControllerManager mgr(
        &ps,
        feedback,
        commands,
        2,
        &dummyLogger);

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

    seven_axis_robot::control::ControllerManager::ModeCompatibilityPolicy policy{};
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
    mgr.update(command, feedbackOut, 0.001);

    if (feedbackOut.switchResult != ControllerSwitchResult::SUCCEEDED ||
        feedbackOut.activeControllerId != ControllerID::GRAVITY_COMP ||
        okController->startCount != 1)
    {
        return 1;
    }

    ControllerCommand idle{};
    mgr.update(idle, feedbackOut, 0.001);
    if (okController->updateCount == 0)
    {
        return 1;
    }

    ControllerCommand incompatible{};
    incompatible.requestSwitch = true;
    incompatible.controllerId = ControllerID::HOMING;
    mgr.update(incompatible, feedbackOut, 0.001);
    if (feedbackOut.switchResult != ControllerSwitchResult::FAILED ||
        feedbackOut.activeControllerId != ControllerID::GRAVITY_COMP)
    {
        return 1;
    }
    if (okController->stopCount != 0)
    {
        return 1;
    }

    seven_axis_robot::control::ControllerManager::ModeCompatibilityPolicy relaxedPolicy{};
    relaxedPolicy.allowedModes = {8, 10};
    mgr.setModeCompatibilityPolicy(relaxedPolicy);

    ControllerCommand failing{};
    failing.requestSwitch = true;
    failing.controllerId = ControllerID::HOMING;
    mgr.update(failing, feedbackOut, 0.001);
    if (feedbackOut.switchResult != ControllerSwitchResult::FAILED ||
        feedbackOut.activeControllerId != ControllerID::NONE ||
        okController->stopCount != 1)
    {
        return 1;
    }

    return 0;
}
