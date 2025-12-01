enum class ControllerState
{
    INACTIVE,
    STARTING,
    RUNNING,
    STOPPING,
    STOPPED,
    ERROR
};

class BaseController
{
public:
    virtual ~BaseController() = default;

    virtual bool init() = 0;

    virtual bool start(std::span<const merai::JointMotionFeedback> motionFbk,
                       std::span<merai::JointMotionCommand> motionCmd) = 0;

    virtual void update(std::span<const merai::JointMotionFeedback> motionFbk,
                        std::span<merai::JointMotionCommand> motionCmd,
                        double dt) = 0;

    virtual void requestStop()
    {
        stop();
    }

    virtual void stop() = 0;
    virtual void teardown() = 0;

    virtual ControllerState state() const
    {
        return state_;
    }

    ControllerState getState() const
    {
        return state();
    }

protected:
    ControllerState state_{ControllerState::INACTIVE};
};
