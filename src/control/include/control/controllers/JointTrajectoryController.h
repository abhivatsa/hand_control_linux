#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RTIpc.h"

#include "control/controllers/BaseController.h"

namespace control
{
    /**
     * @brief Plays a pre-planned joint trajectory from shared memory.
     *
     * - On start(): snapshot latest JointTrajectoryPlan from RTMemoryLayout.
     * - On update(): interpolate positions at 1 kHz.
     * - Publishes progress into jointTrajectoryStatusBuffer.
     */
    class JointTrajectoryController : public BaseController
    {
    public:
        JointTrajectoryController(merai::RTMemoryLayout*           rtLayout,
                                  std::size_t                      jointCount,
                                  merai::multi_ring_logger_memory* loggerMem);

        bool init() override;

        bool start(std::span<const merai::JointMotionFeedback> motionFbk,
                   std::span<merai::JointMotionCommand>        motionCmd) override;

        void update(std::span<const merai::JointMotionFeedback> motionFbk,
                    std::span<merai::JointMotionCommand>        motionCmd,
                    double                                      dt) override;

        void stop() override;
        void teardown() override;

    private:
        void publishStatus(double progress,
                           bool   running,
                           bool   finished,
                           bool   aborted);

        void holdFinalPosition(std::span<merai::JointMotionCommand> motionCmd);

    private:
        merai::RTMemoryLayout*           rtLayout_   = nullptr;
        merai::multi_ring_logger_memory* loggerMem_  = nullptr;
        std::size_t                      jointCount_ = 0;

        merai::JointTrajectoryPlan       plan_{};       // local snapshot
        bool                             hasPlan_      = false;
        std::uint32_t                    activeJobId_  = 0;
        double                           currentTime_  = 0.0;
        std::size_t                      currentIndex_ = 0;
        bool                             finishedOnce_ = false;
    };

} // namespace control
