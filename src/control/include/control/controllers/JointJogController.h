#pragma once

#include <cstddef>
#include <span>

#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "merai/RTIpc.h"

#include "robotics_lib/RobotModel.h"
#include "control/controllers/BaseController.h"

namespace control
{
    /**
     * @brief Simple joint jog controller.
     *
     * Reads JointJogCommand from shared memory each cycle,
     * integrates position with a clamped velocity, and clamps
     * to joint limits from RobotModel.
     */
    class JointJogController : public BaseController
    {
    public:
        JointJogController(merai::RTMemoryLayout*           rtLayout,
                           const robot_lib::RobotModel&     model,
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
        double clampToLimits(double q, std::size_t jointIndex) const;

    private:
        merai::RTMemoryLayout*           rtLayout_   = nullptr;
        const robot_lib::RobotModel&     model_;
        merai::multi_ring_logger_memory* loggerMem_  = nullptr;
        std::size_t                      jointCount_ = 0;

        merai::JointJogCommand           lastCmd_{}; // just for debug / introspection
    };

} // namespace control
