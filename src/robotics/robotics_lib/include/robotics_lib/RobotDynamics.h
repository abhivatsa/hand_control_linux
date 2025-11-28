#pragma once
#include <array>
#include "math_lib/Vector.h"
#include "math_lib/Matrix.h"
#include "robotics_lib/RobotModel.h"

namespace robot_lib
{

            class RobotDynamics {
            public:
                explicit RobotDynamics(const RobotModel& model);

                // All 7-DOF
                int computeInverseDynamics(const robot::math::Vector<7>& jointAngles,
                                        const robot::math::Vector<7>& jointVel,
                                        const robot::math::Vector<7>& jointAcc,
                                        robot::math::Vector<7>&       outTorques) const;

            private:
                robot::math::Matrix<4,4> computeJointTransform(int jointIdx, double jointAngle) const;

                const RobotModel& model_;
            };

} // namespace robot_lib
