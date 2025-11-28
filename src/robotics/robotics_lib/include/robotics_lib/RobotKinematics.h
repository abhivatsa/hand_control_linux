#pragma once

#include "robotics_lib/RobotModel.h"
#include "math_lib/Matrix.h"            // For Matrix<3,3>, Matrix<4,4>, Matrix<6,7>
#include "math_lib/Vector.h"            // For Vector<3>, Vector<7>
#include "math_lib/MatrixVectorOps.h"

namespace robot_lib
{

            /**
             * @brief Kinematics for a 7-DOF arm.
             *
             * FK: takes Vector<7> -> 4×4 pose
             * J : returns 6×7 geometric Jacobian (linear; angular)
             * IK: placeholder (copies input -> output)
             */
            class RobotKinematics
            {
            public:
                explicit RobotKinematics(const RobotModel &model);
                ~RobotKinematics() = default;

                // Forward Kinematics: q(7) -> T(4x4)
                int forwardKinematics(const robot::math::Vector<7> &jointAngles,
                                      robot::math::Matrix<4, 4> &outTransform) const;

                // Geometric Jacobian: 6x7 (top 3 rows = linear, bottom 3 rows = angular)
                int computeJacobian(const robot::math::Vector<7> &jointAngles,
                                    robot::math::Matrix<6, 7> &outJacobian) const;

                // IK placeholder: returns currentJointAngles as-is (extend later)
                int computeInverseKinematics(const robot::math::Vector<7> &currentJointAngles,
                                             const robot::math::Matrix<4, 4> &desiredPose,
                                             robot::math::Vector<7> &outJointAngles) const;

            private:
                // Build 4×4 from R(3×3) and p(3)
                robot::math::Matrix<4, 4> buildHomogeneousTransform(
                    const robot::math::Matrix<3, 3> &rot,
                    const robot::math::Vector<3> &pos) const;

                // Axis-angle rotation (Rodrigues). Robust for near-zero axis.
                robot::math::Matrix<3, 3> axisAngleRotation(
                    const robot::math::Vector<3> &axis,
                    double angle) const;

            private:
                const RobotModel &model_;
            };

} // namespace robot_lib
