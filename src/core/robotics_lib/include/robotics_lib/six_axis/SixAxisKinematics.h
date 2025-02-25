#pragma once

#include "robotics_lib/six_axis/SixAxisModel.h"
#include "math_lib/Matrix.h"
#include "math_lib/Vector.h"

namespace motion_control
{
    namespace robotics
    {
        namespace six_axis
        {

            /**
             * @brief Kinematics class for a 6-axis manipulator, with out-parameter style functions.
             */
            class SixAxisKinematics
            {
            public:
                explicit SixAxisKinematics(const SixAxisModel &model);
                ~SixAxisKinematics() = default;

                /**
                 * @brief Compute the end-effector pose (4×4 homogeneous transform) for a given set of 6 joint angles.
                 *        The result is written into @p outTransform, and the method returns an int status (0 = success).
                 *
                 * @param jointAngles   The 6 current joint angles
                 * @param outTransform  A 4×4 matrix that, on success, contains the resulting end-effector pose
                 * @return int          0 if success, non-zero if there's an error
                 */
                int forwardKinematics(const motion_control::math::Vector<6> &jointAngles,
                                      motion_control::math::Matrix<4, 4> &outTransform) const;

                /**
                 * @brief Compute a 6×6 manipulator Jacobian for the given jointAngles.
                 *        - Rows 0–2 => linear velocity portion
                 *        - Rows 3–5 => angular velocity portion
                 *
                 * @param jointAngles   The 6 current joint angles
                 * @param outJacobian   A 6×6 matrix that, on success, will contain the Jacobian
                 * @return int          0 if success, non-zero if there's an error
                 */
                int computeJacobian(const motion_control::math::Vector<6> &jointAngles,
                                    motion_control::math::Matrix<6, 6> &outJacobian) const;

                /**
                 * @brief Example inverse kinematics routine (could be extended to handle RCM constraints, etc.).
                 *        Takes an initial guess or current joint angles plus a desired end-effector pose
                 *        and writes the resulting 6 joint angles into @p outJointAngles.
                 *
                 * @param currentJointAngles  The 6 current or guess angles
                 * @param desiredPose         A 4×4 homogeneous transform specifying the target pose
                 * @param outJointAngles      A 6×1 vector that, on success, contains the solution angles
                 * @return int                0 if success, non-zero if unsolvable or out of range
                 */
                int computeInverseKinematics(const motion_control::math::Vector<6> &currentJointAngles,
                                             const motion_control::math::Matrix<4, 4> &desiredPose,
                                             motion_control::math::Vector<6> &outJointAngles) const;

            private:
                // A helper to build a 4×4 transform from a 3×3 rotation + 3D position (if needed internally).
                motion_control::math::Matrix<4, 4> buildHomogeneousTransform(
                    const motion_control::math::Matrix<3, 3> &rot,
                    const motion_control::math::Vector<3> &pos) const;

                // A helper for building a rotation matrix from an axis + angle (Rodrigues formula), if needed.
                motion_control::math::Matrix<3, 3> axisAngleRotation(
                    const motion_control::math::Vector<3> &axis,
                    double angle) const;

            private:
                // Reference to your robot model (similar to how you store it in SixAxisDynamics).
                const SixAxisModel &model_;
            };

        } // namespace six_axis
    } // namespace robotics
} // namespace motion_control
