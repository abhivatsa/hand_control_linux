#pragma once

#include "robotics_lib/haptic_device/HapticDeviceModel.h"
#include "math_lib/Matrix.h"            // For Matrix<3,3>, etc.
#include "math_lib/Vector.h"            // For Vector<3> and Vector<6>
#include "math_lib/MatrixVectorOps.h"

namespace hand_control
{
    namespace robotics
    {
        namespace haptic_device
        {

            /**
             * @brief Kinematics class for a robotic hand, with out-parameter style functions.
             * 
             * Note: This example still uses six-joint vectors and 6×6 matrices,
             *       mimicking a 6-DOF hand/finger. Adjust as needed for your hand model.
             */
            class HapticDeviceKinematics
            {
            public:
                explicit HapticDeviceKinematics(const HapticDeviceModel &model);
                ~HapticDeviceKinematics() = default;

                /**
                 * @brief Compute the end-effector pose (4×4 homogeneous transform) 
                 *        for a given set of 6 joint angles.
                 *        The result is written into @p outTransform, and the method
                 *        returns an int status (0 = success).
                 *
                 * @param jointAngles   The 6 current joint angles (adjust if > 6 DOF)
                 * @param outTransform  A 4×4 matrix that, on success, contains the 
                 *                      resulting end-effector pose
                 * @return int          0 if success, non-zero if there's an error
                 */
                int forwardKinematics(const hand_control::math::Vector<6> &jointAngles,
                                      hand_control::math::Matrix<4, 4> &outTransform) const;

                /**
                 * @brief Compute a 6×6 manipulator Jacobian for the given jointAngles.
                 *        - Rows 0–2 => linear velocity portion
                 *        - Rows 3–5 => angular velocity portion
                 *
                 * @param jointAngles   The 6 current joint angles (adjust if > 6 DOF)
                 * @param outJacobian   A 6×6 matrix that, on success, will contain 
                 *                      the Jacobian
                 * @return int          0 if success, non-zero if there's an error
                 */
                int computeJacobian(const hand_control::math::Vector<6> &jointAngles,
                                    hand_control::math::Matrix<6, 6> &outJacobian) const;

                /**
                 * @brief Example inverse kinematics routine (could be extended to 
                 *        handle constraints, etc.). Takes an initial guess or 
                 *        current joint angles plus a desired end-effector pose
                 *        and writes the resulting joint angles into @p outJointAngles.
                 *
                 * @param currentJointAngles  The 6 current or guess angles (adjust if > 6 DOF)
                 * @param desiredPose         A 4×4 homogeneous transform specifying 
                 *                            the target pose
                 * @param outJointAngles      A 6×1 vector that, on success, contains 
                 *                            the solution angles
                 * @return int                0 if success, non-zero if unsolvable 
                 *                            or out of range
                 */
                int computeInverseKinematics(const hand_control::math::Vector<6> &currentJointAngles,
                                             const hand_control::math::Matrix<4, 4> &desiredPose,
                                             hand_control::math::Vector<6> &outJointAngles) const;

            private:
                // A helper to build a 4×4 transform from a 3×3 rotation + 3D position (if needed internally).
                hand_control::math::Matrix<4, 4> buildHomogeneousTransform(
                    const hand_control::math::Matrix<3, 3> &rot,
                    const hand_control::math::Vector<3> &pos) const;

                // A helper for building a rotation matrix from an axis + angle (Rodrigues formula), if needed.
                hand_control::math::Matrix<3, 3> axisAngleRotation(
                    const hand_control::math::Vector<3> &axis,
                    double angle) const;

            private:
                // Reference to your robot hand model.
                const HapticDeviceModel &model_;
            };

        } // namespace haptic_device
    } // namespace robotics
} // namespace hand_control
