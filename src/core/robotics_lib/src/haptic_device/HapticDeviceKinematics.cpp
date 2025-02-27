#include "robotics_lib/haptic_device/HapticDeviceKinematics.h"
#include <cmath> // for sin, cos

namespace hand_control
{
    namespace robotics
    {
        namespace haptic_device
        {

            HapticDeviceKinematics::HapticDeviceKinematics(const HapticDeviceModel &model)
                : model_(model)
            {
                // If you want to precompute anything from model_, do it here.
            }

            int HapticDeviceKinematics::forwardKinematics(
                const hand_control::math::Vector<6> &jointAngles,
                hand_control::math::Matrix<4, 4> &outTransform) const
            {
                using namespace hand_control::math;

                // Start with identity transform (4×4)
                Matrix<4, 4> Ttotal;
                Ttotal.setIdentity();

                // For each of the 6 joints, multiply transforms
                for (int i = 0; i < HapticDeviceModel::NUM_JOINTS; ++i)
                {
                    // 1) Build a 4×4 from the joint’s stored origin (3×3 rotation + 3D pos).
                    const auto &Rorig = model_.getJointOriginRot(i); // Matrix<3,3>
                    const auto &Porig = model_.getJointOriginPos(i); // Vector<3>
                    Matrix<4, 4> T_origin = buildHomogeneousTransform(Rorig, Porig);

                    // 2) Build the rotation about the joint's axis by jointAngles[i].
                    const auto &axis = model_.getJointAxis(i); // Vector<3>
                    double angle = jointAngles[i];
                    Matrix<3, 3> R_joint = axisAngleRotation(axis, angle);

                    // No translation for the joint rotation
                    Matrix<4, 4> T_joint = buildHomogeneousTransform(R_joint, Vector<3>());

                    // 3) Multiply them into the total
                    Matrix<4, 4> T_link = T_origin * T_joint;
                    Ttotal = Ttotal * T_link; // or Ttotal *= T_link;
                }

                // Write out the result
                outTransform = Ttotal;
                return 0; // success
            }

            int HapticDeviceKinematics::computeJacobian(
                const hand_control::math::Vector<6> &jointAngles,
                hand_control::math::Matrix<6, 6> &outJacobian) const
            {
                using namespace hand_control::math;

                outJacobian.setZero(); // fill with 0

                // 1) Compute the end-effector transform => for p_end
                Matrix<4, 4> T_end;
                int ret = forwardKinematics(jointAngles, T_end);
                if (ret != 0)
                {
                    return ret; // pass on the error code
                }

                Vector<3> p_end{{T_end(0, 3), T_end(1, 3), T_end(2, 3)}};

                // 2) We'll accumulate a transform T_current from the base to each joint i
                Matrix<4, 4> T_current;
                T_current.setIdentity();

                for (int i = 0; i < HapticDeviceModel::NUM_JOINTS; ++i)
                {
                    // Joint i: origin transform
                    const auto &Rorig = model_.getJointOriginRot(i);
                    const auto &Porig = model_.getJointOriginPos(i);
                    Matrix<4, 4> T_origin = buildHomogeneousTransform(Rorig, Porig);

                    // Joint i: rotation about axis
                    const auto &axis = model_.getJointAxis(i);
                    double angle = jointAngles[i];
                    Matrix<3, 3> R_joint = axisAngleRotation(axis, angle);
                    Matrix<4, 4> T_joint = buildHomogeneousTransform(R_joint, Vector<3>());

                    // Update T_current
                    Matrix<4, 4> T_link = T_origin * T_joint;
                    T_current = T_current * T_link;

                    // The joint’s position in world coords:
                    Vector<3> p_joint{{T_current(0, 3), T_current(1, 3), T_current(2, 3)}};

                    // The joint axis in world coords (assuming Z axis = rotation axis in local frame):
                    Vector<3> z_axis{{T_current(0, 2), T_current(1, 2), T_current(2, 2)}};

                    // linear part: cross(z_axis, p_end - p_joint)
                    Vector<3> diff{{p_end[0] - p_joint[0],
                                    p_end[1] - p_joint[1],
                                    p_end[2] - p_joint[2]}};
                    Vector<3> Jv = z_axis.cross(diff);

                    // angular part: z_axis (for a revolute joint)
                    Vector<3> Jw = z_axis;

                    // Fill the columns in outJacobian
                    for (int row = 0; row < 3; ++row)
                    {
                        outJacobian(row, i) = Jv[row];     // linear
                        outJacobian(row + 3, i) = Jw[row]; // angular
                    }
                }

                return 0; // success
            }

            int HapticDeviceKinematics::computeInverseKinematics(
                const hand_control::math::Vector<6> &currentJointAngles,
                const hand_control::math::Matrix<4, 4> &desiredPose,
                hand_control::math::Vector<6> &outJointAngles) const
            {
                // Placeholder: always returns success with a trivial “solution”.
                // In practice, you’d do numerical or analytical IK here.

                // For example, you might copy the current angles as if there's no change:
                outJointAngles = currentJointAngles;

                // Return 0 for success, or non-zero if unsolvable.
                return 0;
            }

            // -----------------------------------------------------------------------------
            // Helper Functions
            // -----------------------------------------------------------------------------

            hand_control::math::Matrix<4, 4> HapticDeviceKinematics::buildHomogeneousTransform(
                const hand_control::math::Matrix<3, 3> &rot,
                const hand_control::math::Vector<3> &pos) const
            {
                using namespace hand_control::math;
                Matrix<4, 4> T;
                T.setIdentity();

                // Fill rotation
                for (std::size_t r = 0; r < 3; ++r)
                {
                    for (std::size_t c = 0; c < 3; ++c)
                    {
                        T(r, c) = rot(r, c);
                    }
                }
                // Fill translation
                T(0, 3) = pos[0];
                T(1, 3) = pos[1];
                T(2, 3) = pos[2];

                return T;
            }

            hand_control::math::Matrix<3, 3> HapticDeviceKinematics::axisAngleRotation(
                const hand_control::math::Vector<3> &axis,
                double angle) const
            {
                using namespace hand_control::math;

                // Standard Rodrigues' formula
                Vector<3> axNorm = axis.normalized();
                double x = axNorm[0], y = axNorm[1], z = axNorm[2];

                double c = std::cos(angle);
                double s = std::sin(angle);
                double one_c = 1.0 - c;

                Matrix<3, 3> R;
                R(0, 0) = c + x * x * one_c;
                R(0, 1) = x * y * one_c - z * s;
                R(0, 2) = x * z * one_c + y * s;
                R(1, 0) = y * x * one_c + z * s;
                R(1, 1) = c + y * y * one_c;
                R(1, 2) = y * z * one_c - x * s;
                R(2, 0) = z * x * one_c - y * s;
                R(2, 1) = z * y * one_c + x * s;
                R(2, 2) = c + z * z * one_c;

                return R;
            }

        } // namespace hand_control
    } // namespace robotics
} // namespace hand_control
