#pragma once

#include "robotics_lib/haptic_device/HapticDeviceModel.h"  // Updated to your HapticDeviceModel
#include "math_lib/Vector.h"          // For Vector<3> and Vector<6>
#include "math_lib/Matrix.h"          // For Matrix<3,3>, Matrix<4,4>
#include "math_lib/MatrixVectorOps.h" // If you need matrix*vector operators

namespace seven_axis_robot
{
    namespace robotics
    {
        namespace haptic_device
        {

            /**
             * @brief A simple example class for a hand model's inverse dynamics.
             *        This shows how to use seven_axis_robot::math::Vector/Matrix
             *        rather than Eigen or another external library.
             *
             * Note: This class is still set up for 6 DOF, following your previous
             *       six-axis logic. If your hand has fewer or more joints, update
             *       the Vector/Matrix dimensions and associated math accordingly.
             */
            class HapticDeviceDynamics
            {
            public:
                explicit HapticDeviceDynamics(const HapticDeviceModel &model);

                /**
                 * @brief Compute the inverse dynamics (torques) for given joint angles,
                 *        velocities, and accelerations.
                 *
                 * @param jointAngles  Current joint angles (Vector<6>)
                 * @param jointVel     Joint velocities (Vector<6>)
                 * @param jointAcc     Joint accelerations (Vector<6>)
                 * @param outTorques   Computed torques (Vector<6>)
                 * @return 0 if success, or error code
                 */
                int computeInverseDynamics(
                    const seven_axis_robot::math::Vector<6> &jointAngles,
                    const seven_axis_robot::math::Vector<6> &jointVel,
                    const seven_axis_robot::math::Vector<6> &jointAcc,
                    seven_axis_robot::math::Vector<6> &outTorques) const;

            private:
                /**
                 * @brief Compute the forward transform from link i to link (i+1)
                 *        given the current joint angle.
                 *
                 * @param jointIdx   Index of the joint
                 * @param jointAngle Joint angle in radians
                 * @return 4x4 homogeneous transform
                 */
                seven_axis_robot::math::Matrix<4, 4> computeJointTransform(
                    int jointIdx,
                    double jointAngle) const;

            private:
                const HapticDeviceModel &model_;  // Refer to your hand's model
            };

        } // namespace haptic_device
    } // namespace robotics
} // namespace seven_axis_robot
