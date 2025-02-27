#include "robotics_lib/haptic_device/HapticDeviceModel.h"

namespace hand_control
{
    namespace robotics
    {
        namespace haptic_device
        {
            bool HapticDeviceModel::loadFromParameterServer(const hand_control::merai::ParameterServer &ps)
            {
                // 1) Check if we have enough links & joints for the hand model
                if (ps.linkCount < NUM_LINKS || ps.jointCount < NUM_JOINTS)
                {
                    return false;
                }

                // 2) Store the robot's (hand's) name
                robotName_ = ps.robot_name.c_str();  // or a custom method like .toStdString()

                // 3) Load each LinkConfig into linkMasses_, linkCOMs_, linkInertias_
                for (int i = 0; i < NUM_LINKS; ++i)
                {
                    const auto &linkCfg = ps.links[i];
                    linkMasses_[i] = linkCfg.mass;

                    linkCOMs_[i] = hand_control::math::Vector<3>{
                        { linkCfg.com[0], linkCfg.com[1], linkCfg.com[2] }
                    };

                    linkInertias_[i] = makeInertiaMatrix(
                        linkCfg.inertia[0],
                        linkCfg.inertia[1],
                        linkCfg.inertia[2],
                        linkCfg.inertia[3],
                        linkCfg.inertia[4],
                        linkCfg.inertia[5]
                    );
                }

                // 4) Load each JointConfig into jointParents_, jointChildren_,
                //    jointOriginPos_, jointOriginRot_, jointAxes_, jointMinPos_, jointMaxPos_
                for (int j = 0; j < NUM_JOINTS; ++j)
                {
                    const auto &jointCfg = ps.joints[j];

                    jointParents_[j] = jointCfg.parent.c_str();
                    jointChildren_[j] = jointCfg.child.c_str();

                    jointOriginPos_[j] = hand_control::math::Vector<3>{
                        { jointCfg.origin_pos[0], jointCfg.origin_pos[1], jointCfg.origin_pos[2] }
                    };

                    double roll = jointCfg.origin_orient[0];
                    double pitch = jointCfg.origin_orient[1];
                    double yaw = jointCfg.origin_orient[2];
                    jointOriginRot_[j] = eulerToRotation(roll, pitch, yaw);

                    jointAxes_[j] = hand_control::math::Vector<3>{
                        { jointCfg.axis[0], jointCfg.axis[1], jointCfg.axis[2] }
                    };

                    jointMinPos_[j] = jointCfg.limits.position.min;
                    jointMaxPos_[j] = jointCfg.limits.position.max;
                }

                // Everything loaded successfully
                return true;
            }

            // -------------------------------------------------------------
            // Helper: Build a 3×3 inertia matrix from ixx, iyy, izz, ixy, ixz, iyz
            // -------------------------------------------------------------
            hand_control::math::Matrix<3, 3> HapticDeviceModel::makeInertiaMatrix(
                double ixx, double iyy, double izz,
                double ixy, double ixz, double iyz) const
            {
                hand_control::math::Matrix<3, 3> I;
                I.setZero();
                // Diagonal
                I(0, 0) = ixx;
                I(1, 1) = iyy;
                I(2, 2) = izz;
                // Off-diagonal
                I(0, 1) = ixy;  I(1, 0) = ixy;
                I(0, 2) = ixz;  I(2, 0) = ixz;
                I(1, 2) = iyz;  I(2, 1) = iyz;
                return I;
            }

            // -------------------------------------------------------------
            // Helper: Convert Euler angles (roll, pitch, yaw) to a 3×3 rotation
            // -------------------------------------------------------------
            hand_control::math::Matrix<3, 3> HapticDeviceModel::eulerToRotation(
                double roll, double pitch, double yaw) const
            {
                using std::cos;
                using std::sin;

                double cr = cos(roll),  sr = sin(roll);
                double cp = cos(pitch), sp = sin(pitch);
                double cy = cos(yaw),   sy = sin(yaw);

                hand_control::math::Matrix<3, 3> R;
                R.setZero();

                // Example: Z-Y-X rotation
                R(0,0) = cy*cp;
                R(0,1) = cy*sp*sr - sy*cr;
                R(0,2) = cy*sp*cr + sy*sr;

                R(1,0) = sy*cp;
                R(1,1) = sy*sp*sr + cy*cr;
                R(1,2) = sy*sp*cr - cy*sr;

                R(2,0) = -sp;
                R(2,1) = cp*sr;
                R(2,2) = cp*cr;

                return R;
            }

        } // namespace hand_control
    } // namespace robotics
} // namespace hand_control
