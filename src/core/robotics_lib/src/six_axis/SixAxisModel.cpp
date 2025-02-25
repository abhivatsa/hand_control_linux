#include "robotics_lib/six_axis/SixAxisModel.h"
#include <cmath> // for sin, cos

namespace motion_control
{
    namespace robotics
    {
        namespace six_axis
        {

            bool SixAxisModel::loadFromParameterServer(const merai::ParameterServer &ps)
            {
                // Must have at least 6 links & 6 joints
                if (ps.linkCount < NUM_LINKS || ps.jointCount < NUM_JOINTS)
                {
                    return false;
                }

                // Robot name
                robotName_ = ps.robot_name.c_str();

                // ------------------------
                // A) Load Link Data
                // ------------------------
                for (int i = 0; i < NUM_LINKS; ++i)
                {
                    const auto &linkCfg = ps.links[i];

                    // Mass
                    linkMasses_[i] = linkCfg.mass;

                    // Center of mass => Vector<3>
                    linkCOMs_[i].data[0] = linkCfg.center_of_mass[0];
                    linkCOMs_[i].data[1] = linkCfg.center_of_mass[1];
                    linkCOMs_[i].data[2] = linkCfg.center_of_mass[2];

                    // Inertia => 3x3 matrix
                    linkInertias_[i] = makeInertiaMatrix(
                        linkCfg.inertia.ixx,
                        linkCfg.inertia.iyy,
                        linkCfg.inertia.izz,
                        linkCfg.inertia.ixy,
                        linkCfg.inertia.ixz,
                        linkCfg.inertia.iyz);
                }

                // ------------------------
                // B) Load Joint Data
                // ------------------------
                for (int j = 0; j < NUM_JOINTS; ++j)
                {
                    const auto &jc = ps.joints[j];

                    // Parent/Child
                    jointParents_[j] = jc.parent.c_str();
                    jointChildren_[j] = jc.child.c_str();

                    // 1) Origin position => Vector<3>
                    jointOriginPos_[j].data[0] = jc.origin.position[0];
                    jointOriginPos_[j].data[1] = jc.origin.position[1];
                    jointOriginPos_[j].data[2] = jc.origin.position[2];

                    // 2) Origin orientation => Euler angles -> Matrix<3,3>
                    double r = jc.origin.orientation[0];
                    double p = jc.origin.orientation[1];
                    double y = jc.origin.orientation[2];
                    jointOriginRot_[j] = eulerToRotation(r, p, y);

                    // 3) Joint axis => Vector<3>
                    jointAxes_[j].data[0] = jc.axis[0];
                    jointAxes_[j].data[1] = jc.axis[1];
                    jointAxes_[j].data[2] = jc.axis[2];

                    // 4) Min/Max => store in jointMinPos_, jointMaxPos_ (both are Vector<6>)
                    jointMinPos_.data[j] = jc.parameters.joint_position_limits.min;
                    jointMaxPos_.data[j] = jc.parameters.joint_position_limits.max;
                }

                return true;
            }

            /**
             * @brief Utility to build a symmetric 3x3 inertia matrix from
             *        ixx, iyy, izz, ixy, ixz, iyz.
             */
            motion_control::math::Matrix<3, 3> SixAxisModel::makeInertiaMatrix(
                double ixx, double iyy, double izz,
                double ixy, double ixz, double iyz) const
            {
                using Matrix3x3 = motion_control::math::Matrix<3, 3>;

                Matrix3x3 M;
                M.data[0][0] = ixx;
                M.data[0][1] = ixy;
                M.data[0][2] = ixz;

                M.data[1][0] = ixy;
                M.data[1][1] = iyy;
                M.data[1][2] = iyz;

                M.data[2][0] = ixz;
                M.data[2][1] = iyz;
                M.data[2][2] = izz;
                return M;
            }

            /**
             * @brief Utility: convert Euler angles (roll, pitch, yaw)
             *        to a 3x3 rotation matrix (row-major).
             */
            motion_control::math::Matrix<3, 3> SixAxisModel::eulerToRotation(
                double roll, double pitch, double yaw) const
            {
                using Matrix3x3 = motion_control::math::Matrix<3, 3>;

                double sr = std::sin(roll);
                double cr = std::cos(roll);
                double sp = std::sin(pitch);
                double cp = std::cos(pitch);
                double sy = std::sin(yaw);
                double cy = std::cos(yaw);

                // We'll build R_z(yaw) * R_y(pitch) * R_x(roll), or whichever convention you prefer.
                // For example (Z-Y-X):
                Matrix3x3 R;
                R.data[0][0] = cy * cp;
                R.data[0][1] = cy * sp * sr - sy * cr;
                R.data[0][2] = cy * sp * cr + sy * sr;

                R.data[1][0] = sy * cp;
                R.data[1][1] = sy * sp * sr + cy * cr;
                R.data[1][2] = sy * sp * cr - cy * sr;

                R.data[2][0] = -sp;
                R.data[2][1] = cp * sr;
                R.data[2][2] = cp * cr;

                return R;
            }

        } // namespace six_axis
    } // namespace robotics
} // namespace motion_control
