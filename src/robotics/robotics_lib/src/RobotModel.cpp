#include "robotics_lib/RobotModel.h"
using robot::math::Matrix;
using robot::math::Vector;

namespace robot_lib
{

    // Helpers ---------------------------------------------------------

    static inline Vector<3> makeVec3(double x, double y, double z)
    {
        Vector<3> v{};
        v[0] = x;
        v[1] = y;
        v[2] = z;
        return v;
    }

    bool RobotModel::loadFromParameterServer(const merai::ParameterServer &ps)
    {
        // Sanity: must have at least 8 links and 7 joints in the PS
        if (ps.linkCount < NUM_LINKS || ps.jointCount < NUM_JOINTS)
        {
            return false;
        }

        // Gravity (base frame)
        gravity_ = makeVec3(ps.gravity[0], ps.gravity[1], ps.gravity[2]); // TODO (Shubham): Gravity not there in param server

        // ---------------- Links (0..7) ----------------
        for (int i = 0; i < NUM_LINKS; ++i)
        {
            const auto &L = ps.links[i];
            // JSON: mass_kg → PS: L.mass
            linkMasses_[i] = L.mass;

            // JSON: com_m[3] → PS: L.com[3]
            linkCOMs_[i] = Vector<3>({L.com[0], L.com[1], L.com[2]});

            // JSON: inertia_kgm2[6] → PS: L.inertia[6] (ixx, iyy, izz, ixy, ixz, iyz)
            linkInertias_[i] = makeInertiaMatrix(
                L.inertia[0], L.inertia[1], L.inertia[2],
                L.inertia[3], L.inertia[4], L.inertia[5]);
        }

        // Joints
        for (int j = 0; j < NUM_JOINTS; ++j)
        {
            const auto &J = ps.joints[j];

            // Origin pose
            jointOriginPos_[j] = makeVec3(J.origin_pos[0], J.origin_pos[1], J.origin_pos[2]);
            jointOriginRot_[j] = eulerToRotation(J.origin_orient[0],
                                                 J.origin_orient[1],
                                                 J.origin_orient[2]);

            // Axis with direction sign (optionally normalize)
            double ax = static_cast<double>(J.drive.axis_direction) * J.axis[0]; // (Shubham): changed variable axis_xyz to axis as per param server
            double ay = static_cast<double>(J.drive.axis_direction) * J.axis[1];
            double az = static_cast<double>(J.drive.axis_direction) * J.axis[2];
            const double n = std::sqrt(ax * ax + ay * ay + az * az);
            if (n > 0.0)
            {
                ax /= n;
                ay /= n;
                az /= n;
            } // normalize if not unit
            jointAxes_[j] = makeVec3(ax, ay, az);

            // Soft position limits
            jointMinPos_[j] = J.limits.position.min;
            jointMaxPos_[j] = J.limits.position.max;
        }

        return true;
    }

    // Assuming your Matrix is row-major and default-constructs to zero.
    Matrix<3, 3> RobotModel::makeInertiaMatrix(double ixx, double iyy, double izz,
                                               double ixy, double ixz, double iyz) const
    {
        Matrix<3, 3> I{}; // zero
        I(0, 0) = ixx;
        I(1, 1) = iyy;
        I(2, 2) = izz;
        I(0, 1) = I(1, 0) = ixy;
        I(0, 2) = I(2, 0) = ixz;
        I(1, 2) = I(2, 1) = iyz;
        return I;
    }

    Matrix<3, 3> RobotModel::eulerToRotation(double roll, double pitch, double yaw) const
    {
        const double cr = std::cos(roll), sr = std::sin(roll);
        const double cp = std::cos(pitch), sp = std::sin(pitch);
        const double cy = std::cos(yaw), sy = std::sin(yaw);

        Matrix<3, 3> R{};
        // ZYX (yaw-pitch-roll) convention
        R(0, 0) = cy * cp;
        R(0, 1) = cy * sp * sr - sy * cr;
        R(0, 2) = cy * sp * cr + sy * sr;
        R(1, 0) = sy * cp;
        R(1, 1) = sy * sp * sr + cy * cr;
        R(1, 2) = sy * sp * cr - cy * sr;
        R(2, 0) = -sp;
        R(2, 1) = cp * sr;
        R(2, 2) = cp * cr;
        return R;
    }

} // namespace robot_lib
