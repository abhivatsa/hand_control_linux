#include "robotics_lib/RobotKinematics.h"
#include <cmath> // sin, cos

namespace robot_lib
{

            RobotKinematics::RobotKinematics(const RobotModel &model)
                : model_(model)
            {
            }

            int RobotKinematics::forwardKinematics(
                const robot::math::Vector<7> &jointAngles,
                robot::math::Matrix<4, 4> &outTransform) const
            {
                using namespace robot::math;

                Matrix<4, 4> Ttotal;
                Ttotal.setIdentity();

                for (int i = 0; i < RobotModel::NUM_JOINTS; ++i)
                {
                    const auto &Rorig = model_.getJointOriginRot(i);
                    const auto &Porig = model_.getJointOriginPos(i);
                    Matrix<4, 4> T_origin = buildHomogeneousTransform(Rorig, Porig);

                    const auto &axis = model_.getJointAxis(i);     // axis in local frame
                    const double angle = jointAngles[i];
                    Matrix<3, 3> R_joint = axisAngleRotation(axis, angle);
                    Matrix<4, 4> T_joint = buildHomogeneousTransform(R_joint, Vector<3>()); // zero translation

                    Matrix<4, 4> T_link = T_origin * T_joint;
                    Ttotal = Ttotal * T_link;
                }

                outTransform = Ttotal;
                return 0;
            }

            int RobotKinematics::computeJacobian(
                const robot::math::Vector<7> &jointAngles,
                robot::math::Matrix<6, 7> &outJacobian) const
            {
                using namespace robot::math;

                outJacobian.setZero();

                // End-effector pose (for p_end)
                Matrix<4,4> T_end;
                int ret = forwardKinematics(jointAngles, T_end);
                if (ret != 0) return ret;

                Vector<3> p_end{{ T_end(0,3), T_end(1,3), T_end(2,3) }};

                // Accumulate transforms as we walk the chain
                Matrix<4,4> T_total;       // transform from base to current link frame
                T_total.setIdentity();

                for (int i = 0; i < RobotModel::NUM_JOINTS; ++i)
                {
                    // Fixed origin of joint i (from your model)
                    const auto &Rorig = model_.getJointOriginRot(i);
                    const auto &Porig = model_.getJointOriginPos(i);
                    Matrix<4,4> T_origin = buildHomogeneousTransform(Rorig, Porig);

                    // ----- Transform BEFORE applying joint i's rotation -----
                    Matrix<4,4> T_before = T_total * T_origin;

                    // Joint origin position in base
                    Vector<3> p_i{{ T_before(0,3), T_before(1,3), T_before(2,3) }};

                    // Joint axis in local joint frame (already direction-correct & normalized in your model)
                    const auto &a_loc = model_.getJointAxis(i);

                    // Rotate axis into base frame: a_world = R_before * a_loc
                    Vector<3> a_world{};
                    a_world[0] = T_before(0,0)*a_loc[0] + T_before(0,1)*a_loc[1] + T_before(0,2)*a_loc[2];
                    a_world[1] = T_before(1,0)*a_loc[0] + T_before(1,1)*a_loc[1] + T_before(1,2)*a_loc[2];
                    a_world[2] = T_before(2,0)*a_loc[0] + T_before(2,1)*a_loc[1] + T_before(2,2)*a_loc[2];

                    // Column i of the geometric Jacobian (revolute joint)
                    Vector<3> diff{{ p_end[0]-p_i[0], p_end[1]-p_i[1], p_end[2]-p_i[2] }};
                    Vector<3> Jv = a_world.cross(diff); // linear part
                    Vector<3> Jw = a_world;             // angular part

                    for (int r = 0; r < 3; ++r) {
                        outJacobian(r,     i) = Jv[r];
                        outJacobian(r + 3, i) = Jw[r];
                    }

                    // ----- Advance T_total by this joint's actual rotation to continue the chain -----
                    const double qi = jointAngles[i];
                    Matrix<3,3> R_joint = axisAngleRotation(a_loc, qi);
                    Matrix<4,4> T_joint = buildHomogeneousTransform(R_joint, Vector<3>()); // no translation at the joint itself
                    T_total = T_before * T_joint;
                }

                return 0;
            }


            int RobotKinematics::computeInverseKinematics(
                const robot::math::Vector<7> &currentJointAngles,
                const robot::math::Matrix<4, 4> & /*desiredPose*/,
                robot::math::Vector<7> &outJointAngles) const
            {
                // Placeholder: copy-through
                outJointAngles = currentJointAngles;
                return 0;
            }

            // --- helpers -------------------------------------------------------

            robot::math::Matrix<4, 4> RobotKinematics::buildHomogeneousTransform(
                const robot::math::Matrix<3, 3> &rot,
                const robot::math::Vector<3> &pos) const
            {
                using namespace robot::math;
                Matrix<4, 4> T;
                T.setIdentity();
                for (std::size_t r = 0; r < 3; ++r)
                {
                    for (std::size_t c = 0; c < 3; ++c)
                    {
                        T(r, c) = rot(r, c);
                    }
                }
                T(0, 3) = pos[0];
                T(1, 3) = pos[1];
                T(2, 3) = pos[2];
                return T;
            }

            robot::math::Matrix<3, 3> RobotKinematics::axisAngleRotation(
                const robot::math::Vector<3> &axis,
                double angle) const
            {
                using namespace robot::math;

                const double n = axis.norm();
                if (n < 1e-12) {
                    Matrix<3,3> I; I.setIdentity();
                    return I;
                }
                const Vector<3> a = axis / n;
                const double x = a[0], y = a[1], z = a[2];

                const double c = std::cos(angle);
                const double s = std::sin(angle);
                const double omc = 1.0 - c;

                Matrix<3, 3> R;
                R(0, 0) = c + x * x * omc;
                R(0, 1) = x * y * omc - z * s;
                R(0, 2) = x * z * omc + y * s;
                R(1, 0) = y * x * omc + z * s;
                R(1, 1) = c + y * y * omc;
                R(1, 2) = y * z * omc - x * s;
                R(2, 0) = z * x * omc - y * s;
                R(2, 1) = z * y * omc + x * s;
                R(2, 2) = c + z * z * omc;

                return R;
            }

} // namespace robot_lib
