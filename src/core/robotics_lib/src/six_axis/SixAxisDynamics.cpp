#include "robotics_lib/six_axis/SixAxisDynamics.h"
#include <cmath> // for sin, cos

namespace motion_control
{
    namespace robotics
    {
        namespace six_axis
        {

            SixAxisDynamics::SixAxisDynamics(const SixAxisModel &model)
                : model_(model)
            {
                // If you want to precompute or cache anything from model_, do it here.
            }

            int SixAxisDynamics::computeInverseDynamics(
                const math::Vector<6> &jointAngles,
                const math::Vector<6> &jointVel,
                const math::Vector<6> &jointAcc,
                math::Vector<6> &outTorques) const
            {
                using namespace motion_control::math;

                // -------------------------------------------------------
                // 1) Forward Recursion: compute velocities/accelerations
                // -------------------------------------------------------
                // We'll keep track of angular velocity/accel, linear accel, etc. at each link i.
                // For a 6-axis chain, we have arrays of Vector<3> for w(i), w_dot(i), v_dot(i).
                std::array<Vector<3>, SixAxisModel::NUM_LINKS + 1> angVel;   // w
                std::array<Vector<3>, SixAxisModel::NUM_LINKS + 1> angAccel; // w_dot
                std::array<Vector<3>, SixAxisModel::NUM_LINKS + 1> linAccel; // v_dot

                // Initialize base link 0 with no velocity, no accel, gravity in negative Z
                angVel[0] = Vector<3>();                // [0,0,0]
                angAccel[0] = Vector<3>();              // [0,0,0]
                linAccel[0] = Vector<3>({0, 0, -9.81}); // example gravity

                for (int i = 0; i < SixAxisModel::NUM_JOINTS; ++i)
                {
                    // Build transform for this joint with the current angle
                    Matrix<4, 4> T_i = computeJointTransform(i, jointAngles[i]);

                    // Extract rotation (3×3) from T_i, so we can transform velocities
                    Matrix<3, 3> R_i; // rotation from link (i) to link (i+1)
                    for (int r = 0; r < 3; ++r)
                    {
                        for (int c = 0; c < 3; ++c)
                        {
                            R_i(r, c) = T_i(r, c);
                        }
                    }
                    // The z-axis for this joint's rotation in local coords could be the 3rd column of R_i if your axis is local Z.
                    // But we also store the axis in the model, so you might do axis-angle instead.

                    // Now update the angular velocity: w(i+1) = R_i * w(i) + jointVel[i] * <jointAxis in i+1 coords>
                    // For simplicity, if your axis is local Z, you might do:
                    Vector<3> zAxisInChild = {0, 0, 1}; // if that's your convention
                    // Or transform the axis from the model if needed.

                    // Example approach (like your old code, revolve around local z):
                    angVel[i + 1] = R_i * angVel[i] + (jointVel[i] * zAxisInChild);

                    // Angular accel w_dot(i+1)
                    angAccel[i + 1] = R_i * angAccel[i] + R_i * angVel[i].cross(jointVel[i] * zAxisInChild) + (jointAcc[i] * zAxisInChild);

                    // Linear accel v_dot(i+1). We can compute the offset from T_i.
                    Vector<3> r_i; // translation from T_i
                    r_i[0] = T_i(0, 3);
                    r_i[1] = T_i(1, 3);
                    r_i[2] = T_i(2, 3);

                    // v_dot(i+1) = R_i*( w_dot(i) x r_i + w(i) x (w(i) x r_i) + v_dot(i) )
                    Vector<3> crossTerm = angVel[i].cross(angVel[i].cross(r_i));
                    Vector<3> crossTerm2 = angAccel[i].cross(r_i);
                    linAccel[i + 1] = R_i * (crossTerm2 + crossTerm + linAccel[i]);
                }

                // We'll also store linear accel at the COM of each link if needed. This can be computed in the same loop
                // using w_dot(i+1) cross linkCOM, etc. Or do it in next step.

                // -------------------------------------------------------
                // 2) Backward Recursion: compute forces/torques
                // -------------------------------------------------------
                // We'll accumulate forces, torques from the last link downward.
                // force[i], torque[i] => net force/torque at link i.
                std::array<Vector<3>, SixAxisModel::NUM_LINKS + 1> force;
                std::array<Vector<3>, SixAxisModel::NUM_LINKS + 1> torque;

                // Initialize the "tip" force/torque to zero
                force[SixAxisModel::NUM_LINKS] = Vector<3>(); // e.g. no load at tip
                torque[SixAxisModel::NUM_LINKS] = Vector<3>();

                for (int i = SixAxisModel::NUM_LINKS - 1; i >= 0; --i)
                {
                    // In your code, you did "rotation_mat.transpose()" for the backward step, plus you added the child's force.
                    // We'll do something similar:

                    // First, get the link mass + inertia:
                    double linkMass = model_.getLinkMass(i);
                    const auto &I_i = model_.getLinkInertia(i); // 3×3 inertia
                    const auto &COM = model_.getLinkCOM(i);     // Vector<3> com offset in link coords

                    // Suppose we want the linear accel of the COM:
                    // v_dot_com(i) = w_dot(i+1) x COM + w(i+1) x (w(i+1) x COM) + v_dot(i+1).
                    Vector<3> cross1 = angAccel[i + 1].cross(COM);
                    Vector<3> cross2 = angVel[i + 1].cross(angVel[i + 1].cross(COM));
                    Vector<3> v_dot_com = cross1 + cross2 + linAccel[i + 1];

                    // Force = m * v_dot_com
                    Vector<3> F_i = linkMass * v_dot_com;

                    // Torque about link origin:
                    // Tau_i = I_i * w_dot(i+1) + w(i+1) x (I_i * w(i+1))
                    Vector<3> Iw = I_i * angVel[i + 1];
                    Vector<3> T_i = (I_i * angAccel[i + 1]) + angVel[i + 1].cross(Iw);

                    // Now combine with child link's force/torque
                    // We'll need the transform from link i to i+1 again:
                    Matrix<4, 4> T_i = computeJointTransform(i, jointAngles[i]);
                    Matrix<3, 3> R_i;
                    for (int r = 0; r < 3; ++r)
                    {
                        for (int c = 0; c < 3; ++c)
                        {
                            R_i(r, c) = T_i(r, c);
                        }
                    }
                    // For backward pass, we typically do "force[i] = R_i^T * force[i+1] + F_i"
                    // But we also have to account for the offset from link i to i+1.
                    Vector<3> F_child_rot = R_i.transpose() * force[i + 1];
                    Vector<3> T_child_rot = R_i.transpose() * torque[i + 1];

                    force[i] = F_child_rot + F_i;

                    // torque[i] = T_i + R_i^T * torque[i+1]
                    //           + cross( COM, F_i ) + cross( r_i_(i->i+1), R_i^T * force[i+1] )
                    // We'll fetch the same r_i used in forward pass:
                    Vector<3> r_i;
                    r_i[0] = T_i(0, 3);
                    r_i[1] = T_i(1, 3);
                    r_i[2] = T_i(2, 3);

                    Vector<3> torqueCOM = COM.cross(F_i);
                    Vector<3> torqueChild = r_i.cross(F_child_rot);

                    torque[i] = T_i + T_child_rot + torqueCOM + torqueChild;

                    // Now the joint torque is along the joint axis in local or global coords.
                    // If your manipulator rotates about local Z, you might extract torque[i].z().
                    // Or if you prefer the dot product with the axis i:
                    Vector<3> jointAxis = model_.getJointAxis(i).normalized();
                    outTorques[i] = torque[i].dot(jointAxis);
                }

                return 0; // success
            }

            motion_control::math::Matrix<4, 4> SixAxisDynamics::computeJointTransform(
                int jointIdx,
                double jointAngle) const
            {
                using namespace motion_control::math;

                // 1) Combine the "origin transform" for this joint (from the model)
                //    with a rotation about the stored axis by 'jointAngle'.
                const auto &Rorig = model_.getJointOriginRot(jointIdx); // 3×3
                const auto &Porig = model_.getJointOriginPos(jointIdx); // Vector<3>

                // Build a 4×4 from Rorig, Porig
                Matrix<4, 4> T_origin;
                T_origin.setIdentity();
                for (int r = 0; r < 3; ++r)
                {
                    for (int c = 0; c < 3; ++c)
                    {
                        T_origin(r, c) = Rorig(r, c);
                    }
                    T_origin(r, 3) = Porig[r];
                }

                // 2) Rotation about joint's axis
                Matrix<3, 3> R_joint;
                // e.g. use Rodrigues formula or a simpler approach if you know axis is z. We'll do a quick approach:
                Vector<3> axis = model_.getJointAxis(jointIdx).normalized();
                double c = std::cos(jointAngle);
                double s = std::sin(jointAngle);
                double x = axis[0], y = axis[1], z = axis[2];
                double one_c = 1.0 - c;

                R_joint(0, 0) = c + x * x * one_c;
                R_joint(0, 1) = x * y * one_c - z * s;
                R_joint(0, 2) = x * z * one_c + y * s;
                R_joint(1, 0) = y * x * one_c + z * s;
                R_joint(1, 1) = c + y * y * one_c;
                R_joint(1, 2) = y * z * one_c - x * s;
                R_joint(2, 0) = z * x * one_c - y * s;
                R_joint(2, 1) = z * y * one_c + x * s;
                R_joint(2, 2) = c + z * z * one_c;

                Matrix<4, 4> T_joint;
                T_joint.setIdentity();
                for (int r = 0; r < 3; ++r)
                {
                    for (int c = 0; c < 3; ++c)
                    {
                        T_joint(r, c) = R_joint(r, c);
                    }
                }

                // Combine them: T = T_origin * T_joint
                return (T_origin * T_joint);
            }

        } // namespace six_axis
    } // namespace robotics
} // namespace motion_control
