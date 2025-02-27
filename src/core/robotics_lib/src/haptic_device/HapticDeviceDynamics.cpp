#include "robotics_lib/haptic_device/HapticDeviceDynamics.h"
#include "math_lib/MatrixVectorOps.h"   // Needed if you use operator*(Matrix,Vector)
#include <cmath> // for sin, cos

namespace hand_control
{
    namespace robotics
    {
        namespace haptic_device
        {

            using namespace hand_control::math;

            HapticDeviceDynamics::HapticDeviceDynamics(const HapticDeviceModel &model)
                : model_(model)
            {
                // If you want to precompute or cache anything from model_, do it here.
            }

            int HapticDeviceDynamics::computeInverseDynamics(
                const Vector<6> &jointAngles,
                const Vector<6> &jointVel,
                const Vector<6> &jointAcc,
                Vector<6> &outTorques) const
            {
                // -------------------------------------------------------
                // This is just a skeleton or example approach for a chain
                // with 6 degrees of freedom.
                // Typically, you'd have:
                //    - Forward recursion to compute velocities/accelerations
                //    - Backward recursion to compute forces/torques.
                // -------------------------------------------------------

                // We'll keep track of angular velocity/accel, linear accel for each link i.
                // For a 6-joint chain, store them in arrays of Vector<3> for w(i), w_dot(i), etc.
                std::array<Vector<3>, HapticDeviceModel::NUM_LINKS + 1> angVel;   // w
                std::array<Vector<3>, HapticDeviceModel::NUM_LINKS + 1> angAccel; // w_dot
                std::array<Vector<3>, HapticDeviceModel::NUM_LINKS + 1> linAccel; // v_dot

                // Base link (i=0) with no velocity or accel, but gravity in negative Z:
                angVel[0].setZero();   // [0,0,0]
                angAccel[0].setZero(); // [0,0,0]
                linAccel[0] = Vector<3>({0.0, 0.0, -9.81});

                // -------------------------------------------------------
                // Forward Recursion
                // -------------------------------------------------------
                for (int i = 0; i < HapticDeviceModel::NUM_JOINTS; ++i)
                {
                    // 1) Compute the transform from link i to (i+1) given jointAngles[i]
                    Matrix<4, 4> T_i = computeJointTransform(i, jointAngles[i]);

                    // Extract the top-left 3×3 rotation
                    Matrix<3, 3> R_i;
                    for (int r = 0; r < 3; ++r)
                    {
                        for (int c = 0; c < 3; ++c)
                        {
                            R_i(r, c) = T_i(r, c);
                        }
                    }

                    // If your joints don't always align with local Z, use your model's axis:
                    // Vector<3> zAxisInChild = model_.getJointAxis(i).normalized();
                    // For illustration (assuming local Z = rotation axis):
                    Vector<3> zAxisInChild = {0.0, 0.0, 1.0};

                    // 2) Angular velocity: w(i+1) = R_i * w(i) + (jointVel[i] * zAxisInChild)
                    angVel[i + 1] = R_i * angVel[i] + (jointVel[i] * zAxisInChild);

                    // 3) Angular accel:
                    //    w_dot(i+1) = R_i * w_dot(i)
                    //                 + R_i * (w(i) × (jointVel[i] * zAxisInChild))
                    //                 + (jointAcc[i] * zAxisInChild)
                    angAccel[i + 1] =
                        R_i * angAccel[i]
                        + R_i * (angVel[i].cross(jointVel[i] * zAxisInChild))
                        + (jointAcc[i] * zAxisInChild);

                    // 4) Linear accel:
                    //    v_dot(i+1) = R_i * (v_dot(i) + w_dot(i)×r + w(i)×(w(i)×r))
                    //    where r is the offset from T_i(0..2,3)
                    Vector<3> r_i{
                        T_i(0, 3),
                        T_i(1, 3),
                        T_i(2, 3)
                    };

                    Vector<3> crossTerm = angVel[i].cross(angVel[i].cross(r_i));
                    Vector<3> crossTerm2 = angAccel[i].cross(r_i);

                    linAccel[i + 1] = R_i * (linAccel[i] + crossTerm + crossTerm2);
                }

                // -------------------------------------------------------
                // Backward Recursion
                // -------------------------------------------------------
                std::array<Vector<3>, HapticDeviceModel::NUM_LINKS + 1> force;
                std::array<Vector<3>, HapticDeviceModel::NUM_LINKS + 1> torque;

                // Tip (link 6) has no external load in this example:
                force[HapticDeviceModel::NUM_LINKS].setZero();
                torque[HapticDeviceModel::NUM_LINKS].setZero();

                for (int i = HapticDeviceModel::NUM_LINKS - 1; i >= 0; --i)
                {
                    double linkMass = model_.getLinkMass(i);
                    const auto &I_i = model_.getLinkInertia(i); // 3x3 inertia
                    const auto &COM = model_.getLinkCOM(i);     // Vector<3> center of mass

                    // accel at COM: v_dot_com(i) = w_dot(i+1)×COM
                    //               + w(i+1)×( w(i+1)×COM )
                    //               + v_dot(i+1)
                    Vector<3> v_dot_com =
                        angAccel[i + 1].cross(COM)
                        + angVel[i + 1].cross(angVel[i + 1].cross(COM))
                        + linAccel[i + 1];

                    // Force = m * accel(COM)
                    Vector<3> F_i = linkMass * v_dot_com;

                    // Torque about link origin:
                    // T_i = I_i*w_dot(i+1) + w(i+1)×(I_i*w(i+1))
                    Vector<3> Iw = I_i * angVel[i + 1];
                    Vector<3> T_i = (I_i * angAccel[i + 1]) + angVel[i + 1].cross(Iw);

                    // Transform child's force/torque back to link i frame
                    Matrix<4, 4> T_i_fwd = computeJointTransform(i, jointAngles[i]);
                    Matrix<3, 3> R_i{};
                    for (int r = 0; r < 3; ++r)
                    {
                        for (int c = 0; c < 3; ++c)
                        {
                            R_i(r, c) = T_i_fwd(r, c);
                        }
                    }
                    Vector<3> F_child = R_i.transpose() * force[i + 1];
                    Vector<3> T_child = R_i.transpose() * torque[i + 1];

                    // Combine
                    force[i] = F_child + F_i;

                    // Add the torques from COM offset and transform offset
                    Vector<3> r_i{
                        T_i_fwd(0, 3),
                        T_i_fwd(1, 3),
                        T_i_fwd(2, 3)
                    };

                    Vector<3> torqueCOM = COM.cross(F_i);
                    Vector<3> torqueChild = r_i.cross(F_child);

                    torque[i] = T_i + T_child + torqueCOM + torqueChild;

                    // Finally, project onto joint axis for the actual motor torque
                    Vector<3> jointAxis = model_.getJointAxis(i).normalized();
                    outTorques[i] = torque[i].dot(jointAxis);
                }

                return 0; // success
            }

            Matrix<4, 4> HapticDeviceDynamics::computeJointTransform(
                int jointIdx,
                double jointAngle) const
            {
                // 1) Get the joint's origin transform from the model
                const auto &Rorig = model_.getJointOriginRot(jointIdx); // 3x3
                const auto &Porig = model_.getJointOriginPos(jointIdx); // Vector<3>

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

                // 2) Rotation about the joint axis by 'jointAngle'
                Vector<3> axis = model_.getJointAxis(jointIdx).normalized();
                double c = std::cos(jointAngle);
                double s = std::sin(jointAngle);
                double x = axis[0], y = axis[1], z = axis[2];
                double one_c = 1.0 - c;

                Matrix<3, 3> R_joint;
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

                // 3) Final transform = origin transform * rotation about axis
                Matrix<4, 4> T = T_origin * T_joint;
                return T;
            }

        } // namespace hand_control
    } // namespace robotics
} // namespace hand_control
