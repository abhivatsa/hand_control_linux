#include "robotics_lib/RobotDynamics.h"
#include "math_lib/MatrixVectorOps.h"
#include <cmath>

namespace robot_lib
{

    using robot::math::Matrix;
    using robot::math::Vector;

    RobotDynamics::RobotDynamics(const RobotModel &model)
        : model_(model) {}

    int RobotDynamics::computeInverseDynamics(const Vector<7> &q,
                                              const Vector<7> &dq,
                                              const Vector<7> &ddq,
                                              Vector<7> &outTau) const
    {
        // Forward/Backward Newton–Euler skeleton (kept close to your working code)
        // Base accel = gravity from model (be explicit about direction).
        std::array<Vector<3>, RobotModel::NUM_LINKS> w, wd, vd;
        for (auto &v : w)
            v.setZero();
        for (auto &v : wd)
            v.setZero();
        for (auto &v : vd)
            v.setZero();

        // gravity in base frame
        const auto &g = model_.gravity(); // Vector<3>, e.g. [0,0,-9.80665]
        vd[0] = g;

        for (int i = 0; i < RobotModel::NUM_JOINTS; ++i)
        {
            Matrix<4, 4> T = computeJointTransform(i, q[i]);

            // Extract rotation child->parent (R_cp). Your original code used R^T inconsistently;
            // here we stick to “express child in parent”: parent_R_child = R.
            Matrix<3, 3> R{};
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    R(r, c) = T(r, c);

            const Vector<3> z = model_.getJointAxis(i).normalized();

            // w, wd (expressed in child frame)
            const Vector<3> w_parent = R.transpose() * w[i];
            const Vector<3> wd_parent = R.transpose() * wd[i];

            w[i + 1] = w_parent + dq[i] * z;
            wd[i + 1] = wd_parent + R.transpose() * (w[i].cross(dq[i] * z)) + ddq[i] * z;

            const Vector<3> r{T(0, 3), T(1, 3), T(2, 3)};
            vd[i + 1] = R.transpose() * (vd[i] + wd[i].cross(r) + w[i].cross(w[i].cross(r)));
        }

        std::array<Vector<3>, RobotModel::NUM_LINKS> F, N;
        for (auto &v : F)
            v.setZero();
        for (auto &v : N)
            v.setZero();

        // Tip loads zero
        for (int i = RobotModel::NUM_LINKS - 1; i >= 0; --i)
        {
            const double m = model_.getLinkMass(i);
            const Matrix<3, 3> &I = model_.getLinkInertia(i);
            const Vector<3> &com = model_.getLinkCOM(i);

            const Vector<3> vdot_c = vd[i] + wd[i].cross(com) + w[i].cross(w[i].cross(com));
            const Vector<3> Fi = m * vdot_c;
            const Vector<3> Iw = I * w[i];
            const Vector<3> Ni = I * wd[i] + w[i].cross(Iw);

            // child->parent rotation at (i) (reuse T of joint i if you cache; recompute here)
            if (i < RobotModel::NUM_JOINTS)
            {
                Matrix<4, 4> T = computeJointTransform(i, q[i]);
                Matrix<3, 3> R{};
                for (int r = 0; r < 3; ++r)
                    for (int c = 0; c < 3; ++c)
                        R(r, c) = T(r, c);

                const Vector<3> r{T(0, 3), T(1, 3), T(2, 3)};
                // bring child forces to parent
                Vector<3> Fchild = R * F[i + 1];
                Vector<3> Nchild = R * N[i + 1];

                F[i] = Fi + Fchild;
                N[i] = Ni + Nchild + com.cross(Fi) + r.cross(Fchild);
            }
            else
            {
                // last link (tool)
                F[i] = Fi;
                N[i] = Ni + com.cross(Fi);
            }
        }

        for (int i = 0; i < RobotModel::NUM_JOINTS; ++i)
        {
            const Vector<3> z = model_.getJointAxis(i).normalized();
            outTau[i] = N[i].dot(z);
        }
        return 0;
    }

    Matrix<4, 4> RobotDynamics::computeJointTransform(int jointIdx, double jointAngle) const
    {
        const auto &R0 = model_.getJointOriginRot(jointIdx); // 3x3
        const auto &p0 = model_.getJointOriginPos(jointIdx); // Vector<3>

        Matrix<4, 4> T0;
        T0.setIdentity();
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c)
                T0(r, c) = R0(r, c);
            T0(r, 3) = p0[r];
        }

        const auto axis = model_.getJointAxis(jointIdx).normalized();
        const double c = std::cos(jointAngle), s = std::sin(jointAngle), x = axis[0], y = axis[1], z = axis[2], mc = 1.0 - c;

        Matrix<3, 3> Rj;
        Rj(0, 0) = c + x * x * mc;
        Rj(0, 1) = x * y * mc - z * s;
        Rj(0, 2) = x * z * mc + y * s;
        Rj(1, 0) = y * x * mc + z * s;
        Rj(1, 1) = c + y * y * mc;
        Rj(1, 2) = y * z * mc - x * s;
        Rj(2, 0) = z * x * mc - y * s;
        Rj(2, 1) = z * y * mc + x * s;
        Rj(2, 2) = c + z * z * mc;

        Matrix<4, 4> Tj;
        Tj.setIdentity();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                Tj(r, c) = Rj(r, c);

        return T0 * Tj;
    }

} // namespace robot_lib
