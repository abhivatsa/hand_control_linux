#pragma once

#include "robotics_lib/six_axis/SixAxisModel.h"
#include "math_lib/Matrix.h"
#include "math_lib/Vector.h"

namespace motion_control
{
    namespace robotics
    {
        namespace six_axis
        {

            /**
             * @brief Example Newton-Euler-based dynamics computations for a 6-axis robot.
             */
            class SixAxisDynamics
            {
            public:
                explicit SixAxisDynamics(const SixAxisModel &model);
                ~SixAxisDynamics() = default;

                /**
                 * @brief General Newton-Euler inverse dynamics
                 */
                int computeInverseDynamics(const math::Vector<6> &jointAngles,
                                           const math::Vector<6> &jointVel,
                                           const math::Vector<6> &jointAcc,
                                           math::Vector<6> &outTorques) const;

                // /**
                //  * @brief Mass (inertia) matrix at a given configuration.
                //  * @param jointAngles
                //  * @param outMassMatrix (6×6)
                //  */
                // void computeMassMatrix(const math::Vector<6> &jointAngles,
                //                        math::Matrix<6, 6> &outMassMatrix) const;

                // /**
                //  * @brief Coriolis + centripetal terms c(q, qdot).
                //  *        Typically a 6×1 vector that, combined with your joint velocities, yields the extra forces.
                //  */
                // void computeCoriolisTerm(const math::Vector<6> &jointAngles,
                //                          const math::Vector<6> &jointVel,
                //                          math::Vector<6> &outCoriolis) const;

                // /**
                //  * @brief Gravity torques at a given configuration g(q).
                //  * @param jointAngles
                //  * @param outGravity a 6×1 vector
                //  */
                // void computeGravityTerm(const math::Vector<6> &jointAngles,
                //                         math::Vector<6> &outGravity) const;

            private:
                // Reuse any helper functions from your existing inverse dynamics
                math::Matrix<4, 4> computeJointTransform(int jointIdx, double jointAngle) const;
                // etc...

            private:
                const SixAxisModel &model_;
            };

        } // namespace six_axis
    } // namespace robotics
} // namespace motion_control
