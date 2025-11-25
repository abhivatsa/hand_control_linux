#pragma once

#include <cmath>
#include "Quaternion.h"
#include "RotationMatrix.h"

namespace seven_axis_robot
{
    namespace math
    {

        /**
         * @brief A simple class for storing roll-pitch-yaw in a ZYX convention.
         * - No dynamic memory
         * - No exceptions
         * - Inline, noexcept
         */
        class EulerAngles
        {
        public:
            double roll;  // rotation about X
            double pitch; // rotation about Y
            double yaw;   // rotation about Z

            constexpr EulerAngles() noexcept : roll(0.0), pitch(0.0), yaw(0.0) {}
            constexpr EulerAngles(double r, double p, double y) noexcept : roll(r), pitch(p), yaw(y) {}

            // ------------------------------------------------------------------
            // FROM
            // ------------------------------------------------------------------

            static EulerAngles fromRotationMatrix(const RotationMatrix &R) noexcept
            {
                // ZYX: yaw-pitch-roll
                EulerAngles ea;
                const auto &m = R.mat;

                // Yaw about Z
                ea.yaw = std::atan2(m(1, 0), m(0, 0));

                // Pitch about Y
                double sinp = -m(2, 0); // negative sign for ZYX convention
                if (sinp > 1.0)
                    sinp = 1.0;
                if (sinp < -1.0)
                    sinp = -1.0;
                ea.pitch = std::asin(sinp);

                // Roll about X
                ea.roll = std::atan2(m(2, 1), m(2, 2));
                return ea;
            }

            static EulerAngles fromQuaternion(const Quaternion &q) noexcept
            {
                RotationMatrix R = q.toRotationMatrix();
                return fromRotationMatrix(R);
            }

            // ------------------------------------------------------------------
            // TO
            // ------------------------------------------------------------------

            RotationMatrix toRotationMatrix() const noexcept
            {
                // ZYX
                double cy = std::cos(yaw);
                double sy = std::sin(yaw);
                double cp = std::cos(pitch);
                double sp = std::sin(pitch);
                double cr = std::cos(roll);
                double sr = std::sin(roll);

                RotationMatrix R;
                auto &m = R.mat;

                // row 0
                m(0, 0) = cy * cp;
                m(0, 1) = cy * sp * sr - sy * cr;
                m(0, 2) = cy * sp * cr + sy * sr;

                // row 1
                m(1, 0) = sy * cp;
                m(1, 1) = sy * sp * sr + cy * cr;
                m(1, 2) = sy * sp * cr - cy * sr;

                // row 2
                m(2, 0) = -sp;
                m(2, 1) = cp * sr;
                m(2, 2) = cp * cr;

                return R;
            }

            Quaternion toQuaternion() const noexcept
            {
                // Euler -> RotationMatrix -> Quaternion
                return Quaternion::fromRotationMatrix(toRotationMatrix().mat);
            }
        };

    } // namespace math
} // namespace seven_axis_robot
