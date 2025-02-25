#pragma once

#include <cmath>    // sqrt, etc.
#include "Matrix.h" // We'll use Matrix<3,3>
#include "Vector.h" // if needed

namespace motion_control
{
    namespace math
    {

        /**
         * @brief A basic Quaternion class (x, y, z, w).
         * - No dynamic memory
         * - No exceptions
         * - Real-time friendly
         */
        class Quaternion
        {
        public:
            double x;
            double y;
            double z;
            double w;

            constexpr Quaternion() noexcept : x(0.0), y(0.0), z(0.0), w(1.0) {}
            constexpr Quaternion(double xVal, double yVal, double zVal, double wVal) noexcept
                : x(xVal), y(yVal), z(zVal), w(wVal)
            {
            }

            // ------------------------------------------------------------------
            // Magnitude and Normalization
            // ------------------------------------------------------------------

            double norm() const noexcept
            {
                return std::sqrt(x * x + y * y + z * z + w * w);
            }

            Quaternion normalized() const noexcept
            {
                double n = norm();
                if (n < 1e-12)
                {
                    return *this; // fallback
                }
                double inv = 1.0 / n;
                return Quaternion(x * inv, y * inv, z * inv, w * inv);
            }

            // ------------------------------------------------------------------
            // Basic Operations
            // ------------------------------------------------------------------

            Quaternion operator*(const Quaternion &other) const noexcept
            {
                // Hamilton product
                return Quaternion(
                    w * other.x + x * other.w + y * other.z - z * other.y, // x
                    w * other.y - x * other.z + y * other.w + z * other.x, // y
                    w * other.z + x * other.y - y * other.x + z * other.w, // z
                    w * other.w - x * other.x - y * other.y - z * other.z  // w
                );
            }

            Quaternion conjugate() const noexcept
            {
                return Quaternion(-x, -y, -z, w);
            }

            Quaternion inverse() const noexcept
            {
                double n2 = x * x + y * y + z * z + w * w;
                if (n2 < 1e-12)
                {
                    return *this; // fallback
                }
                double inv = 1.0 / n2;
                return Quaternion(-x * inv, -y * inv, -z * inv, w * inv);
            }

            // ------------------------------------------------------------------
            // Rotation Matrix Conversions
            // ------------------------------------------------------------------

            static Quaternion fromRotationMatrix(const Matrix<3, 3> &R) noexcept
            {
                Quaternion q;
                double trace = R(0, 0) + R(1, 1) + R(2, 2);

                if (trace > 0.0)
                {
                    double s = 0.5 / std::sqrt(trace + 1.0);
                    q.w = 0.25 / s;
                    q.x = (R(2, 1) - R(1, 2)) * s;
                    q.y = (R(0, 2) - R(2, 0)) * s;
                    q.z = (R(1, 0) - R(0, 1)) * s;
                }
                else
                {
                    if (R(0, 0) > R(1, 1) && R(0, 0) > R(2, 2))
                    {
                        double s = 2.0 * std::sqrt(std::max(0.0, R(0, 0) - R(1, 1) - R(2, 2) + 1.0));
                        q.x = 0.25 * s;
                        q.y = (R(0, 1) + R(1, 0)) / s;
                        q.z = (R(0, 2) + R(2, 0)) / s;
                        q.w = (R(2, 1) - R(1, 2)) / s;
                    }
                    else if (R(1, 1) > R(2, 2))
                    {
                        double s = 2.0 * std::sqrt(std::max(0.0, R(1, 1) - R(0, 0) - R(2, 2) + 1.0));
                        q.x = (R(0, 1) + R(1, 0)) / s;
                        q.y = 0.25 * s;
                        q.z = (R(1, 2) + R(2, 1)) / s;
                        q.w = (R(0, 2) - R(2, 0)) / s;
                    }
                    else
                    {
                        double s = 2.0 * std::sqrt(std::max(0.0, R(2, 2) - R(0, 0) - R(1, 1) + 1.0));
                        q.x = (R(0, 2) + R(2, 0)) / s;
                        q.y = (R(1, 2) + R(2, 1)) / s;
                        q.z = 0.25 * s;
                        q.w = (R(1, 0) - R(0, 1)) / s;
                    }
                }
                return q.normalized();
            }

            Matrix<3, 3> toRotationMatrix() const noexcept
            {
                Matrix<3, 3> R;
                double xx = x * x;
                double yy = y * y;
                double zz = z * z;
                double ww = w * w;
                double xy = 2.0 * x * y;
                double xz = 2.0 * x * z;
                double yz = 2.0 * y * z;
                double xw = 2.0 * x * w;
                double yw = 2.0 * y * w;
                double zw = 2.0 * z * w;

                R(0, 0) = ww + xx - yy - zz;
                R(0, 1) = xy - zw;
                R(0, 2) = xz + yw;

                R(1, 0) = xy + zw;
                R(1, 1) = ww - xx + yy - zz;
                R(1, 2) = yz - xw;

                R(2, 0) = xz - yw;
                R(2, 1) = yz + xw;
                R(2, 2) = ww - xx - yy + zz;

                return R;
            }

            // ------------------------------------------------------------------
            // Utility
            // ------------------------------------------------------------------

            bool isValid(double epsilon = 1e-10) const noexcept
            {
                return (norm() > epsilon);
            }
        };

    } // namespace math
} // namespace motion_control
