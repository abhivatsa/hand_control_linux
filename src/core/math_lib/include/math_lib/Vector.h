#pragma once

#include <array>
#include <cstddef>
#include <cmath>

namespace seven_axis_robot
{
    namespace math
    {

        // Forward declaration, if you want to mention Matrix in the interface
        template <std::size_t ROWS, std::size_t COLS>
        class Matrix;

        template <std::size_t N>
        class Vector
        {
        public:
            std::array<double, N> data;

            constexpr Vector() noexcept : data{}
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] = 0.0;
                }
            }

            constexpr Vector(const std::array<double, N> &arr) noexcept : data(arr)
            {
            }

            // ------------------------------------------------------------------
            // Accessors
            // ------------------------------------------------------------------
            constexpr double &operator[](std::size_t i) noexcept { return data[i]; }
            constexpr double operator[](std::size_t i) const noexcept { return data[i]; }
            static constexpr std::size_t size() noexcept { return N; }
            constexpr double *dataPtr() noexcept { return data.data(); }
            constexpr const double *dataPtr() const noexcept { return data.data(); }

            // ------------------------------------------------------------------
            // Basic Arithmetic
            // ------------------------------------------------------------------
            constexpr Vector<N> operator+(const Vector<N> &other) const noexcept
            {
                Vector<N> result;
                for (std::size_t i = 0; i < N; ++i)
                {
                    result[i] = data[i] + other[i];
                }
                return result;
            }

            constexpr Vector<N> operator-(const Vector<N> &other) const noexcept
            {
                Vector<N> result;
                for (std::size_t i = 0; i < N; ++i)
                {
                    result[i] = data[i] - other[i];
                }
                return result;
            }

            constexpr Vector<N> operator*(double scalar) const noexcept
            {
                Vector<N> result;
                for (std::size_t i = 0; i < N; ++i)
                {
                    result[i] = data[i] * scalar;
                }
                return result;
            }

            constexpr Vector<N> operator/(double scalar) const noexcept
            {
                if (scalar == 0.0)
                {
                    // No exception => fallback to return original
                    return *this;
                }
                Vector<N> result;
                double inv = 1.0 / scalar;
                for (std::size_t i = 0; i < N; ++i)
                {
                    result[i] = data[i] * inv;
                }
                return result;
            }

            // Add an initializer-list constructor:
            constexpr Vector(std::initializer_list<double> list) noexcept
            {
                // Copy up to N values from the list
                auto it = list.begin();
                for (std::size_t i = 0; i < N; ++i)
                {
                    if (it != list.end())
                    {
                        data[i] = *it++;
                    }
                    else
                    {
                        data[i] = 0.0;
                    }
                }
            }

            // ------------------------------------------------------------------
            // In-place arithmetic
            // ------------------------------------------------------------------
            constexpr Vector<N> &operator+=(const Vector<N> &other) noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] += other.data[i];
                }
                return *this;
            }

            constexpr Vector<N> &operator-=(const Vector<N> &other) noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] -= other.data[i];
                }
                return *this;
            }

            constexpr Vector<N> &operator*=(double scalar) noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] *= scalar;
                }
                return *this;
            }

            constexpr Vector<N> &operator/=(double scalar) noexcept
            {
                if (scalar != 0.0)
                {
                    double inv = 1.0 / scalar;
                    for (std::size_t i = 0; i < N; ++i)
                    {
                        data[i] *= inv;
                    }
                }
                return *this;
            }

            // ------------------------------------------------------------------
            // Vector-Specific Utilities
            // ------------------------------------------------------------------
            constexpr double dot(const Vector<N> &other) const noexcept
            {
                double sum = 0.0;
                for (std::size_t i = 0; i < N; ++i)
                {
                    sum += data[i] * other.data[i];
                }
                return sum;
            }

            constexpr double sum() const noexcept
            {
                double s = 0.0;
                for (std::size_t i = 0; i < N; ++i)
                {
                    s += data[i];
                }
                return s;
            }

            constexpr double norm() const noexcept
            {
                return std::sqrt(dot(*this));
            }

            constexpr Vector<N> normalized() const noexcept
            {
                double n = norm();
                if (n < 1e-12)
                {
                    return *this; // fallback if near zero
                }
                return *this / n;
            }

            // cross() only valid if N==3
            constexpr Vector<3> cross(const Vector<3> &other) const noexcept
            {
                static_assert(N == 3, "cross() only valid for 3D vectors");
                return Vector<3>{{
                    data[1] * other.data[2] - data[2] * other.data[1], // x
                    data[2] * other.data[0] - data[0] * other.data[2], // y
                    data[0] * other.data[1] - data[1] * other.data[0]  // z
                }};
            }

            constexpr Vector<N> cwiseMultiply(const Vector<N> &other) const noexcept
            {
                Vector<N> result;
                for (std::size_t i = 0; i < N; ++i)
                {
                    result[i] = data[i] * other.data[i];
                }
                return result;
            }

            constexpr void setZero() noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] = 0.0;
                }
            }

            constexpr void setConstant(double val) noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] = val;
                }
            }

            friend constexpr Vector<N> operator*(double scalar, const Vector<N> &v) noexcept
            {
                return v * scalar; // Reuse the member operator
            }

            // If you want Vector::transpose() -> Matrix<1, N> but want to avoid the circular include,
            // you can forward-declare Matrix above, and define this function in a separate .cpp or .tcc.
            // For now, let's just omit or move it to MatrixVectorOps if you like.
        };

    } // namespace math
} // namespace seven_axis_robot
