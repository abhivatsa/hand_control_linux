#pragma once

#include <array>
#include <cstddef>
#include <cmath>
#include "Matrix.h" // We need the template for operator*(Matrix x Vector)

namespace motion_control
{
    namespace math
    {

        /**
         * @brief A fixed-size, stack-allocated vector of dimension N.
         *
         * - No dynamic memory
         * - No exceptions
         * - Inline, noexcept
         */
        template <std::size_t N>
        class Vector
        {
        public:
            std::array<double, N> data;

            /**
             * @brief Default constructor (all zeros).
             */
            constexpr Vector() noexcept : data{}
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] = 0.0;
                }
            }

            /**
             * @brief Construct from a std::array<double, N>.
             */
            constexpr Vector(const std::array<double, N> &arr) noexcept : data(arr)
            {
            }

            // ------------------------------------------------------------------
            // Accessors
            // ------------------------------------------------------------------

            /**
             * @brief Access element by index.
             */
            constexpr double &operator[](std::size_t i) noexcept
            {
                return data[i];
            }

            constexpr double operator[](std::size_t i) const noexcept
            {
                return data[i];
            }

            /**
             * @brief Return how many elements in this vector (i.e., N).
             */
            static constexpr std::size_t size() noexcept
            {
                return N;
            }

            /**
             * @brief Pointer to underlying data array (useful for C APIs).
             */
            constexpr double *dataPtr() noexcept
            {
                return data.data();
            }

            constexpr const double *dataPtr() const noexcept
            {
                return data.data();
            }

            // ------------------------------------------------------------------
            // Basic Arithmetic (non-in-place)
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
                    // no exception => fallback to return original
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

            // ------------------------------------------------------------------
            // In-place arithmetic
            // ------------------------------------------------------------------

            /**
             * @brief vec += other
             */
            constexpr Vector<N> &operator+=(const Vector<N> &other) noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] += other.data[i];
                }
                return *this;
            }

            /**
             * @brief vec -= other
             */
            constexpr Vector<N> &operator-=(const Vector<N> &other) noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] -= other.data[i];
                }
                return *this;
            }

            /**
             * @brief vec *= scalar
             */
            constexpr Vector<N> &operator*=(double scalar) noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] *= scalar;
                }
                return *this;
            }

            /**
             * @brief vec /= scalar
             */
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

            /**
             * @brief Dot Product
             */
            constexpr double dot(const Vector<N> &other) const noexcept
            {
                double sum = 0.0;
                for (std::size_t i = 0; i < N; ++i)
                {
                    sum += data[i] * other.data[i];
                }
                return sum;
            }

            /**
             * @brief Sum of all components
             */
            constexpr double sum() const noexcept
            {
                double s = 0.0;
                for (std::size_t i = 0; i < N; ++i)
                {
                    s += data[i];
                }
                return s;
            }

            /**
             * @brief Norm (Euclidean length).
             */
            constexpr double norm() const noexcept
            {
                return std::sqrt(dot(*this));
            }

            /**
             * @brief Normalized copy of this vector.
             */
            constexpr Vector<N> normalized() const noexcept
            {
                double n = norm();
                if (n < 1e-12)
                {
                    return *this; // fallback if near zero
                }
                return *this / n;
            }

            /**
             * @brief Cross product (only valid if N == 3)
             */
            constexpr Vector<3> cross(const Vector<3> &other) const noexcept
            {
                static_assert(N == 3, "cross() only valid for 3D vectors");
                return Vector<3>{
                    {
                        data[1] * other.data[2] - data[2] * other.data[1], // x
                        data[2] * other.data[0] - data[0] * other.data[2], // y
                        data[0] * other.data[1] - data[1] * other.data[0]  // z
                    }};
            }

            // ------------------------------------------------------------------
            // cwiseMultiply => element-wise (Hadamard) product
            // ------------------------------------------------------------------
            constexpr Vector<N> cwiseMultiply(const Vector<N> &other) const noexcept
            {
                Vector<N> result;
                for (std::size_t i = 0; i < N; ++i)
                {
                    result[i] = data[i] * other.data[i];
                }
                return result;
            }

            // ------------------------------------------------------------------
            // setZero()
            // ------------------------------------------------------------------
            /**
             * @brief Fill the vector with 0.0 in all components.
             */
            constexpr void setZero() noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] = 0.0;
                }
            }

            // ------------------------------------------------------------------
            // setConstant()
            // ------------------------------------------------------------------
            /**
             * @brief Fill the vector with a constant value in all components.
             */
            constexpr void setConstant(double val) noexcept
            {
                for (std::size_t i = 0; i < N; ++i)
                {
                    data[i] = val;
                }
            }

            // ------------------------------------------------------------------
            // Transpose => 1 x N matrix
            // ------------------------------------------------------------------
            constexpr Matrix<1, N> transpose() const noexcept
            {
                Matrix<1, N> result;
                for (std::size_t i = 0; i < N; ++i)
                {
                    result(0, i) = data[i];
                }
                return result;
            }
        };

        // ------------------------------------------------------------------
        // Matrix × Vector Multiplication
        // We define it here, after Vector is fully known.
        // ------------------------------------------------------------------

        /**
         * @brief (ROWSxCOLS) * Vector<COLS> => Vector<ROWS>
         */
        template <std::size_t ROWS, std::size_t COLS>
        constexpr Vector<ROWS> operator*(const Matrix<ROWS, COLS> &mat, const Vector<COLS> &vec) noexcept
        {
            Vector<ROWS> result;
            result.setZero();
            for (std::size_t i = 0; i < ROWS; ++i)
            {
                double sum = 0.0;
                for (std::size_t j = 0; j < COLS; ++j)
                {
                    sum += mat(i, j) * vec[j];
                }
                result[i] = sum;
            }
            return result;
        }

    } // namespace math
} // namespace motion_control