#pragma once

#include <array>
#include <cstddef>
#include <cmath> // For sqrt, etc.

namespace hand_control
{
    namespace math
    {

        /**
         * @brief Stack-based matrix class of size ROWS × COLS, real-time friendly.
         *
         * - No dynamic memory
         * - No exceptions
         * - Inline and noexcept where feasible
         * - C++20 recommended for constexpr math
         */
        template <std::size_t ROWS, std::size_t COLS>
        class Matrix
        {
        public:
            std::array<std::array<double, COLS>, ROWS> data;

            // ------------------------------------------------------------------
            // Constructors
            // ------------------------------------------------------------------
            constexpr Matrix() noexcept : data{}
            {
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        data[i][j] = 0.0;
                    }
                }
            }

            constexpr Matrix(const std::array<std::array<double, COLS>, ROWS> &arr) noexcept
                : data(arr)
            {
            }

            // ------------------------------------------------------------------
            // Element Access
            // ------------------------------------------------------------------
            constexpr double &operator()(std::size_t row, std::size_t col) noexcept
            {
                return data[row][col];
            }

            constexpr double operator()(std::size_t row, std::size_t col) const noexcept
            {
                return data[row][col];
            }

            // ------------------------------------------------------------------
            // Basic Arithmetic
            // ------------------------------------------------------------------
            constexpr Matrix<ROWS, COLS> operator+(const Matrix<ROWS, COLS> &other) const noexcept
            {
                Matrix<ROWS, COLS> result;
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        result(i, j) = data[i][j] + other(i, j);
                    }
                }
                return result;
            }

            constexpr Matrix<ROWS, COLS> operator-(const Matrix<ROWS, COLS> &other) const noexcept
            {
                Matrix<ROWS, COLS> result;
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        result(i, j) = data[i][j] - other(i, j);
                    }
                }
                return result;
            }

            constexpr Matrix<ROWS, COLS> operator*(double scalar) const noexcept
            {
                Matrix<ROWS, COLS> result;
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        result(i, j) = data[i][j] * scalar;
                    }
                }
                return result;
            }

            constexpr Matrix<ROWS, COLS> operator/(double scalar) const noexcept
            {
                Matrix<ROWS, COLS> result;
                if (scalar != 0.0)
                {
                    double inv = 1.0 / scalar;
                    for (std::size_t i = 0; i < ROWS; ++i)
                    {
                        for (std::size_t j = 0; j < COLS; ++j)
                        {
                            result(i, j) = data[i][j] * inv;
                        }
                    }
                }
                else
                {
                    // If scalar is zero, user must handle it (we do no exception).
                    return *this;
                }
                return result;
            }

            template <std::size_t OTHER_COLS>
            constexpr Matrix<ROWS, OTHER_COLS> operator*(const Matrix<COLS, OTHER_COLS> &other) const noexcept
            {
                Matrix<ROWS, OTHER_COLS> result;
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < OTHER_COLS; ++j)
                    {
                        double sum = 0.0;
                        for (std::size_t k = 0; k < COLS; ++k)
                        {
                            sum += data[i][k] * other.data[k][j];
                        }
                        result(i, j) = sum;
                    }
                }
                return result;
            }

            // ------------------------------------------------------------------
            // In-Place Operators
            // ------------------------------------------------------------------
            constexpr Matrix<ROWS, COLS> &operator+=(const Matrix<ROWS, COLS> &other) noexcept
            {
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        data[i][j] += other.data[i][j];
                    }
                }
                return *this;
            }

            constexpr Matrix<ROWS, COLS> &operator-=(const Matrix<ROWS, COLS> &other) noexcept
            {
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        data[i][j] -= other.data[i][j];
                    }
                }
                return *this;
            }

            constexpr Matrix<ROWS, COLS> &operator*=(double scalar) noexcept
            {
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        data[i][j] *= scalar;
                    }
                }
                return *this;
            }

            constexpr Matrix<ROWS, COLS> &operator/=(double scalar) noexcept
            {
                if (scalar != 0.0)
                {
                    double inv = 1.0 / scalar;
                    for (std::size_t i = 0; i < ROWS; ++i)
                    {
                        for (std::size_t j = 0; j < COLS; ++j)
                        {
                            data[i][j] *= inv;
                        }
                    }
                }
                return *this;
            }

            // ------------------------------------------------------------------
            // Transpose
            // ------------------------------------------------------------------
            constexpr Matrix<COLS, ROWS> transpose() const noexcept
            {
                Matrix<COLS, ROWS> result;
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        result(j, i) = data[i][j];
                    }
                }
                return result;
            }

            // ------------------------------------------------------------------
            // Utilities
            // ------------------------------------------------------------------
            constexpr void setZero() noexcept
            {
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        data[i][j] = 0.0;
                    }
                }
            }

            constexpr void setIdentity() noexcept
            {
                static_assert(ROWS == COLS, "setIdentity() only valid for square matrices");
                setZero();
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    data[i][i] = 1.0;
                }
            }

            constexpr void setConstant(double val) noexcept
            {
                for (std::size_t i = 0; i < ROWS; ++i)
                {
                    for (std::size_t j = 0; j < COLS; ++j)
                    {
                        data[i][j] = val;
                    }
                }
            }

            constexpr double *dataPtr() noexcept
            {
                return &data[0][0];
            }

            constexpr const double *dataPtr() const noexcept
            {
                return &data[0][0];
            }
        };

    } // namespace math
} // namespace hand_control
