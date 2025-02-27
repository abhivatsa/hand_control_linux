#pragma once

#include "Matrix.h"
#include "Vector.h"

namespace hand_control
{
    namespace math
    {

        /**
         * @brief Multiply (ROWSxCOLS) matrix by a Vector<COLS>, resulting in Vector<ROWS>.
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

        /**
         * @brief Extract a 3x3 rotation matrix from T (at least 3x3).
         */
        template <std::size_t ROWS, std::size_t COLS>
        constexpr Matrix<3, 3> getRotation3x3(const Matrix<ROWS, COLS> &T) noexcept
        {
            static_assert(ROWS >= 3 && COLS >= 3,
                          "Matrix must be at least 3×3 to extract rotation");
            Matrix<3, 3> R;
            for (std::size_t r = 0; r < 3; ++r)
            {
                for (std::size_t c = 0; c < 3; ++c)
                {
                    R(r, c) = T(r, c);
                }
            }
            return R;
        }

        /**
         * @brief Extract a 3D translation from T (assuming T is at least 3x4).
         */
        template <std::size_t ROWS, std::size_t COLS>
        constexpr Vector<3> getTranslation3(const Matrix<ROWS, COLS> &T) noexcept
        {
            static_assert(ROWS >= 3 && COLS >= 4,
                          "Matrix must be at least 3×4 to extract translation");
            Vector<3> t;
            t[0] = T(0, 3);
            t[1] = T(1, 3);
            t[2] = T(2, 3);
            return t;
        }

        /**
         * @brief Build a 4×4 transform from a 3×3 rotation + 3D translation.
         */
        constexpr Matrix<4, 4> makeTransform4x4(const Matrix<3, 3> &R, const Vector<3> &t) noexcept
        {
            Matrix<4, 4> T;
            T.setIdentity();
            for (std::size_t r = 0; r < 3; ++r)
            {
                for (std::size_t c = 0; c < 3; ++c)
                {
                    T(r, c) = R(r, c);
                }
            }
            T(0, 3) = t[0];
            T(1, 3) = t[1];
            T(2, 3) = t[2];
            return T;
        }

    } // namespace math
} // namespace hand_control
