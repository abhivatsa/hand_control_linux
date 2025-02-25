#pragma once

#include <cstddef>
#include "Matrix.h"

namespace motion_control {
namespace math {

/**
 * @brief Wraps a 3×3 Matrix for rotation.
 */
class RotationMatrix
{
public:
    Matrix<3,3> mat;

    constexpr RotationMatrix() noexcept : mat()
    {
        // Identity by default
        mat.setIdentity();
    }

    constexpr RotationMatrix(const Matrix<3,3>& m) noexcept : mat(m)
    {
    }

    static constexpr RotationMatrix identity() noexcept
    {
        RotationMatrix R;
        R.mat.setIdentity();
        return R;
    }

    // Access
    constexpr double operator()(std::size_t row, std::size_t col) const noexcept
    {
        return mat(row,col);
    }

    constexpr double& operator()(std::size_t row, std::size_t col) noexcept
    {
        return mat(row,col);
    }

    // Quaternion conversion
    static RotationMatrix fromQuaternion(const Quaternion& q) noexcept
    {
        RotationMatrix R;
        R.mat = q.toRotationMatrix();
        return R;
    }

    Quaternion toQuaternion() const noexcept
    {
        return Quaternion::fromRotationMatrix(mat);
    }

    // Compose rotations
    constexpr RotationMatrix operator*(const RotationMatrix& other) const noexcept
    {
        RotationMatrix result;
        result.mat = this->mat * other.mat; // (3x3)*(3x3)
        return result;
    }

    // Return underlying matrix
    constexpr const Matrix<3,3>& toMatrix() const noexcept
    {
        return mat;
    }

    // Check orthonormal
    bool isOrthonormal(double epsilon = 1e-10) const noexcept
    {
        // R^T * R ~ I
        Matrix<3,3> RTR = mat.transpose() * mat;

        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                double expected = (i == j) ? 1.0 : 0.0;
                if (std::fabs(RTR(i,j) - expected) > epsilon) {
                    return false;
                }
            }
        }
        return true;
    }
};

} // namespace math
} // namespace motion_control
