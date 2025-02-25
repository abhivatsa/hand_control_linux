#pragma once

#include <string>
#include <array>

#include "merai/ParameterServer.h" // For the ParameterServer definition
#include "math_lib/Matrix.h"           // For Matrix<3,3>, etc.
#include "math_lib/Vector.h"           // For Vector<3> and Vector<6>

namespace motion_control
{
    namespace robotics
    {
        namespace six_axis
        {

            /**
             * @brief Stores static data for a 6-axis robot, but in Vector/Matrix forms:
             *        - Link mass, center of mass (Vector<3>), inertia (Matrix<3,3>)
             *        - Joint origin (Vector<3> + Matrix<3,3> for orientation)
             *        - Joint axis (Vector<3>)
             *        - Joint min/max angles (Vector<6>)
             */
            class SixAxisModel
            {
            public:
                static constexpr int NUM_LINKS = 6;
                static constexpr int NUM_JOINTS = 6;

                SixAxisModel() = default;
                ~SixAxisModel() = default;

                /**
                 * @brief Load data for 6 links and 6 joints from the ParameterServer,
                 *        converting Euler angles into rotation matrices, etc.
                 * @param ps The ParameterServer (read from shared memory).
                 * @return true if loaded successfully (>= 6 links/joints), else false.
                 */
                bool loadFromParameterServer(const merai::ParameterServer &ps);

                // -------------------------------------------------------------------------
                // Accessors
                // -------------------------------------------------------------------------

                // Robot name, e.g. "UR5"
                const std::string &getRobotName() const { return robotName_; }

                // Link data
                double getLinkMass(int idx) const { return linkMasses_[idx]; }
                const motion_control::math::Vector<3> &getLinkCOM(int idx) const { return linkCOMs_[idx]; }
                const motion_control::math::Matrix<3, 3> &getLinkInertia(int idx) const { return linkInertias_[idx]; }

                // Joint data (parent/child link names, for reference)
                const std::string &getJointParentLink(int idx) const { return jointParents_[idx]; }
                const std::string &getJointChildLink(int idx) const { return jointChildren_[idx]; }

                // Joint origin transform
                // Position => 3D offset
                // Orientation => 3x3 rotation matrix
                const motion_control::math::Vector<3> &getJointOriginPos(int idx) const { return jointOriginPos_[idx]; }
                const motion_control::math::Matrix<3, 3> &getJointOriginRot(int idx) const { return jointOriginRot_[idx]; }

                // Joint axis
                const motion_control::math::Vector<3> &getJointAxis(int idx) const { return jointAxes_[idx]; }

                // Joint limits (we store all 6 in vectors)
                double getJointMinPosition(int idx) const { return jointMinPos_[idx]; }
                double getJointMaxPosition(int idx) const { return jointMaxPos_[idx]; }

            private:
                // Utility to build the inertia matrix from ixx, iyy, izz, ixy, ixz, iyz
                motion_control::math::Matrix<3, 3> makeInertiaMatrix(double ixx, double iyy, double izz,
                                                                     double ixy, double ixz, double iyz) const;

                // Utility to convert Euler angles (roll, pitch, yaw) to a 3x3 rotation matrix
                motion_control::math::Matrix<3, 3> eulerToRotation(double roll, double pitch, double yaw) const;

            private:
                std::string robotName_;

                // Link data
                std::array<double, NUM_LINKS> linkMasses_;
                std::array<motion_control::math::Vector<3>, NUM_LINKS> linkCOMs_;
                std::array<motion_control::math::Matrix<3, 3>, NUM_LINKS> linkInertias_;

                // Joint data
                std::array<std::string, NUM_JOINTS> jointParents_;
                std::array<std::string, NUM_JOINTS> jointChildren_;

                // origin pos => Vector<3>, origin rot => Matrix<3,3>
                std::array<motion_control::math::Vector<3>, NUM_JOINTS> jointOriginPos_;
                std::array<motion_control::math::Matrix<3, 3>, NUM_JOINTS> jointOriginRot_;

                // axis => Vector<3>
                std::array<motion_control::math::Vector<3>, NUM_JOINTS> jointAxes_;

                // joint min & max => store them in Vector<6> each
                motion_control::math::Vector<6> jointMinPos_;
                motion_control::math::Vector<6> jointMaxPos_;
            };

        } // namespace six_axis
    } // namespace robotics
} // namespace motion_control
