#pragma once

#include <string>
#include <array>

// Use your updated merai/ParameterServer.h
#include "hand_control_merai/merai/ParameterServer.h"       // Provides hand_control::merai::ParameterServer
#include "math_lib/Matrix.h"            // For Matrix<3,3>, etc.
#include "math_lib/Vector.h"            // For Vector<3> and Vector<6>
#include "math_lib/MatrixVectorOps.h"

namespace hand_control
{
    namespace robotics
    {
        namespace haptic_device
        {

            /**
             * @brief Stores static data for a robotic hand in Vector/Matrix forms:
             *        - Link mass, center of mass (Vector<3>), inertia (Matrix<3,3>)
             *        - Joint origin (Vector<3> + Matrix<3,3> for orientation)
             *        - Joint axis (Vector<3>)
             *        - Joint min/max angles (Vector<6>)
             *
             * Note: The default constants (NUM_LINKS and NUM_JOINTS) here are
             *       placeholders. Update them to match the actual number of
             *       links/joints in your hand model.
             */
            class HapticDeviceModel
            {
            public:
                // Adjust these to match your hand's actual links and joints.
                static constexpr int NUM_LINKS = 6;
                static constexpr int NUM_JOINTS = 6;

                HapticDeviceModel() = default;
                ~HapticDeviceModel() = default;

                /**
                 * @brief Load data for the robot hand from the ParameterServer,
                 *        e.g. ps.links[0..N], ps.joints[0..N], etc.
                 * @param ps  The ParameterServer struct (populated from JSON).
                 * @return true if at least NUM_LINKS/NUM_JOINTS were present and loaded, else false.
                 */
                bool loadFromParameterServer(const hand_control::merai::ParameterServer &ps);

                // -------------------------------------------------------------------------
                // Accessors
                // -------------------------------------------------------------------------
                const std::string &getRobotName() const { return robotName_; }

                // Link data
                double getLinkMass(int idx) const { return linkMasses_[idx]; }
                const hand_control::math::Vector<3> &getLinkCOM(int idx) const { return linkCOMs_[idx]; }
                const hand_control::math::Matrix<3, 3> &getLinkInertia(int idx) const { return linkInertias_[idx]; }

                // Joint data (parent/child link names, for reference)
                const std::string &getJointParentLink(int idx) const { return jointParents_[idx]; }
                const std::string &getJointChildLink(int idx) const { return jointChildren_[idx]; }

                const hand_control::math::Vector<3> &getJointOriginPos(int idx) const { return jointOriginPos_[idx]; }
                const hand_control::math::Matrix<3, 3> &getJointOriginRot(int idx) const { return jointOriginRot_[idx]; }

                const hand_control::math::Vector<3> &getJointAxis(int idx) const { return jointAxes_[idx]; }

                double getJointMinPosition(int idx) const { return jointMinPos_[idx]; }
                double getJointMaxPosition(int idx) const { return jointMaxPos_[idx]; }

            private:
                // Utility to build a 3×3 inertia matrix from ixx, iyy, izz, ixy, ixz, iyz
                hand_control::math::Matrix<3, 3> makeInertiaMatrix(double ixx, double iyy, double izz,
                                                                     double ixy, double ixz, double iyz) const;

                // Utility to convert Euler angles (roll, pitch, yaw) to a 3×3 rotation matrix
                hand_control::math::Matrix<3, 3> eulerToRotation(double roll, double pitch, double yaw) const;

            private:
                std::string robotName_;

                // Link data
                std::array<double, NUM_LINKS> linkMasses_{};
                std::array<hand_control::math::Vector<3>, NUM_LINKS> linkCOMs_{};
                std::array<hand_control::math::Matrix<3, 3>, NUM_LINKS> linkInertias_{};

                // Joint data
                std::array<std::string, NUM_JOINTS> jointParents_{};
                std::array<std::string, NUM_JOINTS> jointChildren_{};

                // origin pos => Vector<3>, origin rot => Matrix<3,3>
                std::array<hand_control::math::Vector<3>, NUM_JOINTS> jointOriginPos_{};
                std::array<hand_control::math::Matrix<3, 3>, NUM_JOINTS> jointOriginRot_{};

                // axis => Vector<3>
                std::array<hand_control::math::Vector<3>, NUM_JOINTS> jointAxes_{};

                // joint min & max => store them in Vector<6> each
                hand_control::math::Vector<6> jointMinPos_{};
                hand_control::math::Vector<6> jointMaxPos_{};
            };

        } // namespace haptic_device
    } // namespace robotics
} // namespace hand_control
