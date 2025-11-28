#pragma once

#include <string>
#include <array>

#include "merai/ParameterServer.h"         // merai::ParameterServer
#include "math_lib/Matrix.h"         // Matrix<3,3>
#include "math_lib/Vector.h"         // Vector<3>, Vector<N>
#include "math_lib/MatrixVectorOps.h"

namespace robot_lib
{

            class RobotModel {
                public:
                    // Match your JSON: 8 links (incl. tool0), 7 joints (DOF)
                    static constexpr int NUM_LINKS  = 8;
                    static constexpr int NUM_JOINTS = 7;

                    RobotModel() = default;
                    ~RobotModel() = default;

                    // Load from the ParameterServer (already parsed from robot_parameters.json)
                    bool loadFromParameterServer(const merai::ParameterServer& ps);


                    // Links
                    double                             getLinkMass(int idx) const { return linkMasses_[idx]; }
                    const robot::math::Vector<3>&      getLinkCOM(int idx)  const { return linkCOMs_[idx]; }
                    const robot::math::Matrix<3,3>&    getLinkInertia(int idx) const { return linkInertias_[idx]; }

                    // Joints
                    const robot::math::Vector<3>&      getJointOriginPos(int idx)  const { return jointOriginPos_[idx]; }
                    const robot::math::Matrix<3,3>&    getJointOriginRot(int idx)  const { return jointOriginRot_[idx]; }
                    const robot::math::Vector<3>&      getJointAxis(int idx)       const { return jointAxes_[idx]; }
                    double                             getJointMinPosition(int idx) const { return jointMinPos_[idx]; }
                    double                             getJointMaxPosition(int idx) const { return jointMaxPos_[idx]; }

                    // Gravity in base frame
                    const robot::math::Vector<3>&      gravity() const { return gravity_; }

                    
                private:
                    // Helpers
                    robot::math::Matrix<3,3> makeInertiaMatrix(double ixx, double iyy, double izz,
                                                            double ixy, double ixz, double iyz) const;
                    robot::math::Matrix<3,3> eulerToRotation(double roll, double pitch, double yaw) const;

                private:
                    std::string                                        robotName_;

                    // Link data
                    std::array<double,                   NUM_LINKS>     linkMasses_{};
                    std::array<robot::math::Vector<3>,   NUM_LINKS>     linkCOMs_{};
                    std::array<robot::math::Matrix<3,3>, NUM_LINKS>     linkInertias_{};

                    // Joint data
                    std::array<std::string,                NUM_JOINTS>  jointParents_{};
                    std::array<std::string,                NUM_JOINTS>  jointChildren_{};
                    std::array<robot::math::Vector<3>,     NUM_JOINTS>  jointOriginPos_{};
                    std::array<robot::math::Matrix<3,3>,   NUM_JOINTS>  jointOriginRot_{};
                    std::array<robot::math::Vector<3>,     NUM_JOINTS>  jointAxes_{};
                    robot::math::Vector<NUM_JOINTS>                      jointMinPos_{};
                    robot::math::Vector<NUM_JOINTS>                      jointMaxPos_{};

                    // Gravity (m/s^2), base frame
                    robot::math::Vector<3>                             gravity_{};
            };

} // namespace robot_lib
