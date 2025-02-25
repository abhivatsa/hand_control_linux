#pragma once

#include <string>
#include "control/controllers/BaseController.h" // Where BaseController is defined
#include "merai/RTMemoryLayout.h"               // For JointState, JointCommand

#include "robotics_lib/six_axis/SixAxisModel.h"
#include "robotics_lib/six_axis/SixAxisKinematics.h"
#include "robotics_lib/six_axis/SixAxisDynamics.h"

namespace motion_control
{
    namespace control
    {

        /**
         * @brief TeleopController implements a Cartesian teleoperation approach
         *        with a software-defined RCM, mapping user inputs (e.g., dx, dy, dz)
         *        to new joint commands in real time.
         *
         *        Accepts references to the same robot model used by GravityCompController
         *        to ensure consistent kinematics. No dynamic memory is used.
         */
        class TeleopController : public BaseController
        {
        public:
            /**
             * @brief Constructor that takes a reference to the six-axis model.
             *        We'll use it to initialize our kinematics object for RCM enforcement.
             */
            TeleopController(const motion_control::robotics::six_axis::SixAxisModel &model);

            ~TeleopController() override = default;

            // Lifecycle methods from BaseController
            bool init(const std::string &controllerName) override;
            void start() override;
            void update(const merai::JointState *states,
                        merai::JointCommand *commands,
                        int numJoints,
                        double dt) override;
            void stop() override;
            void teardown() override;

            /**
             * @brief Allows external code (like a GUI or hardware callback) to set
             *        incremental Cartesian displacements for teleoperation.
             *        E.g., dx, dy, dz for translation; droll, dpitch, dyaw for orientation changes.
             */
            void setInputCartesian(double dx, double dy, double dz,
                                   double droll, double dpitch, double dyaw);

        private:
            // Reference to the same six-axis model used by other controllers
            const motion_control::robotics::six_axis::SixAxisModel &model_;

            // We create a local kinematics object that references the same model
            motion_control::robotics::six_axis::SixAxisKinematics kinematics_;

            // Keep track of the end-effector "desired pose" in Cartesian space:
            //   [0] = x, [1] = y, [2] = z, [3] = roll, [4] = pitch, [5] = yaw
            double desiredEndEffectorPose_[6];

            // Optionally store the current joint angles from last update
            // for incremental or reference-based IK. For typical 6-DOF manipulator:
            static constexpr int MAX_JOINTS = 6;
            double currentJointAngles_[MAX_JOINTS];

            /**
             * @brief A small helper to enforce an RCM constraint in the pose,
             *        or you can do it inside your computeIKWithRCM function.
             */
            void enforceRCMConstraint();
        };

    } // namespace control
} // namespace motion_control
