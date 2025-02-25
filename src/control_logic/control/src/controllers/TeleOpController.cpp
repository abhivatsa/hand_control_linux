#include "control/controllers/TeleOpController.h"
#include <iostream> // debug prints
#include <cmath>    // if needed for math ops

namespace motion_control
{
    namespace control
    {

        TeleopController::TeleopController(const motion_control::robotics::six_axis::SixAxisModel &model)
            : model_(model),
              kinematics_(model) // Construct kinematics with the same model
        {
            // Initialize everything to 0
            for (int i = 0; i < 6; i++)
            {
                desiredEndEffectorPose_[i] = 0.0;
            }
            for (int j = 0; j < MAX_JOINTS; j++)
            {
                currentJointAngles_[j] = 0.0;
            }
        }

        bool TeleopController::init(const std::string &controllerName)
        {
            name_ = controllerName;
            state_ = ControllerState::INIT;

            std::cout << "[TeleopController] init: " << controllerName << std::endl;
            return true;
        }

        void TeleopController::start()
        {
            state_ = ControllerState::RUNNING;
            std::cout << "[TeleopController] start." << std::endl;
        }

        void TeleopController::update(const merai::JointState *states,
                                      merai::JointCommand *commands,
                                      int numJoints,
                                      double dt)
        {
            if (state_ != ControllerState::RUNNING)
            {
                // If not RUNNING, just hold position
                for (int j = 0; j < numJoints; ++j)
                {
                    commands[j].position = states[j].position;
                }
                return;
            }

            // 1) Convert from arrays to math::Vector<6>
            //    (Assuming numJoints == 6; if variable, handle accordingly.)
            motion_control::math::Vector<6> currentRobotJointAngles;
            motion_control::math::Vector<4> currentInstrumentJointAngles;
            motion_control::math::Matrix<4, 4> currentRobotPose_;
            motion_control::math::Matrix<4, 4> currentInstrumentPose_;

            currentRobotJointAngles.setZero();
            currentInstrumentJointAngles.setZero();
            currentRobotPose_.setZero();
            currentInstrumentPose_.setZero();

            // Here num joints should be 10, use 6 for robot and other 4 for instrument tip
            for (int i = 0; i < numJoints; ++i)
            {
                if (i < 6)
                {
                    currentRobotJointAngles[i] = states[i].position; // e.g. in radians
                }
                else
                {
                    currentInstrumentJointAngles[i - 6] = states[i].position; // e.g. in radians
                }
            }

            kinematics_.forwardKinematics(currentRobotJointAngles, currentRobotPose_);

            // Extract translation and rotation from current pose , and add hand Control incremental position change to it.

            // compute inverse kinematics of instrument tip and calculate desired pose for robottcp
            motion_control::math::Vector<6> desiredRobotJointAngles;
            motion_control::math::Vector<4> desiredInstrumentJointAngles;
            motion_control::math::Matrix<4, 4> desiredRobotPose_;
            motion_control::math::Matrix<4, 4> desiredInstrumentPose_;
            
            desiredRobotJointAngles = currentRobotJointAngles;
            desiredInstrumentJointAngles = currentInstrumentJointAngles;
            desiredRobotPose_ = currentRobotPose_;
            desiredInstrumentPose_ = currentInstrumentPose_;

            // write rcm logic
            

            // write logic here to update the desired robotpose to update inverse kinematics of robot
            kinematics_.computeInverseKinematics(currentRobotJointAngles, desiredRobotPose_, desiredRobotJointAngles);

            for (int i = 0; i < numJoints; ++i)
            {
                if (i < 6)
                {
                    commands[i].position = desiredRobotJointAngles[i]; // e.g. in radians
                }
                else
                {
                    commands[i].position = currentInstrumentJointAngles[i - 6]; // e.g. in radians
                }
            }

        }

        void TeleopController::stop()
        {
            state_ = ControllerState::STOPPED;
            std::cout << "[TeleopController] stop." << std::endl;
        }

        void TeleopController::teardown()
        {
            std::cout << "[TeleopController] teardown." << std::endl;
            state_ = ControllerState::UNINIT;
        }

        void TeleopController::setInputCartesian(double dx, double dy, double dz,
                                                 double droll, double dpitch, double dyaw)
        {
            // Accumulate increments in the desired end-effector pose
            desiredEndEffectorPose_[0] += dx;
            desiredEndEffectorPose_[1] += dy;
            desiredEndEffectorPose_[2] += dz;
            desiredEndEffectorPose_[3] += droll;
            desiredEndEffectorPose_[4] += dpitch;
            desiredEndEffectorPose_[5] += dyaw;
        }

        void TeleopController::enforceRCMConstraint()
        {
            // If your computeInverseKinematicsRCM(...) function already enforces the pivot,
            // you might leave this empty.
            // Otherwise, do partial geometry here to ensure the tool pivot stays at the insertion point.
            // e.g., by limiting certain DOF or referencing a pivot location to compute the
            // relative transform for the end-effector.
        }

    } // namespace control
} // namespace motion_control
