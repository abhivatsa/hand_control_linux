#include "JointTrajectoryController.h"
#include <iostream>  // for debugging if needed
#include <algorithm> // std::max
#include <cmath>

namespace motion_control
{
    namespace control
    {

        JointTrajectoryController::JointTrajectoryController()
            : numJoints_(0),
              maxTime_(0.0),
              globalAccTime_(0.0),
              globalCruiseTime_(0.0),
              segmentTime_(0.0),
              isActive_(false)
        {
            // zero init arrays
            for (size_t i = 0; i < MAX_JOINTS; i++)
            {
                iniPos_[i] = 0.0;
                finalPos_[i] = 0.0;
                jointAcc_[i] = 0.0;
            }
        }

        JointTrajectoryController::~JointTrajectoryController()
        {
        }

        bool JointTrajectoryController::init(size_t numJoints)
        {
            if (numJoints > MAX_JOINTS)
                return false;

            numJoints_ = numJoints;
            return true;
        }

        void JointTrajectoryController::start()
        {
            // Reset the internal time
            segmentTime_ = 0.0;
        }

        void JointTrajectoryController::update(const merai::JointState *jointStates,
                                               merai::JointCommand *jointCommands,
                                               double dt)
        {
            if (!isActive_)
            {
                // If no active trajectory, hold current
                holdPosition(jointStates, jointCommands);
                return;
            }

            // Advance time
            segmentTime_ += dt;
            if (segmentTime_ > maxTime_)
            {
                segmentTime_ = maxTime_;
                isActive_ = false; // We’ve completed the move
            }

            // The user snippet lumps accelerate, cruise, decelerate into “globalAccTime_”,
            // “globalCruiseTime_”, and same “globalAccTime_” for dec. total = 2*acc + cruise.

            double t1 = globalAccTime_;                     // end of accel
            double t2 = globalAccTime_ + globalCruiseTime_; // end of cruise
            double t3 = t2 + globalAccTime_;                // end of decel => maxTime_

            double t = segmentTime_;

            for (size_t j = 0; j < numJoints_; j++)
            {
                // We'll replicate the snippet’s logic:
                double dist = (finalPos_[j] - iniPos_[j]);
                double a = jointAcc_[j]; // we computed sign in setTargetPositions
                // accelerate => v = a * t, s = 0.5 * a * t^2
                // cruise => v = a * tAcc, s = ...
                // decel => symmetrical

                double pos = iniPos_[j]; // default

                if (t <= t1)
                {
                    // accelerating
                    pos = iniPos_[j] + 0.5 * a * (t * t);
                }
                else if (t <= t2)
                {
                    // cruising
                    double acc_pos_end = iniPos_[j] + 0.5 * a * (t1 * t1);
                    double v_cruise = a * t1; // velocity at end of accel
                    double t_cruise = t - t1;
                    pos = acc_pos_end + (v_cruise * t_cruise);
                }
                else if (t <= t3)
                {
                    // decelerating
                    double t_decel = t - t2;
                    double acc_pos_end = iniPos_[j] + 0.5 * a * (t1 * t1);
                    double v_cruise = a * t1;
                    double cruise_dist = v_cruise * (t2 - t1);

                    double pos_cruise_end = acc_pos_end + cruise_dist;

                    // decelerate from v_cruise to 0 over time t_decel
                    // pos = x0 + v_cruise * t_decel - 0.5 * a * t_decel^2
                    // but we use negative a => -a
                    double a_dec = -a;
                    pos = pos_cruise_end + (v_cruise * t_decel) + 0.5 * a_dec * (t_decel * t_decel);
                }
                else
                {
                    // done
                    pos = finalPos_[j];
                }

                // If dist was negative, we have to offset or flip it
                // But in setTargetPositions, we already accounted for sign in jointAcc_[j].
                // So pos is relative to iniPos_ going in the correct direction automatically.

                // clamp final
                if (segmentTime_ >= maxTime_)
                    pos = finalPos_[j];

                jointCommands[j].position = pos;
            }
        }

        void JointTrajectoryController::stop()
        {
            isActive_ = false;
        }

        void JointTrajectoryController::holdPosition(const merai::JointState *jointStates,
                                                     merai::JointCommand *jointCommands) const
        {
            for (size_t j = 0; j < numJoints_; j++)
            {
                jointCommands[j].position = jointStates[j].position;
            }
        }

        // --------------------------------------------------------------------
        // This is where we do the multi-joint synchronization logic
        // exactly like your code snippet. Then store results in the
        // controller so update() can do the time-based stepping.
        // --------------------------------------------------------------------
        bool JointTrajectoryController::planTrajectory(const double *currentPos,
                                                           const double *targetPos,
                                                           double *jointVel,
                                                           double *jointAcc,
                                                           double dt)
        {
            if (numJoints_ == 0)
            {
                return false;
            }

            // 1) Copy ini/final positions, fix sign for velocity/acc if final < initial
            for (size_t j = 0; j < numJoints_; j++)
            {
                iniPos_[j] = currentPos[j];
                finalPos_[j] = targetPos[j];

                // If final is less than initial, flip sign
                if (finalPos_[j] < iniPos_[j])
                {
                    jointVel[j] = -jointVel[j];
                    jointAcc[j] = -jointAcc[j];
                }
            }

            // 2) Compute local times for each joint, pick the maximum
            double max_time = 0.0;
            double local_acc_time = 0.0;
            double local_cruise_time = 0.0;

            double best_acc_time = 0.0; // store for global
            double best_cruise_time = 0.0;

            for (size_t j = 0; j < numJoints_; j++)
            {
                double joint_diff = (finalPos_[j] - iniPos_[j]);
                double dist_needed_full_speed = (jointVel[j] * jointVel[j]) / std::fabs(jointAcc[j]);

                double acc_time = 0.0;
                double cruise_time = 0.0;
                double local_time = 0.0;

                // same logic as snippet
                if (std::fabs(joint_diff) < std::fabs(dist_needed_full_speed))
                {
                    if (std::fabs(joint_diff) > 1e-4)
                    {
                        // recalc velocity
                        double sign_v = (jointVel[j] >= 0.0) ? 1.0 : -1.0;
                        double new_v = 0.99 * sign_v *
                                       std::sqrt(2.0 * std::fabs(joint_diff) * std::fabs(jointAcc[j]));

                        acc_time = std::fabs(new_v / jointAcc[j]);
                        double acc_dist = (new_v * new_v) / (2.0 * jointAcc[j]);
                        cruise_time = (joint_diff - 2.0 * acc_dist) / new_v;
                        local_time = 2.0 * acc_time + cruise_time;
                    }
                    else
                    {
                        acc_time = 0.0;
                        cruise_time = 0.0;
                        local_time = 0.0;
                    }
                }
                else
                {
                    acc_time = std::fabs(jointVel[j] / jointAcc[j]);
                    double acc_dist = (jointVel[j] * jointVel[j]) / (2.0 * std::fabs(jointAcc[j]));
                    cruise_time = (joint_diff - 2.0 * acc_dist) / jointVel[j];
                    local_time = 2.0 * acc_time + cruise_time;
                }

                // track max
                if (local_time > max_time)
                {
                    max_time = local_time;
                    best_acc_time = acc_time;
                    best_cruise_time = cruise_time;
                }
            }

            // 3) Recompute each joint’s acceleration so all finish in that same max_time
            //    distance = a * [acc_time^2 + acc_time*cruise_time]
            //    This lumps dec_time = acc_time for symmetrical trapezoid
            //    total_time = 2*acc_time + cruise_time
            //    => a = dist / (acc_time^2 + acc_time*cruise_time)
            double global_acc_time = best_acc_time;       // store
            double global_cruise_time = best_cruise_time; // store
            double global_total_time = 2.0 * global_acc_time + global_cruise_time;

            // For each joint, fix the acceleration
            for (size_t j = 0; j < numJoints_; j++)
            {
                double joint_diff = (finalPos_[j] - iniPos_[j]);
                double denom = (global_acc_time * global_acc_time +
                                global_acc_time * global_cruise_time);
                if (std::fabs(denom) > 1e-8)
                {
                    double a = (joint_diff / denom);
                    jointAcc_[j] = a; // store in the controller
                }
                else
                {
                    jointAcc_[j] = 0.0; // trivial move
                }
            }

            // 4) Store global times in the controller
            globalAccTime_ = global_acc_time;
            globalCruiseTime_ = global_cruise_time;
            maxTime_ = global_total_time;

            // Mark active, reset time
            isActive_ = true;
            segmentTime_ = 0.0;

            return true;
        }

    } // namespace control
} // namespace motion_control
