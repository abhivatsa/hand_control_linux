#include "control/controllers/HomingController.h"
#include <cmath>     // for std::fabs, std::sqrt
#include <algorithm> // std::max, std::min

namespace hand_control
{
    namespace control
    {
        HomingController::HomingController(const double* homePositions,
                                           int numJoints,
                                           hand_control::merai::JointState* statesPtr,
                                           hand_control::merai::JointCommand* commandsPtr)
            : statesPtr_(statesPtr),
              commandsPtr_(commandsPtr),
              numJoints_(numJoints)
        {
            if (numJoints_ > MAX_JOINTS) {
                numJoints_ = MAX_JOINTS; // clamp
            }

            // Copy the home positions
            for (int i = 0; i < numJoints_; i++)
            {
                homePositions_[i] = homePositions[i];
            }

            // Initialize arrays
            for (int i = 0; i < MAX_JOINTS; i++)
            {
                iniPositions_[i]  = 0.0;
                jointAcc_[i]      = 0.0;
            }
        }

        bool HomingController::init()
        {
            // Validate pointers
            if (!statesPtr_ || !commandsPtr_ || numJoints_ <= 0)
            {
                return false;
            }

            // For now, just set state to INIT
            state_ = ControllerState::INIT;
            return true;
        }

        void HomingController::start()
        {
            // Called when switching to this controller
            // Only proceed if we were INIT or STOPPED
            if (state_ == ControllerState::INIT || state_ == ControllerState::STOPPED)
            {
                // We'll set RUNNING here, but the actual planning (planTrajectory)
                // is triggered in the first update() call or we can do it right here.
                state_ = ControllerState::RUNNING;
                segmentTime_ = 0.0;
                isActive_    = false; // We'll plan in update()
            }
        }

        void HomingController::update(double dt)
        {
            // If the controller is not RUNNING, do nothing
            if (state_ != ControllerState::RUNNING)
            {
                return;
            }

            // If we haven't started a homing trajectory yet, plan it now
            if (!isActive_)
            {
                double currentPos[MAX_JOINTS];
                for (int i = 0; i < numJoints_; i++)
                {
                    currentPos[i] = statesPtr_[i].position;
                }
                planTrajectory(currentPos);
            }

            // If the homing trajectory is active, step it
            if (isActive_)
            {
                segmentTime_ += dt;
                if (segmentTime_ > maxTime_)
                {
                    segmentTime_ = maxTime_;
                    isActive_ = false; // we've completed the move
                }

                double t1 = globalAccTime_;                // end of accel
                double t2 = globalAccTime_ + globalCruiseTime_; // end of cruise
                double t3 = t2 + globalAccTime_;           // end of decel => maxTime_

                double t = segmentTime_;

                for (int j = 0; j < numJoints_; j++)
                {
                    double dist = (homePositions_[j] - iniPositions_[j]);
                    double a = jointAcc_[j]; // includes sign for direction

                    double pos = iniPositions_[j]; // default to start
                    // Phase-based logic (acc -> cruise -> dec)
                    if (t <= t1)
                    {
                        // accelerate
                        pos = iniPositions_[j] + 0.5 * a * (t * t);
                    }
                    else if (t <= t2)
                    {
                        // cruising
                        double acc_pos_end = iniPositions_[j] + 0.5 * a * (t1 * t1);
                        double v_cruise    = a * t1; // velocity at end of accel
                        double t_cruise    = t - t1;
                        pos = acc_pos_end + (v_cruise * t_cruise);
                    }
                    else if (t <= t3)
                    {
                        // decelerating
                        double t_decel = t - t2;
                        double acc_pos_end = iniPositions_[j] + 0.5 * a * (t1 * t1);
                        double v_cruise    = a * t1;
                        double cruise_dist = v_cruise * (t2 - t1);
                        double pos_cruise_end = acc_pos_end + cruise_dist;

                        double a_dec = -a; // symmetrical decel
                        pos = pos_cruise_end + (v_cruise * t_decel)
                              + 0.5 * a_dec * (t_decel * t_decel);
                    }
                    else
                    {
                        // finished
                        pos = homePositions_[j];
                    }

                    // If done or overshoot
                    if (!isActive_)
                    {
                        pos = homePositions_[j];
                    }

                    commandsPtr_[j].position = pos;
                    // For a pure position-control approach, set torque/velocity to 0
                    commandsPtr_[j].velocity = 0.0;
                    commandsPtr_[j].torque   = 0.0;
                }
            }
            else
            {
                // Trajectory done => hold the final (home) position
                holdPosition();
            }
        }

        void HomingController::stop()
        {
            // Called when transitioning away
            if (state_ == ControllerState::RUNNING)
            {
                // Optionally force isActive_ = false
                isActive_ = false;
                state_ = ControllerState::STOPPED;
            }
        }

        void HomingController::teardown()
        {
            // Cleanup if needed
            state_ = ControllerState::UNINIT;
        }

        void HomingController::planTrajectory(const double* currentPos)
        {
            // 1) Copy initial positions
            for (int j = 0; j < numJoints_; j++)
            {
                iniPositions_[j] = currentPos[j];
            }

            // Example velocity/acc limits
            double velLimit = 0.5; // rad/s
            double accLimit = 0.5; // rad/s^2

            double best_local_time = 0.0;
            double best_acc_time   = 0.0;
            double best_cruise_time= 0.0;

            // Compute local times for trapezoid motion
            for (int j = 0; j < numJoints_; j++)
            {
                double diff = homePositions_[j] - iniPositions_[j];
                double sign = (diff >= 0.0) ? 1.0 : -1.0;
                double v    = sign * velLimit;
                double a    = sign * accLimit;

                double dist_needed_full_speed = (v * v) / std::fabs(a);

                double acc_time = 0.0;
                double cruise_time = 0.0;
                double local_time = 0.0;

                if (std::fabs(diff) < std::fabs(dist_needed_full_speed))
                {
                    // can't reach full speed
                    double new_v = sign * std::sqrt(2.0 * std::fabs(diff) * std::fabs(a));
                    acc_time = std::fabs(new_v / a);
                    local_time = 2.0 * acc_time; // accel + decel
                }
                else
                {
                    acc_time = std::fabs(v / a);
                    double acc_dist = (v * v) / (2.0 * std::fabs(a));
                    cruise_time = (std::fabs(diff) - 2.0 * acc_dist) / std::fabs(v);
                    local_time = 2.0 * acc_time + cruise_time;
                }

                if (local_time > best_local_time)
                {
                    best_local_time   = local_time;
                    best_acc_time     = acc_time;
                    best_cruise_time  = cruise_time;
                }
            }

            double global_total_time = best_local_time;
            globalAccTime_    = best_acc_time;
            globalCruiseTime_ = best_cruise_time;
            maxTime_          = global_total_time;

            double denom = (globalAccTime_ * globalAccTime_
                            + globalAccTime_ * globalCruiseTime_);
            for (int j = 0; j < numJoints_; j++)
            {
                double diff = homePositions_[j] - iniPositions_[j];
                if (std::fabs(denom) > 1e-8)
                {
                    jointAcc_[j] = diff / denom;
                }
                else
                {
                    jointAcc_[j] = 0.0;
                }
            }

            segmentTime_ = 0.0;
            isActive_    = true;
        }

        void HomingController::holdPosition()
        {
            // Just hold the current position
            for (int j = 0; j < numJoints_; j++)
            {
                commandsPtr_[j].position = statesPtr_[j].position;
                commandsPtr_[j].velocity = 0.0;
                commandsPtr_[j].torque   = 0.0;
            }
        }

    } // namespace control
} // namespace hand_control
