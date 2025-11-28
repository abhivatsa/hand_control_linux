#include "src/network_api/include/network_api/HapticDeviceAPI.h"
#include <cmath>
#include <cstdio>
#include <iostream>
    namespace network_api
    {

        HapticDeviceAPI::HapticDeviceAPI(merai::RTMemoryLayout *rtLayout,
                                         merai::multi_ring_logger_memory *loggerMem)
            : rtLayout_(rtLayout), loggerMem_(loggerMem)
        {
            if (!rtLayout_)
            {
                log_warn(loggerMem_, "HapticDeviceAPI", 565, "[HapticDeviceAPI] Warning: rtLayout_ is null.");
            }
            if (!loggerMem_)
            {
                log_warn(loggerMem_, "HapticDeviceAPI", 525, "[HapticDeviceAPI] Warning: loggerMem_ is null.");
            }
        }

        void HapticDeviceAPI::logInfo(const char *msg) const
        {
            if (!loggerMem_)
                return;
            // Example:
            // loggerMem_->push("HapticDeviceAPI", msg);
        }

        // ---------------------------------------------------------------------
        // Internal helper: get pointer to the current array of feedback data.
        // Return nullptr if rtLayout_ is not available.
        const merai::JointFeedbackData *HapticDeviceAPI::getCurrentFeedback() const
        {
            if (!rtLayout_)
                return nullptr;

            // 1) find active index
            int activeIdx = rtLayout_->jointFeedbackBuffer.activeIndex.load(std::memory_order_acquire);

            // 2) reference the feedback array
            const auto &feedbackArray = rtLayout_->jointFeedbackBuffer.buffer[activeIdx].data();

            // feedbackArray is an array of JointFeedbackData for each drive/joint
            return feedbackArray; // pointer to the first element
        }

        // ---------------------------------------------------------------------
        //  GET Methods
        // ---------------------------------------------------------------------

        double HapticDeviceAPI::getLinearVelocity() const
        {
            // For simplicity, let's assume "linear velocity" is the velocity of joint 0.
            // Real systems might do forward kinematics or sum multiple joints, etc.
            const auto fb = getCurrentFeedback();
            if (!fb)
                return 0.0;

            // Joint 0
            double linearVelocity = fb[0].motion.velocityActual; // Possibly in rad/s or something
            // TODO: convert to m/s if it's actually an angular measure
            return linearVelocity;
        }

        double HapticDeviceAPI::getAngularVelocityRad() const
        {
            // Let's assume joint 1 might represent an angular axis for the device base.
            // This is purely an example.
            const auto fb = getCurrentFeedback();
            if (!fb)
                return 0.0;

            // Suppose "velocityActual" for joint 1 is in rad/s already
            double angularVelocity = fb[1].motion.velocityActual;
            return angularVelocity;
        }

        double HapticDeviceAPI::getAngularVelocityDeg() const
        {
            return getAngularVelocityRad() * (180.0 / M_PI);
        }

        double HapticDeviceAPI::getGripperAngularVelocityRad() const
        {
            // Suppose the "gripper" is joint 6
            const auto fb = getCurrentFeedback();
            if (!fb)
                return 0.0;

            double gripperAngularVel = fb[6].motion.velocityActual;
            return gripperAngularVel;
        }

        double HapticDeviceAPI::getGripperAngularVelocityDeg() const
        {
            return getGripperAngularVelocityRad() * (180.0 / M_PI);
        }

        bool HapticDeviceAPI::getPosition(double &x, double &y, double &z) const
        {
            const auto fb = getCurrentFeedback();
            if (!fb)
                return false;

            // You have joint positions in fb[i].motion.positionActual, typically in radians or counts
            // In a real system, you'd do forward kinematics or a direct sensor read.
            // Example here: just treat joint 0's position as "x", joint 1 as "y", joint 2 as "z"
            // TOTALLY an example—adapt to your real logic.

            x = fb[0].motion.positionActual;
            y = fb[1].motion.positionActual;
            z = fb[2].motion.positionActual;

            return true;
        }

        bool HapticDeviceAPI::getOrientationRad(double &roll, double &pitch, double &yaw) const
        {
            const auto fb = getCurrentFeedback();
            if (!fb)
                return false;

            // In reality, you'd do forward kinematics on multiple joints.
            // Here, let's pretend joint 3 = roll, joint 4 = pitch, joint 5 = yaw (in radians).
            roll = fb[3].motion.positionActual;
            pitch = fb[4].motion.positionActual;
            yaw = fb[5].motion.positionActual;
            // Possibly scale them from counts -> radians

            return true;
        }

        bool HapticDeviceAPI::getOrientationDeg(double &roll, double &pitch, double &yaw) const
        {
            if (!getOrientationRad(roll, pitch, yaw))
            {
                return false;
            }
            roll *= (180.0 / M_PI);
            pitch *= (180.0 / M_PI);
            yaw *= (180.0 / M_PI);
            return true;
        }

        bool HapticDeviceAPI::getPositionAndOrientationRad(double &x, double &y, double &z,
                                                           double &roll, double &pitch, double &yaw) const
        {
            bool positionWithOrientation = getPosition(x, y, z);
            bool orientationWithPosition = getOrientationRad(roll, pitch, yaw);
            return positionWithOrientation && orientationWithPosition;
        }

        bool HapticDeviceAPI::getPositionAndOrientationDeg(double &x, double &y, double &z,
                                                           double &roll, double &pitch, double &yaw) const
        {
            bool ok = getPositionAndOrientationRad(x, y, z, roll, pitch, yaw);
            if (!ok)
                return false;

            roll *= (180.0 / M_PI);
            pitch *= (180.0 / M_PI);
            yaw *= (180.0 / M_PI);

            return true;
        }

        // ---------------------------------------------------------------------
        //  SET Methods
        // ---------------------------------------------------------------------

        void HapticDeviceAPI::setGravityCompensation(bool enable)
        {
            // There's no direct gravityComp in RTMemoryLayout.
            // This is a placeholder if you had a param or a shared setting for gravity comp.
            logInfo(enable ? "Gravity compensation enabled (stub)."
                           : "Gravity compensation disabled (stub).");
        }

        void HapticDeviceAPI::setBrakes(bool engaged)
        {
            // Similarly, no direct field in RTMemoryLayout for "brakes".
            logInfo(engaged ? "Brakes engaged (stub)."
                            : "Brakes released (stub).");
        }

        void HapticDeviceAPI::setDeviceAngleRad(double angle)
        {
            // Again, no direct field in RTMemoryLayout for "desired angle."
            // If you have a dedicated place, you'd write to it. We'll just log:
            char buf[128];
            std::snprintf(buf, sizeof(buf), "Device angle set to %.3f rad (stub).", angle);
            logInfo(buf);
        }

    } // namespace network_api
