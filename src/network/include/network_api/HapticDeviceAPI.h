#pragma once

#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include <cstddef> // for size_t

    namespace network_api
    {

        class HapticDeviceAPI
        {
        public:
            /// Constructor: needs a pointer to RTMemoryLayout (for joint data) and to logger memory
            HapticDeviceAPI(merai::RTMemoryLayout *rtLayout,
                            merai::multi_ring_logger_memory *loggerMem);

            /// Logging helper
            void logInfo(const char *msg) const;

            // ----------------------------------------------------------------
            //  GET Methods
            // ----------------------------------------------------------------

            /// getLinearVelocity(): returns a single “linear velocity” in m/s (or similar).
            double getLinearVelocity() const;

            /// getAngularVelocityRad(): returns a single angular velocity in rad/s (for the device base?).
            double getAngularVelocityRad() const;

            /// getAngularVelocityDeg(): same as above, but in deg/s.
            double getAngularVelocityDeg() const;

            /// getGripperAngularVelocityRad(): read from a separate “gripper” joint’s feedback, rad/s.
            double getGripperAngularVelocityRad() const;

            /// getGripperAngularVelocityDeg(): same as above, in deg/s.
            double getGripperAngularVelocityDeg() const;

            /// getPosition(): retrieves x, y, z in some linear units (m, cm, etc.).
            /// In reality, you'd convert from joint positions or store an external transform.
            /// Returns false if data is unavailable.
            bool getPosition(double &x, double &y, double &z) const;

            /// getOrientationRad(): e.g. roll/pitch/yaw in radians.
            bool getOrientationRad(double &roll, double &pitch, double &yaw) const;

            /// getOrientationDeg(): e.g. roll/pitch/yaw in degrees.
            bool getOrientationDeg(double &roll, double &pitch, double &yaw) const;

            /// getPositionAndOrientationRad(): combo, returning both position & orientation in rad.
            bool getPositionAndOrientationRad(double &x, double &y, double &z,
                                              double &roll, double &pitch, double &yaw) const;

            /// getPositionAndOrientationDeg(): combo in degrees.
            bool getPositionAndOrientationDeg(double &x, double &y, double &z,
                                              double &roll, double &pitch, double &yaw) const;

            // ----------------------------------------------------------------
            //  SET Methods (placeholder stubs, since RTMemoryLayout doesn't define them)
            // ----------------------------------------------------------------
            void setGravityCompensation(bool enable);
            void setBrakes(bool engaged);
            void setDeviceAngleRad(double angle);

        private:
            /// Pointer to your real-time layout. We'll read from jointFeedbackBuffer for feedback.
            merai::RTMemoryLayout *rtLayout_;

            /// Pointer to your shared memory logger for optional logging.
            merai::multi_ring_logger_memory *loggerMem_;

            /// Helper to safely read the current front feedback array.
            /// If the layout is null, returns nullptr.
            const merai::JointFeedbackData *getCurrentFeedback() const;
        };

    } // namespace network_api
