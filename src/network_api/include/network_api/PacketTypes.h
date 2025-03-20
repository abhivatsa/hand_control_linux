#pragma once

#include <cstdint>
#include <array>

// Example packet containing minimal haptic device data
struct HapticStatePacket {
    float jointPositions[7];   // 7 DOF positions
    float jointVelocities[7];  // 7 DOF velocities
    float forces[7];           // 7 DOF force readings
    // ... add more if needed
};

struct CommandPacket {
    // E.g., new setpoints or device configuration commands from external user
    float desiredPositions[7];
    bool enableForceFeedback;
    // ...
};
