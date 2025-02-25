#include <iostream>
#include <cmath>  // for std::sin, etc.

#include "control/hardware_abstraction/MockHardwareAbstractionLayer.h"

namespace motion_control
{
    namespace control
    {
        MockHardwareAbstractionLayer::MockHardwareAbstractionLayer(
            motion_control::merai::RTMemoryLayout*           rtLayout,
            const motion_control::merai::ParameterServer*    paramServerPtr,
            motion_control::merai::multi_ring_logger_memory* loggerMem
        )
            : rtLayout_(rtLayout),
              paramServerPtr_(paramServerPtr),
              loggerMem_(loggerMem)
        {
            if (!rtLayout_)
            {
                std::cerr << "[MockHAL] Warning: Null rtLayout.\n";
            }
            if (!paramServerPtr_)
            {
                std::cerr << "[MockHAL] Warning: Null ParameterServer.\n";
            }
        }

        bool MockHardwareAbstractionLayer::init()
        {
            // Setup driveCount_, ioCount_ from paramServerPtr_ if needed
            // Zero out local arrays, etc.
            return true;
        }

        bool MockHardwareAbstractionLayer::read()
        {
            // 1) Simulate servo drive states in SI if desired
            // 2) Possibly interpret localDriveOutputs_ => update localDriveInputs_ (simulate CiA 402 transitions)
            simulateDriveStateTransitions();

            // 3) Simulate I/O changes (toggle digital, sine wave analog, etc.)
            return true;
        }

        bool MockHardwareAbstractionLayer::write()
        {
            // Accept new commands from localJointCommands_ / localDriveOutputs_
            // Possibly store for next cycle
            return true;
        }

        void MockHardwareAbstractionLayer::simulateDriveStateTransitions()
        {
            // Example:
            // if out[i].controlWord = 0x000F => enable operation
            // then set in[i].statusWord bits for operationEnabled
        }
    } // namespace control
} // namespace motion_control
