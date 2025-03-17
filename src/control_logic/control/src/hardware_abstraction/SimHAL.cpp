#include <iostream>
#include <cmath>  // for std::sin, etc.

#include "control/hardware_abstraction/SimHAL.h"

namespace hand_control
{
    namespace control
    {
        SimHAL::SimHAL(
            hand_control::merai::RTMemoryLayout*           rtLayout,
            const hand_control::merai::ParameterServer*    paramServerPtr,
            hand_control::merai::multi_ring_logger_memory* loggerMem
        )
            : rtLayout_(rtLayout),
              paramServerPtr_(paramServerPtr),
              loggerMem_(loggerMem)
        {
            if (!rtLayout_)
            {
                std::cerr << "[SimHAL] Warning: Null rtLayout.\n";
            }
            if (!paramServerPtr_)
            {
                std::cerr << "[SimHAL] Warning: Null ParameterServer.\n";
            }
        }

        bool SimHAL::init()
        {
            // Setup driveCount_ from paramServerPtr_
            driveCount_ = paramServerPtr_->driveCount;
            if (driveCount_ > hand_control::merai::MAX_DRIVES)
            {
                driveCount_ = hand_control::merai::MAX_DRIVES;
            }

            // Zero out local arrays for joint data and drive control
            for (int i = 0; i < driveCount_; ++i)
            {
                localJointStates_[i].position = 0.0;
                localJointStates_[i].velocity = 0.0;
                localJointStates_[i].torque   = 0.0;

                localJointCommands_[i].position = 0.0;
                localJointCommands_[i].velocity = 0.0;
                localJointCommands_[i].torque   = 0.0;
            }

            return true;
        }

        bool SimHAL::read()
        {
            // 1) Simulate servo drive states in SI if desired
            // (e.g., simulate sensor feedback or drive state transitions)
            simulateDriveStateTransitions();

            // 2) Simulate I/O changes (e.g., toggle digital inputs, sine wave analog values, etc.)
            simulateJointIOChanges();

            return true;
        }

        bool SimHAL::write()
        {
            // 1) Accept new commands from localJointCommands_ / localDriveOutputs_
            // 2) Store these for the next cycle or simulation step
            // (e.g., simulate writing control commands to hardware)
            return true;
        }

        void SimHAL::simulateDriveStateTransitions()
        {
            // // Simulate state transitions for each drive based on its control word or mode.
            // for (int i = 0; i < driveCount_; ++i)
            // {
            //     // Example: If controlWord is 0x000F, enable operation (simulate status)
            //     if (DriveInputControl_[i].controlWord == 0x000F)
            //     {
            //         DriveOutputControl_[i].statusWord |= 0x01; // Simulate "operation enabled" status
            //     }

            //     // You can add more complex logic here based on control bits or internal states
            // }
        }

        void SimHAL::simulateJointIOChanges()
        {
            // Example: Simulate digital and analog I/O signals for joints
            for (int i = 0; i < driveCount_; ++i)
            {
                // Toggle digital input
                localJointIOs_[i].digitalInputA = !localJointIOs_[i].digitalInputA;

                // Generate a sine wave analog input (e.g., simulating a sensor value)
                localJointIOs_[i].analogInput = std::sin(i * 0.1); // Simple sine wave pattern

                // Optionally, simulate outputs based on joint commands
                localJointIOs_[i].digitalOutputA = localJointCommands_[i].torque > 0.5;  // Example output logic
            }
        }
    } // namespace control
} // namespace hand_control
