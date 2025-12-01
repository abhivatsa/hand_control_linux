#pragma once

#include <cstddef>
#include <cstdint>

#include "control/ControlData.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"

namespace control
{
    /**
     * @brief Manages CiA-402 drive state transitions at joint level.
     *
     * Reads statusWord from JointControlFeedback and writes controlWord
     * into JointControlCommand based on merai::DriveCommand and the
     * decoded merai::DriveStatus.
     */
    class DriveStateManager
    {
    public:
        DriveStateManager(std::size_t driveCount,
                          merai::multi_ring_logger_memory *loggerMem);

        bool init();

        void update(const ControlCycleInputs &in,
                    ControlCycleOutputs       &out);

    private:
        merai::DriveStatus decodeStatusword(std::uint16_t statusWord);

    private:
        std::size_t                    driveCount_ = 0;
        merai::multi_ring_logger_memory *loggerMem_ = nullptr;
    };

} // namespace control
