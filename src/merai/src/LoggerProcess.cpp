#include <iostream>
#include <thread>
#include <chrono>

#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"

int main()
{
    // Create or open the logger shared memory (read/write).
    // Adjust size or readOnly flag as needed.
    hand_control::merai::RAII_SharedMemory loggerShm(
        "/LoggerShm",
        sizeof(hand_control::merai::multi_ring_logger_memory)
    );

    auto* loggerPtr = reinterpret_cast<hand_control::merai::multi_ring_logger_memory*>(
        loggerShm.getPtr()
    );

    while (true)
    {
        hand_control::merai::shared_log_message msg;

        // Fieldbus logs
        while (hand_control::merai::pop_fieldbus_log(loggerPtr, msg))
        {
            std::cout << "[Fieldbus] [LVL=" << static_cast<int>(msg.level) << "] "
                      << "[CODE=" << msg.code << "] " << msg.text << "\n";
        }

        // Control logs
        while (hand_control::merai::pop_control_log(loggerPtr, msg))
        {
            std::cout << "[Control] [LVL=" << static_cast<int>(msg.level) << "] "
                      << "[CODE=" << msg.code << "] " << msg.text << "\n";
        }

        // Logic logs
        while (hand_control::merai::pop_logic_log(loggerPtr, msg))
        {
            std::cout << "[Logic] [LVL=" << static_cast<int>(msg.level) << "] "
                      << "[CODE=" << msg.code << "] " << msg.text << "\n";
        }

        // Sleep or wait on a condition variable, etc.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
