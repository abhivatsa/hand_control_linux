#include <iostream>
#include <thread>
#include <chrono>

#include "merai/SharedLogger.h"
#include "merai/RAII_SharedMemory.h"
#ifdef USE_SYSTEMD_JOURNAL
#include <systemd/sd-journal.h>
#endif

namespace
{
    void emit_log(const char* module, const hand_control::merai::shared_log_message& msg)
    {
        double ts_ms = static_cast<double>(msg.timestamp) / 1e6;
#ifdef USE_SYSTEMD_JOURNAL
        sd_journal_send("MESSAGE=%s", msg.text,
                        "PRIORITY=%d", static_cast<int>(msg.level),
                        "CODE=%d", msg.code,
                        "MODULE=%s", module,
                        "TIMESTAMP_MS=%.3f", ts_ms,
                        nullptr);
#else
        (void)module;
        (void)ts_ms;
#endif
    }
}

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
    if (loggerPtr->magic != 0x4C4F4747)
    {
        std::cerr << "[Logger] Invalid magic in shared memory. Exiting.\n";
        return 1;
    }
    if (loggerPtr->version != 1)
    {
        std::cerr << "[Logger] Unsupported logger version: " << loggerPtr->version << ". Exiting.\n";
        return 1;
    }

    while (true)
    {
        hand_control::merai::shared_log_message msg;

        // Fieldbus logs
        while (hand_control::merai::pop_fieldbus_log(loggerPtr, msg))
        {
            emit_log("Fieldbus", msg);
        }

        // Control logs
        while (hand_control::merai::pop_control_log(loggerPtr, msg))
        {
            emit_log("Control", msg);
        }

        // Logic logs
        while (hand_control::merai::pop_logic_log(loggerPtr, msg))
        {
            emit_log("Logic", msg);
        }

        // Report drops if any
        auto fb_drops = loggerPtr->fieldbus_ring.dropped.load(std::memory_order_relaxed);
        auto ctrl_drops = loggerPtr->control_ring.dropped.load(std::memory_order_relaxed);
        auto logic_drops = loggerPtr->logic_ring.dropped.load(std::memory_order_relaxed);
        if (fb_drops || ctrl_drops || logic_drops)
        {
            std::cout << "[Logger] drops - fieldbus:" << fb_drops
                      << " control:" << ctrl_drops
                      << " logic:" << logic_drops << "\n";
        }

        // Sleep or wait on a condition variable, etc.
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
