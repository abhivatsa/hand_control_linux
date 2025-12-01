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
    int to_journal_priority(merai::shared_log_level level)
    {
        // syslog/journald priorities:
        // 0=emerg, 1=alert, 2=crit, 3=err, 4=warning, 5=notice, 6=info, 7=debug
        switch (level)
        {
        case merai::shared_log_level::debug: return 7;
        case merai::shared_log_level::info:  return 6;
        case merai::shared_log_level::warn:  return 4;
        case merai::shared_log_level::error: return 3;
        default:                             return 6;
        }
    }

    void emit_log(const char* module, const merai::shared_log_message& msg)
    {
        double ts_ms = static_cast<double>(msg.timestamp) / 1e6;

    #ifdef USE_SYSTEMD_JOURNAL
        const int pri = to_journal_priority(msg.level);

        sd_journal_send("MESSAGE=%s",      msg.text,
                        "PRIORITY=%d",     pri,
                        "CODE=%d",         msg.code,
                        "MODULE=%s",       module,
                        "TIMESTAMP_MS=%.3f", ts_ms,
                        nullptr);
    #else
        std::cout << "[" << module << "] "
                  << ts_ms << " ms "
                  << "level=" << static_cast<int>(msg.level)
                  << " code=" << msg.code
                  << " msg=" << msg.text
                  << std::endl;
    #endif
    }
} // namespace

int main()
{
    // Logger is the consumer and *must* update the ring's tail,
    // so we open the SHM read/write (not read-only).
    merai::RAII_SharedMemory loggerShm(
        "/LoggerShm",
        sizeof(merai::multi_ring_logger_memory),
        /*readOnly=*/false);

    auto* loggerPtr = static_cast<merai::multi_ring_logger_memory*>(
        loggerShm.getPtr());

    if (loggerPtr->magic != merai::multi_ring_logger_memory::MAGIC)
    {
        std::cerr << "[Logger] Invalid magic in shared memory. Exiting.\n";
        return 1;
    }
    if (loggerPtr->version != merai::multi_ring_logger_memory::VERSION)
    {
        std::cerr << "[Logger] Unsupported logger version: "
                  << loggerPtr->version << ". Exiting.\n";
        return 1;
    }

    while (true)
    {
        merai::shared_log_message msg;

        // Fieldbus logs
        while (merai::pop_fieldbus_log(loggerPtr, msg))
        {
            emit_log("Fieldbus", msg);
        }

        // Control logs
        while (merai::pop_control_log(loggerPtr, msg))
        {
            emit_log("Control", msg);
        }

        // Logic logs
        while (merai::pop_logic_log(loggerPtr, msg))
        {
            emit_log("Logic", msg);
        }

        // Report drops if any
        auto fb_drops   = loggerPtr->fieldbus_ring.dropped.load(std::memory_order_relaxed);
        auto ctrl_drops = loggerPtr->control_ring.dropped.load(std::memory_order_relaxed);
        auto logic_drops= loggerPtr->logic_ring.dropped.load(std::memory_order_relaxed);

        if (fb_drops || ctrl_drops || logic_drops)
        {
            std::cout << "[Logger] drops - fieldbus:" << fb_drops
                      << " control:" << ctrl_drops
                      << " logic:"   << logic_drops
                      << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return 0;
}
