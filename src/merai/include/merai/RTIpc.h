#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "merai/RTMemoryLayout.h"

namespace merai
{
    inline bool validate_rt_layout(const RTMemoryLayout* rt)
    {
        return rt &&
               rt->magic   == RT_MEMORY_MAGIC &&
               rt->version == RT_MEMORY_VERSION;
    }

    inline std::size_t clamp_drive_count(std::size_t driveCount)
    {
        return (driveCount > MAX_SERVO_DRIVES) ? MAX_SERVO_DRIVES : driveCount;
    }

    // ----------------------------------------------------
    // Generic double buffer helpers
    //
    // Single-producer, multi-consumer pattern:
    //
    // Producer (fieldbus/control/etc):
    //   int backIdx = back_index(db);
    //   auto& buf   = db.buffer[backIdx];
    //   // fill buf completely
    //   publish(db, backIdx);
    //
    // Consumer:
    //   T local{};
    //   read_snapshot(db, local);
    //
    // The seqlock-style sequence counter ensures the consumer
    // never sees a torn snapshot, even if producer loops faster
    // or gets jittered on the RT scheduler.
    // ----------------------------------------------------

    // Producer: choose back buffer and mark write in progress.
    template <typename T>
    inline int back_index(DoubleBuffer<T>& db)
    {
        // Mark writer entering (sequence becomes odd).
        db.sequence.fetch_add(1u, std::memory_order_acq_rel);

        int active = db.activeIndex.load(std::memory_order_relaxed);
        return 1 - active;
    }

    // Producer: publish a fully written back buffer.
    template <typename T>
    inline void publish(DoubleBuffer<T>& db, int backIdx)
    {
        // Ensure all writes to the back buffer are visible before flipping.
        std::atomic_thread_fence(std::memory_order_release);

        db.activeIndex.store(backIdx, std::memory_order_release);

        // Mark writer leaving (sequence becomes even).
        db.sequence.fetch_add(1u, std::memory_order_acq_rel);
    }

    // Consumer: seqlock-style snapshot of the active buffer.
    template <typename T>
    inline void read_snapshot(const DoubleBuffer<T>& db, T& dst)
    {
        for (;;)
        {
            std::uint32_t start = db.sequence.load(std::memory_order_acquire);
            if (start & 1u)
            {
                // Writer is in progress; retry.
                continue;
            }

            int idx = db.activeIndex.load(std::memory_order_acquire);
            dst = db.buffer[idx];

            std::atomic_thread_fence(std::memory_order_acquire);
            std::uint32_t end = db.sequence.load(std::memory_order_acquire);

            if (start == end && !(end & 1u))
            {
                // No writer touched the buffer during this copy.
                break;
            }
        }
    }

} // namespace merai
