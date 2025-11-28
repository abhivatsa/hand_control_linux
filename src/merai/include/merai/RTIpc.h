#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include "merai/RTMemoryLayout.h"

namespace merai
{
    inline bool validate_rt_layout(const RTMemoryLayout *rt)
    {
        return rt &&
               rt->magic == RT_MEMORY_MAGIC &&
               rt->version == RT_MEMORY_VERSION;
    }

    inline size_t clamp_drive_count(size_t driveCount)
    {
        return (driveCount > MAX_SERVO_DRIVES) ? static_cast<size_t>(MAX_SERVO_DRIVES) : driveCount;
    }

    // ---------------- Generic double buffer helpers ----------------
    template <typename T>
    inline int back_index(const DoubleBuffer<T> &db)
    {
        int active = db.activeIndex.load(std::memory_order_relaxed);
        return 1 - active;
    }

    // Producer publishes a fully written back buffer.
    template <typename T>
    inline void publish(DoubleBuffer<T> &db, int backIdx)
    {
        db.activeIndex.store(backIdx, std::memory_order_release);
    }

    // Consumer snapshot of the currently active buffer.
    template <typename T>
    inline void read_snapshot(const DoubleBuffer<T> &db, T &dst)
    {
        int idx = db.activeIndex.load(std::memory_order_acquire);
        dst = db.buffer[idx];
    }
} // namespace merai
