#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>
#include "merai/RTMemoryLayout.h"

namespace seven_axis_robot
{
    namespace merai
    {
        inline bool validate_rt_layout(const RTMemoryLayout* rt)
        {
            return rt &&
                   rt->magic == RT_MEMORY_MAGIC &&
                   rt->version == RT_MEMORY_VERSION;
        }

        inline size_t clamp_drive_count(size_t driveCount)
        {
            return (driveCount > MAX_SERVO_DRIVES) ? static_cast<size_t>(MAX_SERVO_DRIVES) : driveCount;
        }

        struct BufferReadMeta
        {
            uint64_t seq = 0;
            bool fresh = true;
            int frontIndex = 0;
        };

        // ---------------- Servo buffer helpers ----------------
        inline int servo_tx_back_index(const ServoBuffers& buf)
        {
            int front = buf.txFrontIndex.load(std::memory_order_acquire);
            return 1 - front;
        }

        inline int servo_rx_back_index(const ServoBuffers& buf)
        {
            int front = buf.rxFrontIndex.load(std::memory_order_acquire);
            return 1 - front;
        }

        inline void publish_servo_tx(ServoBuffers& buf, int backIdx)
        {
            buf.txFrontIndex.store(backIdx, std::memory_order_release);
            buf.txSeq.fetch_add(1, std::memory_order_release);
        }

        inline void publish_servo_rx(ServoBuffers& buf, int backIdx)
        {
            buf.rxFrontIndex.store(backIdx, std::memory_order_release);
            buf.rxSeq.fetch_add(1, std::memory_order_release);
        }

        inline BufferReadMeta read_servo_rx(const ServoBuffers& buf,
                                            ServoSharedData& out,
                                            uint64_t lastSeq)
        {
            BufferReadMeta meta{};
            meta.seq = buf.rxSeq.load(std::memory_order_acquire);
            meta.frontIndex = buf.rxFrontIndex.load(std::memory_order_acquire);
            meta.fresh = (meta.seq != lastSeq);
            out = buf.buffer[meta.frontIndex];
            return meta;
        }

        inline BufferReadMeta read_servo_tx(const ServoBuffers& buf,
                                            ServoSharedData& out,
                                            uint64_t lastSeq)
        {
            BufferReadMeta meta{};
            meta.seq = buf.txSeq.load(std::memory_order_acquire);
            meta.frontIndex = buf.txFrontIndex.load(std::memory_order_acquire);
            meta.fresh = (meta.seq != lastSeq);
            out = buf.buffer[meta.frontIndex];
            return meta;
        }

        // ---------------- Generic double buffer helpers ----------------
        template <typename T>
        inline int back_index(const DoubleBuffer<T>& buf)
        {
            int front = buf.frontIndex.load(std::memory_order_acquire);
            return 1 - front;
        }

        template <typename T>
        inline void publish(DoubleBuffer<T>& buf, int backIdx)
        {
            buf.frontIndex.store(backIdx, std::memory_order_release);
            buf.seq.fetch_add(1, std::memory_order_release);
        }

        template <typename T>
        inline BufferReadMeta read_latest(const DoubleBuffer<T>& buf,
                                          T& out,
                                          uint64_t lastSeq)
        {
            BufferReadMeta meta{};
            meta.seq = buf.seq.load(std::memory_order_acquire);
            meta.frontIndex = buf.frontIndex.load(std::memory_order_acquire);
            meta.fresh = (meta.seq != lastSeq);
            out = buf.buffer[meta.frontIndex];
            return meta;
        }
    } // namespace merai
} // namespace seven_axis_robot
