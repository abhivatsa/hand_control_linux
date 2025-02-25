#pragma once

#include <atomic>
#include <cstdint>
#include <cstring>
#include <cstdio>  // for snprintf, optional

namespace motion_control
{
    namespace merai
    {
        // =============================
        // 1) Basic Log Definitions
        // =============================
        enum class shared_log_level : int
        {
            debug = 0,
            info,
            warn,
            error
        };

        struct shared_log_message
        {
            uint64_t         timestamp;  // e.g. microseconds or system time
            shared_log_level level;
            int              code;
            char             text[64];   // short text, truncated if longer
        };

        // =============================
        // 2) Single Ring Buffer (SPSC)
        // =============================
        constexpr std::size_t log_buffer_size = 1024;

        struct logger_shared_memory
        {
            shared_log_message buffer[log_buffer_size];
            std::atomic<size_t> head {0}; // producer
            std::atomic<size_t> tail {0}; // consumer
        };

        inline void push_log_message(logger_shared_memory* shm, const shared_log_message& msg)
        {
            size_t head      = shm->head.load(std::memory_order_relaxed);
            size_t next_head = (head + 1) % log_buffer_size;

            // Write the item
            shm->buffer[head] = msg;

            // Publish new head
            shm->head.store(next_head, std::memory_order_release);

            // If buffer was full, move tail
            if (next_head == shm->tail.load(std::memory_order_acquire))
            {
                size_t old_tail = (shm->tail.load(std::memory_order_relaxed) + 1) % log_buffer_size;
                shm->tail.store(old_tail, std::memory_order_release);
            }
        }

        inline bool pop_log_message(logger_shared_memory* shm, shared_log_message& out_msg)
        {
            size_t tail = shm->tail.load(std::memory_order_relaxed);
            if (tail == shm->head.load(std::memory_order_acquire))
            {
                // buffer empty
                return false;
            }
            out_msg = shm->buffer[tail];
            size_t next_tail = (tail + 1) % log_buffer_size;
            shm->tail.store(next_tail, std::memory_order_release);
            return true;
        }

        // =============================
        // 3) Multi-Ring Buffer Layout
        // =============================
        struct multi_ring_logger_memory
        {
            logger_shared_memory fieldbus_ring;
            logger_shared_memory control_ring;
            logger_shared_memory logic_ring;
            // Add more ring buffers here if needed (e.g. comm_ring, safety_ring, etc.)
        };

        // =============================
        // 4) Push/Pop for Each Module
        // =============================
        inline void push_fieldbus_log(multi_ring_logger_memory* multi, const shared_log_message& msg)
        {
            push_log_message(&multi->fieldbus_ring, msg);
        }

        inline bool pop_fieldbus_log(multi_ring_logger_memory* multi, shared_log_message& out_msg)
        {
            return pop_log_message(&multi->fieldbus_ring, out_msg);
        }

        inline void push_control_log(multi_ring_logger_memory* multi, const shared_log_message& msg)
        {
            push_log_message(&multi->control_ring, msg);
        }

        inline bool pop_control_log(multi_ring_logger_memory* multi, shared_log_message& out_msg)
        {
            return pop_log_message(&multi->control_ring, out_msg);
        }

        inline void push_logic_log(multi_ring_logger_memory* multi, const shared_log_message& msg)
        {
            push_log_message(&multi->logic_ring, msg);
        }

        inline bool pop_logic_log(multi_ring_logger_memory* multi, shared_log_message& out_msg)
        {
            return pop_log_message(&multi->logic_ring, out_msg);
        }

        // =============================
        // 5) Generic Logging Helpers
        // =============================
        inline void push_log_message_for_module(multi_ring_logger_memory* multi,
                                                const char* module_name,
                                                const shared_log_message& msg)
        {
            if (std::strcmp(module_name, "Fieldbus") == 0)
            {
                push_fieldbus_log(multi, msg);
            }
            else if (std::strcmp(module_name, "Control") == 0)
            {
                push_control_log(multi, msg);
            }
            else if (std::strcmp(module_name, "Logic") == 0)
            {
                push_logic_log(multi, msg);
            }
            // else if ... more modules
        }

        inline void log_message(multi_ring_logger_memory* multi,
                                const char* module_name,
                                shared_log_level level,
                                int code,
                                const char* text)
        {
            shared_log_message msg{};
            msg.timestamp = 0ULL; // replace with your real timestamp function
            msg.level     = level;
            msg.code      = code;
            std::strncpy(msg.text, text, sizeof(msg.text) - 1);

            push_log_message_for_module(multi, module_name, msg);
        }

        // Additional convenience wrappers for each log level
        inline void log_debug(multi_ring_logger_memory* multi,
                              const char* module_name,
                              int code,
                              const char* text)
        {
            log_message(multi, module_name, shared_log_level::debug, code, text);
        }

        inline void log_info(multi_ring_logger_memory* multi,
                             const char* module_name,
                             int code,
                             const char* text)
        {
            log_message(multi, module_name, shared_log_level::info, code, text);
        }

        inline void log_warn(multi_ring_logger_memory* multi,
                             const char* module_name,
                             int code,
                             const char* text)
        {
            log_message(multi, module_name, shared_log_level::warn, code, text);
        }

        inline void log_error(multi_ring_logger_memory* multi,
                              const char* module_name,
                              int code,
                              const char* text)
        {
            log_message(multi, module_name, shared_log_level::error, code, text);
        }
    } // namespace merai
} // namespace motion_control
