#include <iostream>
#include "merai/SharedLogger.h"

using namespace hand_control::merai;

int main()
{
    multi_ring_logger_memory mem{};

    if (mem.magic != multi_ring_logger_memory::MAGIC ||
        mem.version != multi_ring_logger_memory::VERSION)
    {
        std::cerr << "Logger magic/version mismatch\n";
        return 1;
    }

    shared_log_message msg{};
    msg.level = shared_log_level::info;
    msg.code = 42;
    std::strncpy(msg.text, "hello", sizeof(msg.text) - 1);
    msg.text_length = static_cast<uint32_t>(std::strlen(msg.text));
    msg.timestamp = 123;

    // Push more than buffer size to force drops
    for (size_t i = 0; i < log_buffer_size + 10; ++i)
    {
        push_fieldbus_log(&mem, msg);
    }

    // At least one drop should have occurred
    if (mem.fieldbus_ring.dropped.load(std::memory_order_relaxed) == 0)
    {
        std::cerr << "Expected drops not recorded\n";
        return 1;
    }

    // Pop one message and validate contents
    shared_log_message out{};
    if (!pop_fieldbus_log(&mem, out))
    {
        std::cerr << "Pop failed\n";
        return 1;
    }
    if (out.code != msg.code || out.text_length == 0)
    {
        std::cerr << "Message content mismatch\n";
        return 1;
    }

    return 0;
}

