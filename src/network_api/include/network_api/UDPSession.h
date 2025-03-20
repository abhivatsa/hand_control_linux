#pragma once

#include <string>
#include <vector>
// Forward-declare ENet types if needed
// #include <enet/enet.h>

namespace network_api {

class UDPSession {
public:
    UDPSession();
    ~UDPSession();

    // Open/close session
    bool open(const std::string& ip, unsigned short port);
    void close();

    // Send data
    bool sendPacket(const void* data, size_t size);

    // Poll or receive data
    // Return either raw bytes or a higher-level Packet structure
    bool receivePacket(std::vector<uint8_t>& outData);

    // Any additional methods for ENet initialization, event loops, etc.

private:
    // e.g., ENetHost* host_ = nullptr;
    // e.g., ENetPeer* peer_ = nullptr;
    // or raw socket if not using ENet
};

} // namespace network_api
