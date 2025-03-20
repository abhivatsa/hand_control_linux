#include "network_api/UDPSession.h"
// #include <enet/enet.h>
#include <iostream>

namespace network_api {

UDPSession::UDPSession() {
    // If using ENet, perhaps call enet_initialize() globally.
}

UDPSession::~UDPSession() {
    close();
}

bool UDPSession::open(const std::string& ip, unsigned short port) {
    // If using ENet:
    // ENetAddress address;
    // enet_address_set_host(&address, ip.c_str());
    // address.port = port;
    // host_ = enet_host_create(&address, ...);
    // return (host_ != nullptr);

    // If using raw sockets, open them here.

    std::cout << "UDPSession opened on " << ip << ":" << port << "\n";
    return true; // or false if failure
}

void UDPSession::close() {
    // Cleanup
    // if (host_) {
    //     enet_host_destroy(host_);
    //     host_ = nullptr;
    // }
}

bool UDPSession::sendPacket(const void* data, size_t size) {
    // e.g., ENetPacket* packet = enet_packet_create(data, size, ENET_PACKET_FLAG_UNSEQUENCED);
    // enet_host_broadcast(host_, 0, packet);

    // or raw sendto() for UDP
    return true;
}

bool UDPSession::receivePacket(std::vector<uint8_t>& outData) {
    // Example with ENet:
    // ENetEvent event;
    // while (enet_host_service(host_, &event, 0) > 0) {
    //     if (event.type == ENET_EVENT_TYPE_RECEIVE) {
    //         outData.assign(event.packet->data, event.packet->data + event.packet->dataLength);
    //         enet_packet_destroy(event.packet);
    //         return true;
    //     }
    // }
    return false;
}

} // namespace network_api
