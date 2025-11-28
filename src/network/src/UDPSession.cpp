#include "network_api/include/network_api/UDPSession.h"
#include <iostream>
#include <cstring>
#include <cstdio>

    namespace network_api
    {

        UDPSession::UDPSession() {}
        UDPSession::~UDPSession() { destroyServerHost(); }

        void UDPSession::setLogger(merai::multi_ring_logger_memory *loggerMem)
        {
            loggerMem_ = loggerMem;
        }

        void UDPSession::setDeviceAPI(HapticDeviceAPI *api)
        {
            deviceAPI_ = api;
        }

        void UDPSession::logInfo(const char *msg)
        {
            if (!loggerMem_)
                return;
            // e.g. loggerMem_->push("UDPSession", msg);
        }

        bool UDPSession::createServerHost(const char *ip, unsigned short port)
        {
            if (serverHost_)
            {
                logInfo("Server host already created.");
                return false;
            }

            ENetAddress address;
            if (!ip || std::strcmp(ip, "0.0.0.0") == 0 || ip[0] == '\0')
            {
                address.host = ENET_HOST_ANY;
            }
            else
            {
                enet_address_set_host(&address, ip);
            }
            address.port = port;

            serverHost_ = enet_host_create(&address,
                                           16, // max # of connections
                                           2,  // channels
                                           0,  // incoming bandwidth
                                           0); // outgoing bandwidth

            if (!serverHost_)
            {
                logInfo("Failed to create ENet server.");
                return false;
            }

            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "ENet server created on %s:%u",
                          ip, port);
            logInfo(buf);

            return true;
        }

        void UDPSession::pollEvents()
        {
            if (!serverHost_)
                return;

            ENetEvent event;
            while (enet_host_service(serverHost_, &event, 0) > 0)
            {
                switch (event.type)
                {
                case ENET_EVENT_TYPE_CONNECT:
                    handleConnect(event);
                    break;
                case ENET_EVENT_TYPE_RECEIVE:
                    handleReceive(event);
                    enet_packet_destroy(event.packet);
                    break;
                case ENET_EVENT_TYPE_DISCONNECT:
                    handleDisconnect(event);
                    break;
                default:
                    break;
                }
            }
        }

        void UDPSession::destroyServerHost()
        {
            if (serverHost_)
            {
                enet_host_destroy(serverHost_);
                serverHost_ = nullptr;
                logInfo("Server host destroyed.");
            }
        }

        void UDPSession::handleConnect(ENetEvent &event)
        {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "Client connected from %x:%u",
                          event.peer->address.host,
                          event.peer->address.port);
            logInfo(buf);
        }

        void UDPSession::handleReceive(ENetEvent &event)
        {
            char buf[128];
            std::snprintf(buf, sizeof(buf),
                          "Received %u bytes on channel %u",
                          event.packet->dataLength,
                          event.channelID);
            logInfo(buf);

            // Parse incoming data. For demonstration, treat it as a simple string command:
            std::string cmd((char *)event.packet->data, event.packet->dataLength);

            // If we have a device API, call relevant set methods
            if (deviceAPI_)
            {
                if (cmd.find("SET_GRAVITY_COMP=1") != std::string::npos)
                {
                    deviceAPI_->setGravityCompensation(true);
                }
                else if (cmd.find("SET_GRAVITY_COMP=0") != std::string::npos)
                {
                    deviceAPI_->setGravityCompensation(false);
                }
                else if (cmd.find("SET_BRAKES=1") != std::string::npos)
                {
                    deviceAPI_->setBrakes(true);
                }
                else if (cmd.find("SET_BRAKES=0") != std::string::npos)
                {
                    deviceAPI_->setBrakes(false);
                }
                // ... add more commands as needed ...
            }
        }

        void UDPSession::handleDisconnect(ENetEvent &event)
        {
            logInfo("Client disconnected.");
        }

        void UDPSession::sendHapticData(double linearVel, double angularVel)
        {
            if (!serverHost_)
                return;

            char buffer[64];
            std::snprintf(buffer, sizeof(buffer),
                          "VEL=%.3f,ANG=%.3f", linearVel, angularVel);

            ENetPacket *packet = enet_packet_create(buffer,
                                                    std::strlen(buffer) + 1,
                                                    ENET_PACKET_FLAG_UNSEQUENCED);
            for (size_t i = 0; i < serverHost_->peerCount; ++i)
            {
                ENetPeer *peer = &serverHost_->peers[i];
                if (peer->state == ENET_PEER_STATE_CONNECTED)
                {
                    enet_peer_send(peer, 0, packet);
                }
            }
            enet_host_flush(serverHost_);

            logInfo("Haptic data packet sent.");
        }

    } // namespace network_api
