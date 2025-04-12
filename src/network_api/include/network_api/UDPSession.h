#pragma once

#include <enet/enet.h>
#include "merai/SharedLogger.h"
#include "HapticDeviceAPI.h" // if we want to call deviceAPI

namespace hand_control
{
    namespace network_api
    {

        class UDPSession
        {
        public:
            UDPSession();
            ~UDPSession();

            // Create ENet server on IP and port. Return false if fails.
            bool createServerHost(const char *ip, unsigned short port);

            // Poll for incoming connections, packets, etc.
            void pollEvents();

            // Destroy the ENet server if it exists
            void destroyServerHost();

            // (Optional) Provide references to logger and device API
            void setLogger(merai::multi_ring_logger_memory *loggerMem);
            void setDeviceAPI(HapticDeviceAPI *api);

            // Example method to send device state
            void sendHapticData(double linearVel, double angularVel);

        private:
            ENetHost *serverHost_ = nullptr;

            // Pointers for logging and calling device methods
            merai::multi_ring_logger_memory *loggerMem_ = nullptr;
            HapticDeviceAPI *deviceAPI_ = nullptr;

            // Private helpers
            void handleConnect(ENetEvent &event);
            void handleReceive(ENetEvent &event);
            void handleDisconnect(ENetEvent &event);

            void logInfo(const char *msg);
        };

    } // namespace network_api
} // namespace hand_control
