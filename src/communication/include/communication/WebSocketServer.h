#ifndef WEB_SOCKET_SERVER_H
#define WEB_SOCKET_SERVER_H

#include <uwebsockets/App.h>  // Main uWebSockets header
#include "communication/MessageProtocol.h"  // If you want to parse messages here
#include "communication/CommConfig.h"       // For CommServerConfig
#include "communication/CommManager.h"      // For CommManager reference
#include <thread>
#include <atomic>
#include <memory>

namespace RealTimeSystem
{
    namespace Communication
    {
        class WebSocketServer
        {
        public:
            WebSocketServer(const CommServerConfig& config, CommManager& commManager);
            ~WebSocketServer();

            /**
             * @brief Start listening for WebSocket connections in a separate thread.
             * @return true if successfully started, false otherwise.
             */
            bool start();

            /**
             * @brief Stop the server (close all connections, stop thread).
             */
            void stop();

            /**
             * @brief Broadcast a text message to all connected clients.
             * @param message The string to broadcast.
             */
            void broadcastMessage(const std::string& message);

        private:
            /**
             * @brief Internal method that runs the uWebSockets event loop.
             *        Called on a separate thread by `start()`.
             */
            void runServerLoop();

            /**
             * @brief Optional: sets up SSL/TLS options if config_.useSSL is true.
             * @return true if successful, false otherwise.
             */
            bool setupTLS();

        private:
            CommServerConfig config_;
            CommManager& commManager_;

            // Thread control
            std::thread serverThread_;
            std::atomic<bool> running_{false};

            // We’ll store the uWS::App or uWS::SSLApp in a void* or variant if needed,
            // or we can use std::unique_ptr with a templated approach. For simplicity:
            std::unique_ptr<uWS::App> wsApp_;      // for non-TLS
            std::unique_ptr<uWS::SSLApp> wsSSLApp_; // for TLS

            // Keep track of connected WebSocket clients, e.g. using some ID mapping
            // or direct references to "uWS::WebSocket".
            // For a minimal example, we won’t store them. 
        };
    }
}

#endif // WEB_SOCKET_SERVER_H
