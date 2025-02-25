#include "WebSocketServer.h"
#include <iostream>

namespace RealTimeSystem
{
    namespace Communication
    {
        WebSocketServer::WebSocketServer(const CommServerConfig& config, CommManager& commManager)
            : config_(config), commManager_(commManager)
        {
        }

        WebSocketServer::~WebSocketServer()
        {
            stop();
        }

        bool WebSocketServer::start()
        {
            if (running_.exchange(true)) {
                // Already running
                std::cerr << "[WebSocketServer] start() called but server is already running.\n";
                return false;
            }

            // If TLS is enabled, set up the certificate/key:
            if (config_.useSSL) {
                if (!setupTLS()) {
                    std::cerr << "[WebSocketServer] TLS setup failed.\n";
                    running_ = false;
                    return false;
                }
            }

            // Launch a new thread to run the server event loop
            serverThread_ = std::thread(&WebSocketServer::runServerLoop, this);

            return true;
        }

        void WebSocketServer::stop()
        {
            if (!running_.exchange(false)) {
                // Already stopped
                return;
            }

            // uWebSockets doesn't have a "stop()" call in the simplest sense,
            // but we can gracefully shut down the event loop by capturing a reference
            // to the app and calling close on all websockets, etc.
            // For now, we'll just wait for the thread to join.

            if (serverThread_.joinable()) {
                serverThread_.join();
            }

            std::cout << "[WebSocketServer] Server stopped.\n";
        }

        bool WebSocketServer::setupTLS()
        {
            // For uWebSockets TLS, you typically pass options directly to SSLApp().
            // We'll just verify the files exist, etc. in production code.
            // Return true if everything is good.

            // Example check:
            // 1) Check that config_.sslCertPath and config_.sslKeyPath are not empty
            // 2) Possibly check file readability

            return true;
        }

        void WebSocketServer::runServerLoop()
        {
            try {
                // If useSSL == true, create an SSLApp with your cert/key
                if (config_.useSSL) {
                    /* Example for passing TLS options:
                     * ssl_options.key_file_name = config_.sslKeyPath.c_str();
                     * ssl_options.cert_file_name = config_.sslCertPath.c_str();
                     * ssl_options.passphrase = "";
                     * // etc.
                     */
                    auto sslApp = uWS::SSLApp({
                        .key_file_name = config_.sslKeyPath.c_str(),
                        .cert_file_name = config_.sslCertPath.c_str()
                    });
                    
                    wsSSLApp_ = std::make_unique<uWS::SSLApp>(std::move(sslApp));

                    // Register WebSocket behavior
                    wsSSLApp_->ws<false>("/*", {   // <false> = non-message compression, adjust if needed
                        .open = [this](auto *ws) {
                            std::cout << "[WebSocketServer] New TLS WebSocket connection opened.\n";
                            // Optionally track ws in a container for broadcast
                        },
                        .message = [this](auto *ws, std::string_view msg, uWS::OpCode opCode) {
                            // Parse the message or pass to CommManager
                            // Example:
                            auto parsedOpt = MessageProtocol::parseIncomingMessage(std::string(msg));
                            if (parsedOpt) {
                                commManager_.handleIncomingMessage(*parsedOpt);
                            } else {
                                std::cerr << "[WebSocketServer] Failed to parse incoming msg.\n";
                            }
                        },
                        .close = [this](auto *ws, int code, std::string_view message) {
                            std::cout << "[WebSocketServer] WebSocket closed: " << code << "\n";
                            // Remove from any tracking
                        }
                    });

                    // Listen on config_.port
                    wsSSLApp_->listen(config_.port, [this](auto *token) {
                        if (token) {
                            std::cout << "[WebSocketServer] Listening on port " << config_.port << " (TLS)\n";
                        } else {
                            std::cerr << "[WebSocketServer] Failed to listen on port " << config_.port << "\n";
                            running_ = false;
                        }
                    });

                    // If everything is okay, run the event loop
                    if (running_) {
                        wsSSLApp_->run();
                    }
                }
                else {
                    // Non-TLS mode
                    auto app = uWS::App();

                    wsApp_ = std::make_unique<uWS::App>(std::move(app));

                    // Setup event handlers
                    wsApp_->ws<false>("/*", {
                        .open = [this](auto *ws) {
                            std::cout << "[WebSocketServer] New WebSocket connection opened.\n";
                            // Optionally track ws for broadcast
                        },
                        .message = [this](auto *ws, std::string_view msg, uWS::OpCode opCode) {
                            auto parsedOpt = MessageProtocol::parseIncomingMessage(std::string(msg));
                            if (parsedOpt) {
                                commManager_.handleIncomingMessage(*parsedOpt);
                            } else {
                                std::cerr << "[WebSocketServer] Parse error.\n";
                            }
                        },
                        .close = [this](auto *ws, int code, std::string_view message) {
                            std::cout << "[WebSocketServer] WebSocket closed: " << code << "\n";
                            // Remove from tracking
                        }
                    });

                    wsApp_->listen(config_.port, [this](auto *token) {
                        if (token) {
                            std::cout << "[WebSocketServer] Listening on port " << config_.port << " (non-TLS)\n";
                        } else {
                            std::cerr << "[WebSocketServer] Failed to listen on port " << config_.port << "\n";
                            running_ = false;
                        }
                    });

                    if (running_) {
                        wsApp_->run();
                    }
                }
            }
            catch (std::exception &e) {
                std::cerr << "[WebSocketServer] runServerLoop exception: " << e.what() << "\n";
                running_ = false;
            }
        }

        void WebSocketServer::broadcastMessage(const std::string& message)
        {
            // For a minimal example, you need to track connections (i.e., store a list of ws pointers).
            // Then you can do something like:
            //
            // for (auto& ws : clients_) {
            //     ws->send(message, uWS::OpCode::TEXT);
            // }
            //
            // Here, we don’t show a container, but you'd maintain it in `.open` and `.close`.
            //
            // If you want a simpler approach, uWebSockets also has "publish/subscribe" channels.
            // You could publish to a channel, and each ws is subscribed.
            // e.g. ws->subscribe("myChannel"); app->publish("myChannel", message);
            //
            // We'll just log it for now:
            std::cout << "[WebSocketServer] (broadcast stub) " << message << "\n";
        }
    }
}
