#include <iostream>
#include "WebSocketServer.h"
#include "CommConfig.h"

int main()
{
    // 1. Load config from JSON (pseudo code):
    ServerConfig cfg;
    cfg.url = "wss://*:5567";
    cfg.certificate = "/etc/ssl/certs/merai.pem";
    cfg.privateKey = "/etc/ssl/private/merai.key";

    // 2. Create and init server
    WebSocketServer server;
    if (!server.init(cfg)) {
        std::cerr << "Failed to init WebSocket server.\n";
        return 1;
    }

    // 3. Start server
    server.start();

    std::cout << "WebSocket server running on port 5567 with self-signed certificate.\n";
    std::cout << "Press ENTER to stop...\n";
    std::cin.get();

    // 4. Stop server
    server.stop();
    std::cout << "Server stopped. Goodbye!\n";

    return 0;
}

// #include "WebSocketServer.h"
// #include "CommManager.h"
// #include "MessageProtocol.h"
// #include <iostream>

// int main()
// {
//     // Example: Initialize server, manager, etc.
//     communication::CommManager commMgr;

//     // Example JSON message from front-end:
//     std::string incomingJson = R"({
//         "type": "START_MOTION",
//         "trajectoryId": "traj_001",
//         "velocityScale": 0.8
//     })";

//     auto parsedMsgOpt = communication::parseIncomingMessage(incomingJson);
//     if (parsedMsgOpt)
//     {
//         commMgr.handleIncomingMessage(*parsedMsgOpt);
//     }
//     else
//     {
//         std::cerr << "Failed to parse incoming message.\n";
//     }

//     // Possibly spin up WebSocket server, etc.
//     // ...
//     return 0;
// }

