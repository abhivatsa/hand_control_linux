#ifndef COMM_MANAGER_H
#define COMM_MANAGER_H

#include "communication/MessageProtocol.h"

namespace RealTimeSystem
{
    namespace Communication
    {
        class CommManager
        {
        public:
            // Constructor, destructor, etc.
            CommManager();
            ~CommManager();

            // Called when a new WebSocket message arrives
            void handleIncomingMessage(const ParsedMessage &msg);

        private:
            // Example private handlers for each major category
            void handleMotionCommand(const ParsedMessage &msg);
            void handleConfigurationCommand(const ParsedMessage &msg);
            void handleStatusOrSafety(const ParsedMessage &msg);
            void handleLogging(const ParsedMessage &msg);

            // ... possibly more ...
        };
    } // namespace communication
}