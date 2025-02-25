#include "communication/CommManager.h"
#include <iostream> // for demo logging
// Include other necessary headers for accessing your motion subsystem, etc.

namespace RealTimeSystem
{
    namespace Communication
    {
        CommManager::CommManager()
        {
            // Constructor
        }

        CommManager::~CommManager()
        {
            // Destructor
        }

        void CommManager::handleIncomingMessage(const ParsedMessage &msg)
        {
            switch (msg.messageType)
            {
            case MessageType::START_MOTION:
            case MessageType::STOP_MOTION:
            case MessageType::PAUSE_MOTION:
            case MessageType::RESUME_MOTION:
            case MessageType::E_STOP:
            case MessageType::JOG_COMMAND:
                handleMotionCommand(msg);
                break;

            case MessageType::SET_SPEED:
            case MessageType::SET_ACCEL_LIMIT:
            case MessageType::SET_TOOL_DATA:
            case MessageType::ENABLE_COLLISION_AVOIDANCE:
            case MessageType::SET_HOME_POSITION:
                handleConfigurationCommand(msg);
                break;

            case MessageType::GET_STATUS:
            case MessageType::SYSTEM_STATUS_UPDATE:
            case MessageType::SAFETY_STATE_UPDATE:
            case MessageType::ERROR_FAULT:
            case MessageType::WARNING_ALARM:
                handleStatusOrSafety(msg);
                break;

            case MessageType::GET_JOINT_STATES:
            case MessageType::JOINT_STATES_UPDATE:
            case MessageType::GET_POSE:
            case MessageType::POSE_UPDATE:
            case MessageType::LOG_REQUEST:
            case MessageType::LOG_RESPONSE:
                handleLogging(msg);
                break;

            default:
                // Handle unknown or not implemented
                std::cerr << "[CommManager] Unknown message type received.\n";
                break;
            }
        }

        void CommManager::handleMotionCommand(const ParsedMessage &msg)
        {
            // Example: Switch again or do if-else
            switch (msg.messageType)
            {
            case MessageType::START_MOTION:
                // parse any data from msg.data
                // call your motion subsystem to start
                std::cout << "[CommManager] Handling START_MOTION.\n";
                break;
            case MessageType::STOP_MOTION:
                // ...
                break;
            case MessageType::E_STOP:
                // ...
                break;
            // ...
            default:
                break;
            }
        }

        void CommManager::handleConfigurationCommand(const ParsedMessage &msg)
        {
            switch (msg.messageType)
            {
            case MessageType::SET_SPEED:
            {
                // Example: read "speedPercentage" from msg.data
                if (msg.data.contains("speedPercentage"))
                {
                    double spd = msg.data["speedPercentage"].get<double>();
                    // pass to your motion subsystem
                    std::cout << "[CommManager] Set speed to " << spd << "%.\n";
                }
                break;
            }
            case MessageType::SET_ACCEL_LIMIT:
                // ...
                break;
            case MessageType::ENABLE_COLLISION_AVOIDANCE:
                // ...
                break;
            // ...
            default:
                break;
            }
        }

        void CommManager::handleStatusOrSafety(const ParsedMessage &msg)
        {
            // GET_STATUS might be a request you handle differently than an UPDATE
            // ...
            std::cout << "[CommManager] Handling status/safety message.\n";
        }

        void CommManager::handleLogging(const ParsedMessage &msg)
        {
            switch (msg.messageType)
            {
            case MessageType::LOG_REQUEST:
                // Possibly fetch logs, then respond with LOG_RESPONSE
                break;
            case MessageType::LOG_RESPONSE:
                // This might be a message from the system or from the robot to the UI
                break;
            // ...
            default:
                break;
            }
        }
    }
} // namespace RealTimeSystem
