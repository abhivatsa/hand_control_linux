#include "communication/MessageProtocol.h"
#include <unordered_map>

namespace RealTimeSystem
{
    namespace Communication
    {
        // A helper table for string <-> enum conversions
        static const std::unordered_map<std::string, MessageType> stringToEnumMap = {
            {"START_MOTION", MessageType::START_MOTION},
            {"STOP_MOTION", MessageType::STOP_MOTION},
            {"PAUSE_MOTION", MessageType::PAUSE_MOTION},
            {"RESUME_MOTION", MessageType::RESUME_MOTION},
            {"E_STOP", MessageType::E_STOP},
            {"JOG_COMMAND", MessageType::JOG_COMMAND},

            {"SET_SPEED", MessageType::SET_SPEED},
            {"SET_ACCEL_LIMIT", MessageType::SET_ACCEL_LIMIT},
            {"SET_TOOL_DATA", MessageType::SET_TOOL_DATA},
            {"ENABLE_COLLISION_AVOIDANCE", MessageType::ENABLE_COLLISION_AVOIDANCE},
            {"SET_HOME_POSITION", MessageType::SET_HOME_POSITION},

            {"GET_STATUS", MessageType::GET_STATUS},
            {"SYSTEM_STATUS_UPDATE", MessageType::SYSTEM_STATUS_UPDATE},
            {"SAFETY_STATE_UPDATE", MessageType::SAFETY_STATE_UPDATE},
            {"ERROR_FAULT", MessageType::ERROR_FAULT},
            {"WARNING_ALARM", MessageType::WARNING_ALARM},

            {"GET_JOINT_STATES", MessageType::GET_JOINT_STATES},
            {"JOINT_STATES_UPDATE", MessageType::JOINT_STATES_UPDATE},
            {"GET_POSE", MessageType::GET_POSE},
            {"POSE_UPDATE", MessageType::POSE_UPDATE},
            {"LOG_REQUEST", MessageType::LOG_REQUEST},
            {"LOG_RESPONSE", MessageType::LOG_RESPONSE},
        };

        static const std::unordered_map<MessageType, std::string> enumToStringMap = {
            {MessageType::START_MOTION, "START_MOTION"},
            {MessageType::STOP_MOTION, "STOP_MOTION"},
            {MessageType::PAUSE_MOTION, "PAUSE_MOTION"},
            {MessageType::RESUME_MOTION, "RESUME_MOTION"},
            {MessageType::E_STOP, "E_STOP"},
            {MessageType::JOG_COMMAND, "JOG_COMMAND"},

            {MessageType::SET_SPEED, "SET_SPEED"},
            {MessageType::SET_ACCEL_LIMIT, "SET_ACCEL_LIMIT"},
            {MessageType::SET_TOOL_DATA, "SET_TOOL_DATA"},
            {MessageType::ENABLE_COLLISION_AVOIDANCE, "ENABLE_COLLISION_AVOIDANCE"},
            {MessageType::SET_HOME_POSITION, "SET_HOME_POSITION"},

            {MessageType::GET_STATUS, "GET_STATUS"},
            {MessageType::SYSTEM_STATUS_UPDATE, "SYSTEM_STATUS_UPDATE"},
            {MessageType::SAFETY_STATE_UPDATE, "SAFETY_STATE_UPDATE"},
            {MessageType::ERROR_FAULT, "ERROR_FAULT"},
            {MessageType::WARNING_ALARM, "WARNING_ALARM"},

            {MessageType::GET_JOINT_STATES, "GET_JOINT_STATES"},
            {MessageType::JOINT_STATES_UPDATE, "JOINT_STATES_UPDATE"},
            {MessageType::GET_POSE, "GET_POSE"},
            {MessageType::POSE_UPDATE, "POSE_UPDATE"},
            {MessageType::LOG_REQUEST, "LOG_REQUEST"},
            {MessageType::LOG_RESPONSE, "LOG_RESPONSE"},
            {MessageType::UNKNOWN, "UNKNOWN"}};

        MessageType stringToMessageType(const std::string &typeStr)
        {
            auto it = stringToEnumMap.find(typeStr);
            if (it != stringToEnumMap.end())
            {
                return it->second;
            }
            return MessageType::UNKNOWN;
        }

        std::string messageTypeToString(MessageType type)
        {
            auto it = enumToStringMap.find(type);
            if (it != enumToStringMap.end())
            {
                return it->second;
            }
            return "UNKNOWN";
        }

        std::optional<ParsedMessage> parseIncomingMessage(const std::string &jsonPayload)
        {
            try
            {
                auto jsonObj = nlohmann::json::parse(jsonPayload);

                // Expect a "type" field
                if (!jsonObj.contains("type") || !jsonObj["type"].is_string())
                {
                    return std::nullopt;
                }

                std::string typeStr = jsonObj["type"].get<std::string>();
                MessageType msgType = stringToMessageType(typeStr);

                // Build the ParsedMessage
                ParsedMessage parsed;
                parsed.messageType = msgType;

                // We can remove 'type' field from data or keep it. Let's remove it to avoid duplication
                jsonObj.erase("type");
                parsed.data = jsonObj;

                return parsed;
            }
            catch (...)
            {
                // If parsing fails or unexpected format
                return std::nullopt;
            }
        }

        std::string serializeMessage(MessageType type, const nlohmann::json &data)
        {
            nlohmann::json outJson;
            outJson["type"] = messageTypeToString(type);

            // Merge provided data fields into outJson
            for (auto it = data.begin(); it != data.end(); ++it)
            {
                outJson[it.key()] = it.value();
            }

            return outJson.dump(); // Convert to a string
        }
    } // namespace Communication
} // namespace RealTimeSystem

