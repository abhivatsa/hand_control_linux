#ifndef MESSAGE_PROTOCOL_H
#define MESSAGE_PROTOCOL_H

#include <string>
#include <optional>
#include "external_libraries/json.hpp" // Assuming you're using this for JSON (adjust if using a different library)

namespace RealTimeSystem
{
    namespace Communication
    {
        // List of all recognized message types
        enum class MessageType
        {
            UNKNOWN = 0,

            // --- Motion Commands ---
            START_MOTION,
            STOP_MOTION,
            PAUSE_MOTION,
            RESUME_MOTION,
            E_STOP,
            JOG_COMMAND,

            // --- Configuration ---
            SET_SPEED,
            SET_ACCEL_LIMIT,
            SET_TOOL_DATA,
            ENABLE_COLLISION_AVOIDANCE,
            SET_HOME_POSITION,

            // --- Status & Safety ---
            GET_STATUS,
            SYSTEM_STATUS_UPDATE,
            SAFETY_STATE_UPDATE,
            ERROR_FAULT,
            WARNING_ALARM,

            // --- Monitoring & Logging ---
            GET_JOINT_STATES,
            JOINT_STATES_UPDATE,
            GET_POSE,
            POSE_UPDATE,
            LOG_REQUEST,
            LOG_RESPONSE,

            // ... add more as needed ...
        };

        /**
         * @brief Convert a string (e.g. "START_MOTION") to a MessageType enum.
         *
         * @param typeStr The string indicating the message type.
         * @return MessageType The matching enum value, or MessageType::UNKNOWN if unrecognized.
         */
        MessageType stringToMessageType(const std::string &typeStr);

        /**
         * @brief Convert a MessageType enum back to string (for outgoing messages).
         *
         * @param type The message type enum.
         * @return std::string The corresponding string (e.g. "START_MOTION").
         */
        std::string messageTypeToString(MessageType type);

        /**
         * @brief A container for parsed messages.
         *
         * For simplicity, we store the entire JSON payload in `data`.
         * The `type` field is already extracted into `messageType`.
         */
        struct ParsedMessage
        {
            MessageType messageType = MessageType::UNKNOWN;
            nlohmann::json data; // The remainder of the JSON payload
        };

        /**
         * @brief Parse an incoming JSON string into a ParsedMessage structure.
         *
         * @param jsonPayload The raw JSON string.
         * @return std::optional<ParsedMessage> If parsing fails, returns std::nullopt.
         */
        std::optional<ParsedMessage> parseIncomingMessage(const std::string &jsonPayload);

        /**
         * @brief Serialize a message (type + data) back into JSON string for sending.
         *
         * @param type The message type enum.
         * @param data The JSON payload (fields, etc.).
         * @return std::string The JSON-encoded string, ready to send over WebSocket.
         */
        std::string serializeMessage(MessageType type, const nlohmann::json &data);
    } // namespace communication
}// namespace RealTimeSystem

#endif // MESSAGE_PROTOCOL_H