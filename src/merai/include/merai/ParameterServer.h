#pragma once

#include <array>
#include <cstdint>
#include "merai/FixedString.h"

namespace hand_control
{
    namespace merai
    {
        // -------------------------------------------------------------------------
        // 1) EtherCAT Drive Config (unchanged)
        // -------------------------------------------------------------------------

        constexpr int MAX_DRIVES = 16;
        constexpr size_t MAX_STR_LEN = 16;

        // PDO Mapping
        struct PdoMappingEntry
        {
            FixedString<MAX_STR_LEN> object_index;
            uint8_t subindex = 0;
            uint32_t bit_length = 0;
            FixedString<MAX_STR_LEN> data_type;
            FixedString<MAX_STR_LEN> description;
        };

        struct SyncManagerConfig
        {
            int id = 0;
            FixedString<MAX_STR_LEN> type; // e.g. "rxpdo", "txpdo"
            bool watchdog_enabled = false;

            static constexpr int MAX_ASSIGNMENTS = 4;
            std::array<FixedString<MAX_STR_LEN>, MAX_ASSIGNMENTS> pdo_assignments{};
            int assignmentCount = 0;

            static constexpr int MAX_MAPPING_PER_ASSIGNMENT = 8;

            struct MappingsForOneAssignment
            {
                FixedString<MAX_STR_LEN> assignmentKey;
                std::array<PdoMappingEntry, MAX_MAPPING_PER_ASSIGNMENT> entries;
                int entryCount = 0;
            };

            static constexpr int MAX_ASSIGNMENT_OBJECTS = 4;
            std::array<MappingsForOneAssignment, MAX_ASSIGNMENT_OBJECTS> mappingGroups{};
            int mappingGroupCount = 0;
        };

        struct SdoConfig
        {
            FixedString<MAX_STR_LEN> object_index;
            uint8_t subindex = 0;
            int value = 0;
            FixedString<MAX_STR_LEN> data_type;
        };

        struct DistributedClockConfig
        {
            FixedString<MAX_STR_LEN> object_index;
            uint32_t cycle_time_ns = 0;
            uint32_t sync0 = 0;
            uint32_t sync1 = 0;
            uint32_t offset_ns = 0;
        };

        struct DriveConfig
        {
            int id = 0;
            uint16_t alias = 0;
            uint16_t position = 0;

            FixedString<MAX_STR_LEN> product_name;
            uint32_t vendor_id = 0;
            uint32_t product_code = 0;

            FixedString<MAX_STR_LEN> type; // e.g. "servo", "io"

            DistributedClockConfig distributed_clock;

            static constexpr int MAX_SYNC_MANAGERS = 4;
            std::array<SyncManagerConfig, MAX_SYNC_MANAGERS> syncManagers;
            int syncManagerCount = 0;

            static constexpr int MAX_SDOS = 8;
            std::array<SdoConfig, MAX_SDOS> sdo_configuration;
            int sdoCount = 0;
        };

        // -------------------------------------------------------------------------
        // 2) Robot Config (adapted to match your "robot_parameter.json")
        // -------------------------------------------------------------------------

        constexpr int MAX_LINKS = 16;
        constexpr int MAX_JOINTS = 16;

        // Each link now uses "com" (array of 3) and "inertia" (array of 6).
        struct LinkConfig
        {
            FixedString<MAX_STR_LEN> name;
            double mass = 0.0;
            // JSON field: "com": [...]
            std::array<double, 3> com{{0.0, 0.0, 0.0}};
            // JSON field: "inertia": e.g. [0.1, 0.1, 0.1, 0.0, 0.0, 0.0]
            // ixx, iyy, izz, ixy, ixz, iyz
            std::array<double, 6> inertia{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        };

        // For joint "limits" object with subfields "position", "velocity", "acceleration"
        struct JointLimitRange
        {
            double min = 0.0;
            double max = 0.0;
        };

        struct JointLimits
        {
            JointLimitRange position;
            JointLimitRange velocity;
            JointLimitRange acceleration;
        };

        struct JointConfig
        {
            FixedString<MAX_STR_LEN> name;
            FixedString<MAX_STR_LEN> type;
            FixedString<MAX_STR_LEN> parent;
            FixedString<MAX_STR_LEN> child;

            // Instead of a single 'Origin', your JSON has "origin_pos" and "origin_orient"
            //   e.g. "origin_pos": [0.0, 0.0, 0.4], "origin_orient": [...]
            std::array<double, 3> origin_pos{{0.0, 0.0, 0.0}};
            std::array<double, 3> origin_orient{{0.0, 0.0, 0.0}};

            // "axis": [0.0, 0.0, 1.0]
            std::array<double, 3> axis{{0.0, 0.0, 1.0}};

            // "limits": { "position": {...}, "velocity": {...}, "acceleration": {...} }
            JointLimits limits;

            // Additional fields from your JSON (gear_ratio, encoder_counts, etc.)
            double gear_ratio = 1.0;
            int encoder_counts = 0;
            int axis_direction = 1;
            int torque_axis_direction = 1;
            double rated_torque = 0.0;
            bool enable_drive = false;
            double rated_current = 0.0;
            double torque_constant = 0.0;

            // "limits_active": { "position": true, "velocity": true, "acceleration": true, "torque": true }
            // We'll store them in a struct or booleans:
            bool limit_position_active = false;
            bool limit_velocity_active = false;
            bool limit_acceleration_active = false;
            bool limit_torque_active = false;

            // "position_offset": 0.0
            double position_offset = 0.0;
        };

        struct startupConfig
        {
            long controlLoopNs = 1000000;
            long fieldbusLoopNs = 1000000;
            long logicLoopNs = 1000000;
            bool simulateMode = false;
        };

        // -------------------------------------------------------------------------
        // 3) Aggregated Parameter Server
        // -------------------------------------------------------------------------

        struct ParameterServer
        {
            // A) EtherCAT drive data
            std::array<DriveConfig, MAX_DRIVES> drives;
            int driveCount = 0;

            // B) Robot data
            FixedString<MAX_STR_LEN> robot_name;
            FixedString<MAX_STR_LEN> manipulator_type;

            std::array<LinkConfig, MAX_LINKS> links;
            int linkCount = 0;

            std::array<JointConfig, MAX_JOINTS> joints;
            int jointCount = 0;

            // C) startup config
            startupConfig startup;
        };

        /**
         * @brief Parse function that loads the EtherCAT, Robot, and startup config
         *        from JSON files, populating the ParameterServer struct.
         *
         * @param ecatConfigFile   Path to EtherCAT JSON (like "ethercat_config.json")
         * @param robotParamFile   Path to robot param JSON
         * @param startupFile      Path to startup config JSON
         * @return ParameterServer fully populated with data
         */
        ParameterServer parseParameterServer(const std::string &ecatConfigFile,
                                             const std::string &robotParamFile,
                                             const std::string &startupFile);

    } // namespace merai
} // namespace hand_control
