#pragma once

#include <array>
#include <cstdint>
#include "merai/FixedString.h"

namespace motion_control
{
    namespace merai
    {
        // ---------------------------------------------------------------------
        // 1) EtherCAT Drive Config
        // ---------------------------------------------------------------------

        constexpr int MAX_DRIVES = 16;
        constexpr size_t MAX_STR_LEN = 16;

        // PDO Mapping
        struct PdoMappingEntry
        {
            FixedString<MAX_STR_LEN> object_index; // e.g. "0x6040"
            uint8_t subindex = 0;
            uint32_t bit_length = 0;
            FixedString<MAX_STR_LEN> data_type; // e.g. "uint16"
        };

        struct SyncManagerConfig
        {
            int id = 0;
            FixedString<MAX_STR_LEN> type; // "rxpdo", "txpdo"
            bool watchdog_enabled = false;

            static constexpr int MAX_ASSIGNMENTS = 3;
            std::array<FixedString<MAX_STR_LEN>, MAX_ASSIGNMENTS> pdo_assignments{};
            int assignmentCount = 0;

            // For each PDO assignment, store up to 8 mapping entries
            static constexpr int MAX_MAPPING_PER_ASSIGNMENT = 8;

            struct MappingsForOneAssignment
            {
                FixedString<MAX_STR_LEN> assignmentKey; // e.g. "0x1600"
                std::array<PdoMappingEntry, MAX_MAPPING_PER_ASSIGNMENT> entries;
                int entryCount = 0;
            };

            static constexpr int MAX_ASSIGNMENT_OBJECTS = 3;
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
            FixedString<MAX_STR_LEN> object_index; // e.g. "0x0300"
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

            FixedString<MAX_STR_LEN> vendor_name;
            FixedString<MAX_STR_LEN> product_name;
            uint32_t vendor_id = 0;
            uint32_t product_code = 0;

            FixedString<MAX_STR_LEN> type; // e.g. "servo", "io"

            // distributed clock
            DistributedClockConfig distributed_clock;

            // sync managers
            static constexpr int MAX_SYNC_MANAGERS = 4;
            std::array<SyncManagerConfig, MAX_SYNC_MANAGERS> syncManagers;
            int syncManagerCount = 0;

            // sdo
            static constexpr int MAX_SDOS = 8;
            std::array<SdoConfig, MAX_SDOS> sdo_configuration;
            int sdoCount = 0;
        };

        // ---------------------------------------------------------------------
        // 2) Robot Config
        // ---------------------------------------------------------------------

        constexpr int MAX_LINKS = 16;
        constexpr int MAX_JOINTS = 16;

        struct Inertia
        {
            double ixx = 0.0;
            double iyy = 0.0;
            double izz = 0.0;
            double ixy = 0.0;
            double ixz = 0.0;
            double iyz = 0.0;
        };

        struct LinkConfig
        {
            FixedString<MAX_STR_LEN> name;
            double mass = 0.0;
            std::array<double, 3> center_of_mass{{0.0, 0.0, 0.0}};
            Inertia inertia;
        };

        struct JointLimits
        {
            double min = 0.0;
            double max = 0.0;
        };

        struct EncoderResolution
        {
            int counts_per_revolution = 0;
        };

        struct JointParameters
        {
            JointLimits joint_position_limits;
            JointLimits joint_velocity_limits;
            JointLimits joint_acceleration_limits;
            double gear_ratio = 1.0;
            EncoderResolution encoder_resolution;
            int axis_direction = 1;
            bool enable_drive = false;
            int torque_axis_direction = 1;
            double motor_rated_torque = 0.0;
            bool position_limits_active = false;
            bool velocity_limit_active = false;
            bool acceleration_limit_active = false;
            bool torque_limit_active = false;
            double joint_position_offset = 0.0;
        };

        struct Origin
        {
            std::array<double, 3> position{{0.0, 0.0, 0.0}};
            std::array<double, 3> orientation{{0.0, 0.0, 0.0}};
        };

        struct JointConfig
        {
            FixedString<MAX_STR_LEN> name;
            FixedString<MAX_STR_LEN> type;   // e.g. "revolute", "prismatic"
            FixedString<MAX_STR_LEN> parent; // link name
            FixedString<MAX_STR_LEN> child;  // link name

            Origin origin;
            std::array<double, 3> axis{{0.0, 0.0, 1.0}};
            JointParameters parameters;
        };

        struct startupConfig
        {
            long controlLoopNs = 1000000;
            long fieldbusLoopNs = 1000000;
            long logicLoopNs = 1000000;
            bool simulateMode = false;
        };

        // ---------------------------------------------------------------------
        // 3) Aggregated Parameter Server
        // ---------------------------------------------------------------------

        struct ParameterServer
        {
            // A) EtherCAT drive data
            std::array<DriveConfig, MAX_DRIVES> drives;
            int driveCount = 0;

            // B) Robot data
            //    ADDED: robot_name & manipulator_type fields to store the JSON "robot" name & type
            FixedString<MAX_STR_LEN> robot_name;
            FixedString<MAX_STR_LEN> manipulator_type;

            std::array<LinkConfig, MAX_LINKS> links;
            int linkCount = 0;

            std::array<JointConfig, MAX_JOINTS> joints;
            int jointCount = 0;

            // C) startup config
            startupConfig startup;
        };

    } // namespace merai
} // namespace motion_control
