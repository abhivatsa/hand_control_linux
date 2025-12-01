#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <type_traits>

#include "merai/Enums.h"

namespace merai
{
    // -------------------------------------------------------------------------
    // 1) EtherCAT Drive Config
    // -------------------------------------------------------------------------

    constexpr int          MAX_DRIVES          = 16;
    constexpr std::uint32_t PARAM_SERVER_MAGIC   = 0x50415241; // 'PARA'
    constexpr std::uint32_t PARAM_SERVER_VERSION = 1;

    struct PdoMappingEntry
    {
        std::uint16_t object_index = 0;
        std::uint8_t  subindex     = 0;
        std::uint32_t bit_length   = 0;
        PdoDataType   data_type    = PdoDataType::Unknown;
    };

    struct SyncManagerConfig
    {
        int       id               = 0;
        SyncType  type             = SyncType::Unknown;
        bool      watchdog_enabled = false;

        static constexpr int MAX_ASSIGNMENTS = 4;
        std::array<std::uint16_t, MAX_ASSIGNMENTS> pdo_assignments{};
        int assignmentCount = 0;

        static constexpr int MAX_MAPPING_PER_ASSIGNMENT = 8;

        struct MappingsForOneAssignment
        {
            std::uint16_t assignmentKey = 0;
            std::array<PdoMappingEntry, MAX_MAPPING_PER_ASSIGNMENT> entries{};
            int           entryCount = 0;
        };

        static constexpr int MAX_ASSIGNMENT_OBJECTS = 4;
        std::array<MappingsForOneAssignment, MAX_ASSIGNMENT_OBJECTS> mappingGroups{};
        int mappingGroupCount = 0;
    };

    struct SdoConfig
    {
        std::uint16_t object_index = 0;
        std::uint8_t  subindex     = 0;
        int           value        = 0;
        PdoDataType   data_type    = PdoDataType::Unknown;
    };

    struct DistributedClockConfig
    {
        std::uint16_t object_index   = 0;
        std::uint32_t cycle_time_ns  = 0;
        std::uint32_t sync0          = 0;
        std::uint32_t sync1          = 0;
        std::uint32_t offset_ns      = 0;
    };

    struct DriveConfig
    {
        int           id          = 0;
        std::uint16_t alias       = 0;
        std::uint16_t position    = 0;

        std::uint32_t vendor_id   = 0;
        std::uint32_t product_code= 0;

        DriveType     type        = DriveType::Unknown; // servo / io

        DistributedClockConfig distributed_clock{};

        static constexpr int MAX_SYNC_MANAGERS = 4;
        std::array<SyncManagerConfig, MAX_SYNC_MANAGERS> syncManagers{};
        int syncManagerCount = 0;

        static constexpr int MAX_SDOS = 8;
        std::array<SdoConfig, MAX_SDOS> sdo_configuration{};
        int sdoCount = 0;
    };

    // -------------------------------------------------------------------------
    // 2) Robot Config
    // -------------------------------------------------------------------------

    constexpr int MAX_LINKS  = 16;
    constexpr int MAX_JOINTS = 16;

    struct LinkConfig
    {
        double mass = 0.0;
        std::array<double, 3> com{{0.0, 0.0, 0.0}};
        // inertia: [ixx, iyy, izz, ixy, ixz, iyz]
        std::array<double, 6> inertia{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    };

    struct JointLimitRange
    {
        double min = 0.0;
        double max = 0.0;
    };

    struct JointLimits
    {
        JointLimitRange position;
        JointLimitRange velocity;
        JointLimitRange torque;
    };

    struct JointConfig
    {
        int parent_index = -1;
        int child_index  = -1;

        std::array<double, 3> origin_pos{{0.0, 0.0, 0.0}};
        std::array<double, 3> origin_orient{{0.0, 0.0, 0.0}};
        std::array<double, 3> axis{{0.0, 0.0, 1.0}};

        JointLimits limits{};

        struct DriveParameters
        {
            double gear_ratio          = 1.0;
            int    encoder_counts      = 1024;
            int    axis_direction      = 1;
            int    torque_axis_direction = 1;
            double rated_torque        = 0.0;
            bool   enable_drive        = false;
            double rated_current       = 0.0;
            double torque_constant     = 0.0;
        };

        DriveParameters drive{};

        double position_offset = 0.0;
    };

    // -------------------------------------------------------------------------
    // 3) Aggregated Parameter Server
    // -------------------------------------------------------------------------

    struct ParameterServer
    {
        std::uint32_t magic   = PARAM_SERVER_MAGIC;
        std::uint32_t version = PARAM_SERVER_VERSION;

        // Gravity vector (m/s^2)
        std::array<double, 3> gravity{{0.0, 0.0, -9.81}};

        // EtherCAT drives
        std::array<DriveConfig, MAX_DRIVES> drives{};
        int driveCount = 0;

        // Robot links
        std::array<LinkConfig, MAX_LINKS> links{};
        int linkCount = 0;

        // Robot joints
        std::array<JointConfig, MAX_JOINTS> joints{};
        int jointCount = 0;
    };

    ParameterServer parseParameterServer(const std::string& ecatConfigFile,
                                         const std::string& robotParamFile);

    static_assert(std::is_trivially_copyable<ParameterServer>::value,
                  "ParameterServer must be trivially copyable for SHM use");

} // namespace merai
