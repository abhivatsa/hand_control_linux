#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"  // for MAX_SERVO_DRIVES
#include "json.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>   // std::strtoul
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace merai
{

    using json = nlohmann::json;

    // ------------------------------------------------------------
    // Small helpers
    // ------------------------------------------------------------

    inline std::string toLowerCopy(const std::string& s)
    {
        std::string out = s;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c)
                       { return static_cast<char>(std::tolower(c)); });
        return out;
    }

    inline std::uint32_t parseHexToUint32(const std::string& s,
                                          std::uint32_t defaultValue = 0)
    {
        if (s.empty())
        {
            return defaultValue;
        }
        return static_cast<std::uint32_t>(std::strtoul(s.c_str(), nullptr, 0));
    }

    inline std::uint16_t parseHexToUint16(const std::string& s,
                                          std::uint16_t defaultValue = 0)
    {
        return static_cast<std::uint16_t>(parseHexToUint32(s, defaultValue));
    }

    inline DriveType parseDriveType(const std::string& typeStr)
    {
        const auto t = toLowerCopy(typeStr);
        if (t == "servo")      return DriveType::Servo;
        if (t == "io" || t == "i/o") return DriveType::Io;
        return DriveType::Unknown;
    }

    inline SyncType parseSyncType(const std::string& typeStr)
    {
        const auto t = toLowerCopy(typeStr);
        if (t == "rxpdo") return SyncType::RxPdo;
        if (t == "txpdo") return SyncType::TxPdo;
        return SyncType::Unknown;
    }

    inline PdoDataType parsePdoDataType(const std::string& dtStr)
    {
        const auto t = toLowerCopy(dtStr);
        if (t == "int8")   return PdoDataType::Int8;
        if (t == "int16")  return PdoDataType::Int16;
        if (t == "int32")  return PdoDataType::Int32;
        if (t == "uint16") return PdoDataType::UInt16;
        if (t == "uint32") return PdoDataType::UInt32;
        return PdoDataType::Unknown;
    }

    // ---------------------------------------------------------------------
    // EtherCAT config parsing
    // ---------------------------------------------------------------------

    void parseEthercatConfig(ParameterServer& paramServer,
                             const std::string& ecatConfigFile)
    {
        std::ifstream ifs(ecatConfigFile);
        if (!ifs.is_open())
        {
            throw std::runtime_error("Could not open EtherCAT config: " + ecatConfigFile);
        }

        json j;
        ifs >> j;

        if (!j.contains("drives"))
        {
            throw std::runtime_error("EtherCAT config missing 'drives' array.");
        }
        auto drivesArray = j["drives"];

        for (auto& drv : drivesArray)
        {
            if (paramServer.driveCount >= MAX_DRIVES)
            {
                std::cerr << "Warning: Reached MAX_DRIVES. Additional drives ignored.\n";
                break;
            }

            DriveConfig& dc = paramServer.drives[paramServer.driveCount];
            paramServer.driveCount++;

            // Basic fields
            dc.id       = drv.value("id", 0);
            dc.alias    = drv.value("alias", 0);
            dc.position = drv.value("position", 0);

            // vendor_id, product_code (hex strings e.g. "0x0000abcd")
            if (drv.contains("vendor_id"))
            {
                std::string vStr = drv["vendor_id"].get<std::string>();
                dc.vendor_id     = parseHexToUint32(vStr, 0);
            }
            if (drv.contains("product_code"))
            {
                std::string pStr = drv["product_code"].get<std::string>();
                dc.product_code  = parseHexToUint32(pStr, 0);
            }

            // Type: "servo" or "io"
            dc.type = parseDriveType(drv.value("type", ""));

            // distributed_clock
            if (drv.contains("distributed_clock"))
            {
                auto clk = drv["distributed_clock"];
                dc.distributed_clock.object_index  =
                    parseHexToUint16(clk.value("object_index", "0x0000"), 0);
                dc.distributed_clock.cycle_time_ns = clk.value("cycle_time_ns", 0u);
                dc.distributed_clock.sync0         = clk.value("sync0", 0u);
                dc.distributed_clock.sync1         = clk.value("sync1", 0u);
                dc.distributed_clock.offset_ns     = clk.value("offset_ns", 0u);
            }

            // pdo_configuration -> sync_managers
            if (drv.contains("pdo_configuration"))
            {
                auto pdo = drv["pdo_configuration"];
                if (pdo.contains("sync_managers"))
                {
                    auto smArray = pdo["sync_managers"];
                    for (auto& sm : smArray)
                    {
                        if (dc.syncManagerCount >= DriveConfig::MAX_SYNC_MANAGERS)
                        {
                            std::cerr << "Warning: Reached MAX_SYNC_MANAGERS, ignoring extra.\n";
                            break;
                        }

                        SyncManagerConfig& smc = dc.syncManagers[dc.syncManagerCount];
                        dc.syncManagerCount++;

                        smc.id               = sm.value("id", 0);
                        smc.type             = parseSyncType(sm.value("type", ""));
                        smc.watchdog_enabled = sm.value("watchdog_enabled", false);

                        // pdo_assignments
                        if (sm.contains("pdo_assignments"))
                        {
                            auto paArray = sm["pdo_assignments"];
                            for (auto& pa : paArray)
                            {
                                if (smc.assignmentCount >= SyncManagerConfig::MAX_ASSIGNMENTS)
                                {
                                    std::cerr << "Warning: Reached MAX_ASSIGNMENTS.\n";
                                    break;
                                }
                                smc.pdo_assignments[smc.assignmentCount] =
                                    parseHexToUint16(pa.get<std::string>(), 0);
                                smc.assignmentCount++;
                            }
                        }

                        // pdo_mappings
                        if (sm.contains("pdo_mappings"))
                        {
                            auto pmaps = sm["pdo_mappings"];
                            for (auto it = pmaps.begin(); it != pmaps.end(); ++it)
                            {
                                if (smc.mappingGroupCount >= SyncManagerConfig::MAX_ASSIGNMENT_OBJECTS)
                                {
                                    std::cerr << "Warning: Reached MAX_ASSIGNMENT_OBJECTS.\n";
                                    break;
                                }

                                auto& mg = smc.mappingGroups[smc.mappingGroupCount];
                                smc.mappingGroupCount++;

                                mg.assignmentKey = parseHexToUint16(it.key(), 0);
                                auto entryArr    = it.value(); // array

                                for (auto& en : entryArr)
                                {
                                    if (mg.entryCount >= SyncManagerConfig::MAX_MAPPING_PER_ASSIGNMENT)
                                    {
                                        std::cerr << "Warning: Reached MAX_MAPPING_PER_ASSIGNMENT.\n";
                                        break;
                                    }

                                    PdoMappingEntry& pme = mg.entries[mg.entryCount];
                                    mg.entryCount++;

                                    pme.object_index =
                                        parseHexToUint16(en.value("object_index", "0x0000"), 0);
                                    pme.subindex   =
                                        static_cast<std::uint8_t>(en.value("subindex", 0));
                                    pme.bit_length = en.value("bit_length", 0u);
                                    pme.data_type  =
                                        parsePdoDataType(en.value("data_type", ""));
                                }
                            }
                        }
                    }
                }
            }

            // sdo_configuration
            if (drv.contains("sdo_configuration"))
            {
                auto sdoArray = drv["sdo_configuration"];
                for (auto& scJson : sdoArray)
                {
                    if (dc.sdoCount >= DriveConfig::MAX_SDOS)
                    {
                        std::cerr << "Warning: Reached MAX_SDOS, ignoring extra.\n";
                        break;
                    }

                    SdoConfig& sc = dc.sdo_configuration[dc.sdoCount];
                    dc.sdoCount++;

                    sc.object_index =
                        parseHexToUint16(scJson.value("object_index", "0x0000"), 0);
                    sc.subindex   =
                        static_cast<std::uint8_t>(scJson.value("subindex", 0));
                    sc.value      = scJson.value("value", 0);
                    sc.data_type  =
                        parsePdoDataType(scJson.value("data_type", ""));
                }
            }
        }
    }

    // ---------------------------------------------------------------------
    // Robot parameters parsing
    // ---------------------------------------------------------------------

    void parseRobotParameters(ParameterServer& paramServer,
                              const std::string& robotFile)
    {
        std::ifstream ifs(robotFile);
        if (!ifs.is_open())
        {
            throw std::runtime_error("Could not open robot params: " + robotFile);
        }

        json j;
        ifs >> j;

        if (!j.contains("robot"))
        {
            throw std::runtime_error("Robot JSON missing 'robot' object.");
        }
        auto robotObj = j["robot"];

        // gravity
        if (robotObj.contains("gravity"))
        {
            auto g = robotObj["gravity"];
            if (g.is_array() && g.size() == 3)
            {
                paramServer.gravity[0] = g[0].get<double>();
                paramServer.gravity[1] = g[1].get<double>();
                paramServer.gravity[2] = g[2].get<double>();
            }
        }

        // links
        if (robotObj.contains("links"))
        {
            auto linkArr = robotObj["links"];
            for (auto& lk : linkArr)
            {
                if (paramServer.linkCount >= MAX_LINKS)
                {
                    std::cerr << "Warning: Reached MAX_LINKS, ignoring extra links.\n";
                    break;
                }

                LinkConfig& lc = paramServer.links[paramServer.linkCount];
                paramServer.linkCount++;

                lc.mass = lk.value("mass", 0.0);

                if (lk.contains("com"))
                {
                    auto comArr = lk["com"];
                    if (comArr.is_array() && comArr.size() == 3)
                    {
                        lc.com[0] = comArr[0].get<double>();
                        lc.com[1] = comArr[1].get<double>();
                        lc.com[2] = comArr[2].get<double>();
                    }
                }

                if (lk.contains("inertia"))
                {
                    auto inArr = lk["inertia"];
                    if (inArr.is_array() && inArr.size() == 6)
                    {
                        for (std::size_t i = 0; i < 6; ++i)
                        {
                            lc.inertia[i] = inArr[i].get<double>();
                        }
                    }
                }
            }
        }

        // joints
        if (robotObj.contains("joints"))
        {
            auto jointArr = robotObj["joints"];
            for (auto& jt : jointArr)
            {
                if (paramServer.jointCount >= MAX_JOINTS)
                {
                    std::cerr << "Warning: Reached MAX_JOINTS, ignoring extra joints.\n";
                    break;
                }

                JointConfig& jc = paramServer.joints[paramServer.jointCount];
                paramServer.jointCount++;

                // origin_pos
                if (jt.contains("origin_pos"))
                {
                    auto pos = jt["origin_pos"];
                    if (pos.is_array() && pos.size() == 3)
                    {
                        jc.origin_pos[0] = pos[0].get<double>();
                        jc.origin_pos[1] = pos[1].get<double>();
                        jc.origin_pos[2] = pos[2].get<double>();
                    }
                }

                // origin_orient
                if (jt.contains("origin_orient"))
                {
                    auto ori = jt["origin_orient"];
                    if (ori.is_array() && ori.size() == 3)
                    {
                        jc.origin_orient[0] = ori[0].get<double>();
                        jc.origin_orient[1] = ori[1].get<double>();
                        jc.origin_orient[2] = ori[2].get<double>();
                    }
                }

                // axis
                if (jt.contains("axis"))
                {
                    auto ax = jt["axis"];
                    if (ax.is_array() && ax.size() == 3)
                    {
                        jc.axis[0] = ax[0].get<double>();
                        jc.axis[1] = ax[1].get<double>();
                        jc.axis[2] = ax[2].get<double>();
                    }
                }

                // limits
                if (jt.contains("limits"))
                {
                    auto lm = jt["limits"];

                    if (lm.contains("position"))
                    {
                        auto p = lm["position"];
                        jc.limits.position.min = p.value("min", 0.0);
                        jc.limits.position.max = p.value("max", 0.0);
                    }
                    if (lm.contains("velocity"))
                    {
                        auto v = lm["velocity"];
                        jc.limits.velocity.min = v.value("min", 0.0);
                        jc.limits.velocity.max = v.value("max", 0.0);
                    }
                    if (lm.contains("torque"))
                    {
                        auto tq = lm["torque"];
                        jc.limits.torque.min = tq.value("min", 0.0);
                        jc.limits.torque.max = tq.value("max", 0.0);
                    }
                }

                // drive parameters: may be nested under "drive"
                auto get_drive_double = [&](const char* key, double def) -> double
                {
                    if (jt.contains("drive"))
                    {
                        auto d = jt["drive"];
                        if (d.contains(key))
                        {
                            return d.value(key, def);
                        }
                    }
                    return jt.value(key, def);
                };
                auto get_drive_int = [&](const char* key, int def) -> int
                {
                    if (jt.contains("drive"))
                    {
                        auto d = jt["drive"];
                        if (d.contains(key))
                        {
                            return d.value(key, def);
                        }
                    }
                    return jt.value(key, def);
                };
                auto get_drive_bool = [&](const char* key, bool def) -> bool
                {
                    if (jt.contains("drive"))
                    {
                        auto d = jt["drive"];
                        if (d.contains(key))
                        {
                            return d.value(key, def);
                        }
                    }
                    return jt.value(key, def);
                };

                jc.drive.gear_ratio          = get_drive_double("gear_ratio", 1.0);
                jc.drive.encoder_counts      = get_drive_int("encoder_counts", 0);
                jc.drive.axis_direction      = get_drive_int("axis_direction", 1);
                jc.drive.torque_axis_direction = get_drive_int("torque_axis_direction", 1);
                jc.drive.rated_torque        = get_drive_double("rated_torque", 0.0);
                jc.drive.enable_drive        = get_drive_bool("enable_drive", false);
                jc.drive.torque_constant     = get_drive_double("torque_constant", 0.0);
                jc.drive.rated_current       = get_drive_double("rated_current", 0.0);

                jc.position_offset           = jt.value("position_offset", 0.0);
            }
        }
    }

    // ---------------------------------------------------------------------
    // parseParameterServer
    // ---------------------------------------------------------------------

    ParameterServer parseParameterServer(const std::string& ecatConfigFile,
                                         const std::string& robotParamFile)
    {
        ParameterServer paramServer{}; // default-initialized
        paramServer.magic   = PARAM_SERVER_MAGIC;
        paramServer.version = PARAM_SERVER_VERSION;

        // 1) EtherCAT config
        parseEthercatConfig(paramServer, ecatConfigFile);

        // 2) Robot config
        parseRobotParameters(paramServer, robotParamFile);

        // Clamp counts to RT + array limits.
        const int rtMaxDrives =
            std::min<int>(static_cast<int>(MAX_SERVO_DRIVES), MAX_DRIVES);
        if (paramServer.driveCount > rtMaxDrives)
        {
            std::cerr << "Warning: driveCount (" << paramServer.driveCount
                      << ") exceeds RT limit (" << rtMaxDrives << "); clamping.\n";
            paramServer.driveCount = rtMaxDrives;
        }

        const int rtMaxJoints =
            std::min<int>(static_cast<int>(MAX_SERVO_DRIVES), MAX_JOINTS);
        if (paramServer.jointCount > rtMaxJoints)
        {
            std::cerr << "Warning: jointCount (" << paramServer.jointCount
                      << ") exceeds RT limit (" << rtMaxJoints << "); clamping.\n";
            paramServer.jointCount = rtMaxJoints;
        }

        return paramServer;
    }

} // namespace merai
