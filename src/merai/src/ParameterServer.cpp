#include "merai/ParameterServer.h"
#include "json.hpp"

#include <fstream>
#include <stdexcept>
#include <iostream>
#include <cstdlib>  // for std::strtoul
#include <cstring>  // for std::memcpy, etc.

namespace hand_control
{
namespace merai
{

// ---------------------------------------------------------------------
// Helper: parseEthercatConfig (unchanged from your existing code)
// ---------------------------------------------------------------------
void parseEthercatConfig(ParameterServer& paramServer,
                         const std::string& ecatConfigFile)
{
    std::ifstream ifs(ecatConfigFile);
    if (!ifs.is_open())
    {
        throw std::runtime_error("Could not open EtherCAT config: " + ecatConfigFile);
    }

    nlohmann::json j;
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

        dc.product_name.set(drv.value("product_name", ""));

        // vendor_id, product_code (hex strings e.g. "0x0000abcd")
        if (drv.contains("vendor_id"))
        {
            std::string vStr = drv["vendor_id"].get<std::string>();
            dc.vendor_id = std::strtoul(vStr.c_str(), nullptr, 16);
        }
        if (drv.contains("product_code"))
        {
            std::string pStr = drv["product_code"].get<std::string>();
            dc.product_code = std::strtoul(pStr.c_str(), nullptr, 16);
        }

        // Type: "servo" or "io"
        dc.type.set(drv.value("type", "unknown"));

        // distributed_clock
        if (drv.contains("distributed_clock"))
        {
            auto clk = drv["distributed_clock"];
            dc.distributed_clock.object_index.set(clk.value("object_index", "0x0000"));
            dc.distributed_clock.cycle_time_ns = clk.value("cycle_time_ns", 0);
            dc.distributed_clock.sync0         = clk.value("sync0", 0);
            dc.distributed_clock.sync1         = clk.value("sync1", 0);
            dc.distributed_clock.offset_ns     = clk.value("offset_ns", 0);
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
                    smc.type.set(sm.value("type", ""));  // "rxpdo" or "txpdo"
                    smc.watchdog_enabled = sm.value("watchdog_enabled", false);

                    // pdo_assignments
                    if (sm.contains("pdo_assignments"))
                    {
                        auto paArray = sm["pdo_assignments"];
                        for (auto& pa : paArray)
                        {
                            if (smc.assignmentCount >= smc.MAX_ASSIGNMENTS)
                            {
                                std::cerr << "Warning: Reached MAX_ASSIGNMENTS.\n";
                                break;
                            }
                            smc.pdo_assignments[smc.assignmentCount].set(pa.get<std::string>());
                            smc.assignmentCount++;
                        }
                    }

                    // pdo_mappings -> for each assignment key -> array of mapping entries
                    if (sm.contains("pdo_mappings"))
                    {
                        auto pmaps = sm["pdo_mappings"];
                        for (auto it = pmaps.begin(); it != pmaps.end(); ++it)
                        {
                            if (smc.mappingGroupCount >= smc.MAX_ASSIGNMENT_OBJECTS)
                            {
                                std::cerr << "Warning: Reached MAX_ASSIGNMENT_OBJECTS.\n";
                                break;
                            }

                            auto& mg = smc.mappingGroups[smc.mappingGroupCount];
                            smc.mappingGroupCount++;

                            mg.assignmentKey.set(it.key());
                            auto entryArr = it.value(); // array of objects

                            for (auto& en : entryArr)
                            {
                                if (mg.entryCount >= smc.MAX_MAPPING_PER_ASSIGNMENT)
                                {
                                    std::cerr << "Warning: Reached MAX_MAPPING_PER_ASSIGNMENT.\n";
                                    break;
                                }
                                PdoMappingEntry& pme = mg.entries[mg.entryCount];
                                mg.entryCount++;

                                pme.object_index.set(en.value("object_index", ""));
                                pme.subindex   = static_cast<uint8_t>(en.value("subindex", 0));
                                pme.bit_length = en.value("bit_length", 0);
                                pme.data_type.set(en.value("data_type", ""));
                                // parse "description" if present
                                pme.description.set(en.value("description", ""));
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

                sc.object_index.set(scJson.value("object_index", ""));
                sc.subindex  = static_cast<uint8_t>(scJson.value("subindex", 0));
                sc.value     = scJson.value("value", 0);
                sc.data_type.set(scJson.value("data_type", ""));
            }
        }
    }
}

// ---------------------------------------------------------------------
// Helper: parseRobotParameters
// ---------------------------------------------------------------------
void parseRobotParameters(ParameterServer& paramServer,
                          const std::string& robotFile)
{
    std::ifstream ifs(robotFile);
    if (!ifs.is_open())
    {
        throw std::runtime_error("Could not open robot params: " + robotFile);
    }

    nlohmann::json j;
    ifs >> j;

    if (!j.contains("robot"))
    {
        throw std::runtime_error("Robot JSON missing 'robot' object.");
    }
    auto robotObj = j["robot"];

    // 1) Robot name & manipulator_type
    paramServer.robot_name.set(robotObj.value("name", ""));
    paramServer.manipulator_type.set(robotObj.value("manipulator_type", ""));

    // 2) links
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

            lc.name.set(lk.value("name", ""));
            lc.mass = lk.value("mass", 0.0);

            // com: array of 3 => "com"
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

            // inertia: array of 6 => [ixx, iyy, izz, ixy, ixz, iyz]
            if (lk.contains("inertia"))
            {
                auto inArr = lk["inertia"];
                if (inArr.is_array() && inArr.size() == 6)
                {
                    for (size_t i = 0; i < 6; i++)
                    {
                        lc.inertia[i] = inArr[i].get<double>();
                    }
                }
            }
        }
    }

    // 3) joints
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

            jc.name.set(jt.value("name", ""));
            jc.type.set(jt.value("type", ""));
            jc.parent.set(jt.value("parent", ""));
            jc.child.set(jt.value("child", ""));

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

            // "limits": { "position": {min, max}, "velocity": {...}, "acceleration": {...} }
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
                if (lm.contains("acceleration"))
                {
                    auto a = lm["acceleration"];
                    jc.limits.acceleration.min = a.value("min", 0.0);
                    jc.limits.acceleration.max = a.value("max", 0.0);
                }
            }

            // gear_ratio, encoder_counts, axis_direction, torque_axis_direction, rated_torque, enable_drive
            jc.gear_ratio          = jt.value("gear_ratio", 1.0);
            jc.encoder_counts      = jt.value("encoder_counts", 0);
            jc.axis_direction      = jt.value("axis_direction", 1);
            jc.torque_axis_direction = jt.value("torque_axis_direction", 1);
            jc.rated_torque        = jt.value("rated_torque", 0.0);
            jc.enable_drive        = jt.value("enable_drive", false);

            // "limits_active": { "position": ..., "velocity": ..., "acceleration": ..., "torque": ... }
            if (jt.contains("limits_active"))
            {
                auto la = jt["limits_active"];
                jc.limit_position_active     = la.value("position", false);
                jc.limit_velocity_active     = la.value("velocity", false);
                jc.limit_acceleration_active = la.value("acceleration", false);
                jc.limit_torque_active       = la.value("torque", false);
            }

            // position_offset
            jc.position_offset = jt.value("position_offset", 0.0);
        }
    }
}

// ---------------------------------------------------------------------
// Helper: parseStartupParameters (unchanged from your existing code)
// ---------------------------------------------------------------------
void parseStartupParameters(ParameterServer& paramServer,
                            const std::string& startupFile)
{
    std::ifstream ifs(startupFile);
    if (!ifs.is_open())
    {
        throw std::runtime_error("Could not open startup config file: " + startupFile);
    }

    nlohmann::json j;
    ifs >> j;

    paramServer.startup.fieldbusLoopNs = j.value("fieldbus_Loop_ns", 1000000L);
    paramServer.startup.controlLoopNs  = j.value("control_loop_ns",  1000000L);
    paramServer.startup.logicLoopNs    = j.value("logic_loop_ns",    10000000L);
    paramServer.startup.simulateMode   = j.value("simulate_mode",    false);
}

// ---------------------------------------------------------------------
// parseParameterServer: a convenience function calling all parsers
// ---------------------------------------------------------------------
ParameterServer parseParameterServer(const std::string& ecatConfigFile,
                                     const std::string& robotParamFile,
                                     const std::string& startupFile)
{
    ParameterServer paramServer; // zero-initialized

    // 1) EtherCAT config
    parseEthercatConfig(paramServer, ecatConfigFile);
    // 2) Robot config
    parseRobotParameters(paramServer, robotParamFile);
    // 3) Startup config
    parseStartupParameters(paramServer, startupFile);

    return paramServer;
}

} // namespace merai
} // namespace hand_control
