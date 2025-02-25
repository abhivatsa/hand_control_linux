#include "merai/ParameterServer.h"
#include "json.hpp"

#include <fstream>
#include <stdexcept>
#include <iostream>
#include <cstdlib>  // for std::strtoul
#include <cstring>  // for std::memcpy, etc.

namespace motion_control
{
    namespace merai
    {
        // ---------------------------------------------------------------------
        // Helper: parseEthercatConfig
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

                dc.vendor_name.set(drv.value("vendor_name", ""));
                dc.product_name.set(drv.value("product_name", ""));

                // vendor_id, product_code (often hex strings like "0x000022d2")
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

                    if (lk.contains("center_of_mass"))
                    {
                        auto comArr = lk["center_of_mass"];
                        if (comArr.is_array() && comArr.size() == 3)
                        {
                            lc.center_of_mass[0] = comArr[0].get<double>();
                            lc.center_of_mass[1] = comArr[1].get<double>();
                            lc.center_of_mass[2] = comArr[2].get<double>();
                        }
                    }

                    if (lk.contains("inertia"))
                    {
                        // If "inertia" is an array of 6? or an object with keys?
                        // Example approach if it's object-based:
                        auto in = lk["inertia"];
                        lc.inertia.ixx = in.value("ixx", 0.0);
                        lc.inertia.iyy = in.value("iyy", 0.0);
                        lc.inertia.izz = in.value("izz", 0.0);
                        lc.inertia.ixy = in.value("ixy", 0.0);
                        lc.inertia.ixz = in.value("ixz", 0.0);
                        lc.inertia.iyz = in.value("iyz", 0.0);
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

                    // origin
                    if (jt.contains("origin"))
                    {
                        auto o = jt["origin"];
                        if (o.contains("position"))
                        {
                            auto pos = o["position"];
                            if (pos.is_array() && pos.size() == 3)
                            {
                                jc.origin.position[0] = pos[0].get<double>();
                                jc.origin.position[1] = pos[1].get<double>();
                                jc.origin.position[2] = pos[2].get<double>();
                            }
                        }
                        if (o.contains("orientation"))
                        {
                            auto ori = o["orientation"];
                            if (ori.is_array() && ori.size() == 3)
                            {
                                jc.origin.orientation[0] = ori[0].get<double>();
                                jc.origin.orientation[1] = ori[1].get<double>();
                                jc.origin.orientation[2] = ori[2].get<double>();
                            }
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

                    // parameters
                    if (jt.contains("parameters"))
                    {
                        auto pm = jt["parameters"];
                        // joint_position_limits
                        if (pm.contains("joint_position_limits"))
                        {
                            jc.parameters.joint_position_limits.min
                                = pm["joint_position_limits"].value("min", 0.0);
                            jc.parameters.joint_position_limits.max
                                = pm["joint_position_limits"].value("max", 0.0);
                        }
                        // velocity
                        if (pm.contains("joint_velocity_limits"))
                        {
                            jc.parameters.joint_velocity_limits.min
                                = pm["joint_velocity_limits"].value("min", 0.0);
                            jc.parameters.joint_velocity_limits.max
                                = pm["joint_velocity_limits"].value("max", 0.0);
                        }
                        // acceleration
                        if (pm.contains("joint_acceleration_limits"))
                        {
                            jc.parameters.joint_acceleration_limits.min
                                = pm["joint_acceleration_limits"].value("min", 0.0);
                            jc.parameters.joint_acceleration_limits.max
                                = pm["joint_acceleration_limits"].value("max", 0.0);
                        }

                        jc.parameters.gear_ratio
                            = pm.value("gear_ratio", 1.0);

                        if (pm.contains("encoder_resolution"))
                        {
                            jc.parameters.encoder_resolution.counts_per_revolution
                                = pm["encoder_resolution"].value("counts_per_revolution", 0);
                        }

                        jc.parameters.axis_direction
                            = pm.value("axis_direction", 1);
                        jc.parameters.enable_drive
                            = pm.value("enable_drive", false);
                        jc.parameters.torque_axis_direction
                            = pm.value("torque_axis_direction", 1);
                        jc.parameters.motor_rated_torque
                            = pm.value("motor_rated_torque", 0.0);

                        jc.parameters.position_limits_active
                            = pm.value("position_limits_active", false);
                        jc.parameters.velocity_limit_active
                            = pm.value("velocity_limit_active", false);
                        jc.parameters.acceleration_limit_active
                            = pm.value("acceleration_limit_active", false);
                        jc.parameters.torque_limit_active
                            = pm.value("torque_limit_active", false);
                        jc.parameters.joint_position_offset
                            = pm.value("joint_position_offset", 0.0);
                    }
                }
            }
        }

        // ---------------------------------------------------------------------
        // Helper: parseStartupParameters
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

            // The JSON is top-level:
            //  { "fieldbus_Loop_ns": ..., "control_loop_ns": ..., "logic_loop_ns": ..., ... }
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

            parseEthercatConfig(paramServer, ecatConfigFile);
            parseRobotParameters(paramServer, robotParamFile);
            parseStartupParameters(paramServer, startupFile);

            return paramServer;
        }

    } // namespace merai
} // namespace motion_control
