#include <iostream>
#include <string>
#include "merai/ParameterServer.h"

int main()
{
    try
    {
        // Resolve config paths relative to the source tree layout
#ifndef SOURCE_DIR
#define SOURCE_DIR "."
#endif
        const std::string ecatFile = std::string(SOURCE_DIR) + "/config/ethercat_config.json";
        const std::string robotFile = std::string(SOURCE_DIR) + "/config/robot_parameters.json";
        const std::string startupFile = std::string(SOURCE_DIR) + "/config/startup_config.json";

        hand_control::merai::ParameterServer ps =
            hand_control::merai::parseParameterServer(ecatFile, robotFile, startupFile);

        // Basic sanity checks
        if (ps.magic != hand_control::merai::PARAM_SERVER_MAGIC ||
            ps.version != hand_control::merai::PARAM_SERVER_VERSION)
        {
            std::cerr << "ParameterServer magic/version mismatch\n";
            return 1;
        }
        if (ps.driveCount <= 0 || ps.driveCount > hand_control::merai::MAX_DRIVES)
        {
            std::cerr << "Unexpected driveCount: " << ps.driveCount << "\n";
            return 1;
        }
        if (ps.jointCount <= 0 || ps.jointCount > hand_control::merai::MAX_JOINTS)
        {
            std::cerr << "Unexpected jointCount: " << ps.jointCount << "\n";
            return 1;
        }
        // Validate a known field from the JSON (first drive vendor/product)
        if (ps.drives[0].vendor_id == 0 || ps.drives[0].product_code == 0)
        {
            std::cerr << "Vendor/product IDs not parsed\n";
            return 1;
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Exception: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
