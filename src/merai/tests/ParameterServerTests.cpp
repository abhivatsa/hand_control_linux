#include <filesystem>
#include <iostream>
#include <string>

#include "merai/ParameterServer.h"

int main()
{
    try
    {
        std::filesystem::path cfg = CONFIG_DIR;
        if (!std::filesystem::exists(cfg / "ethercat_config.json"))
        {
            std::cerr << "Config directory invalid: " << cfg << "\n";
            return 1;
        }

        const std::string ecatFile  = (cfg / "ethercat_config.json").string();
        const std::string robotFile = (cfg / "robot_parameters.json").string();

        merai::ParameterServer ps =
            merai::parseParameterServer(ecatFile, robotFile);

        if (ps.magic != merai::PARAM_SERVER_MAGIC ||
            ps.version != merai::PARAM_SERVER_VERSION)
        {
            std::cerr << "ParameterServer magic/version mismatch\n";
            return 1;
        }

        if (ps.driveCount <= 0 || ps.driveCount > merai::MAX_DRIVES)
        {
            std::cerr << "Unexpected driveCount: " << ps.driveCount << "\n";
            return 1;
        }

        if (ps.jointCount <= 0 || ps.jointCount > merai::MAX_JOINTS)
        {
            std::cerr << "Unexpected jointCount: " << ps.jointCount << "\n";
            return 1;
        }

        // Validate vendor/product parsed for first drive
        if (ps.drives[0].vendor_id == 0 || ps.drives[0].product_code == 0)
        {
            std::cerr << "Vendor/product IDs not parsed\n";
            return 1;
        }
    }
    catch (const std::exception& ex)
    {
        std::cerr << "Exception: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}
