#include "communication/CommConfig.h"
#include "external_libraries/json.hpp"
#include <fstream>
#include <iostream>

namespace RealTimeSystem
{
    namespace Communication
    {
        bool loadCommServerConfig(const std::string &path, CommServerConfig &outConfig)
        {
            try
            {
                std::ifstream file(path);
                if (!file.is_open())
                {
                    std::cerr << "[CommConfig] Cannot open file: " << path << "\n";
                    return false;
                }

                nlohmann::json j;
                file >> j;
                if (!j.contains("Server"))
                {
                    std::cerr << "[CommConfig] JSON does not contain 'Server' key\n";
                    return false;
                }

                auto serverObj = j["Server"];
                outConfig.host = serverObj.value("host", "0.0.0.0");
                outConfig.port = serverObj.value("port", 8080);
                outConfig.useSSL = serverObj.value("useSSL", false);
                outConfig.sslCertPath = serverObj.value("sslCertPath", "");
                outConfig.sslKeyPath = serverObj.value("sslKeyPath", "");

                return true;
            }
            catch (std::exception &e)
            {
                std::cerr << "[CommConfig] Error parsing config: " << e.what() << "\n";
                return false;
            }
        }
    }
}