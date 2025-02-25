#ifndef COMM_CONFIG_H
#define COMM_CONFIG_H

#include <string>

namespace RealTimeSystem
{
    namespace Communication
    {
        struct CommServerConfig
        {
            std::string host;
            int port;
            bool useSSL;
            std::string sslCertPath;
            std::string sslKeyPath;
            // Optional: std::string sslCaPath; (if needed for a chain of certs)
        };

    }
}

#endif // COMM_CONFIG_H