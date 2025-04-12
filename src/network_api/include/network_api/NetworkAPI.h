#pragma once

#include <atomic>
#include <string>
#include "merai/RAII_SharedMemory.h"
#include "merai/ParameterServer.h"
#include "merai/RTMemoryLayout.h"
#include "merai/SharedLogger.h"
#include "UDPSession.h"
#include "HapticDeviceAPI.h"

namespace hand_control
{
    namespace network_api
    {

        class NetworkAPI
        {
        public:
            NetworkAPI(const std::string &paramServerShmName,
                       size_t paramServerShmSize,
                       const std::string &rtDataShmName,
                       size_t rtDataShmSize,
                       const std::string &loggerShmName,
                       size_t loggerShmSize);

            ~NetworkAPI();

            bool init();
            void run();
            void requestStop();

        private:
            // The main 1 kHz cyclic function
            void cyclicTask();

            // Periodic loop helpers
            struct period_info
            {
                timespec next_period;
                long period_ns;
            };
            void periodic_task_init(period_info *pinfo, long periodNs);
            void inc_period(period_info *pinfo);
            void wait_rest_of_period(period_info *pinfo);

            // Shared Memory RAII
            merai::RAII_SharedMemory paramServerShm_;
            merai::RAII_SharedMemory rtDataShm_;
            merai::RAII_SharedMemory loggerShm_;

            // Pointers after mapping
            const merai::ParameterServer *paramServerPtr_ = nullptr;
            merai::RTMemoryLayout *rtLayout_ = nullptr;
            merai::multi_ring_logger_memory *loggerMem_ = nullptr;

            // Keep track if we should stop the loop
            std::atomic<bool> stopRequested_{false};

            // Our ENet-based session
            UDPSession udpSession_;

            // The device API that reads/writes from rtLayout_, uses logger
            HapticDeviceAPI deviceAPI_;

            // Possibly store IP/port from param server, if you like
            std::string ip_;
            unsigned short port_ = 8080;
        };

    } // namespace network_api
} // namespace hand_control
