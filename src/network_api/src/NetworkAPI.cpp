#include "network_api/include/network_api/NetworkAPI.h"
#include <stdexcept>
#include <iostream>
#include <ctime>
#include <enet/enet.h> // for enet_initialize(), etc.

namespace hand_control
{
    namespace network_api
    {

        NetworkAPI::NetworkAPI(const std::string &paramServerShmName,
                               size_t paramServerShmSize,
                               const std::string &rtDataShmName,
                               size_t rtDataShmSize,
                               const std::string &loggerShmName,
                               size_t loggerShmSize)
            // Initialize shared mem RAII
            : paramServerShm_(paramServerShmName, paramServerShmSize, true),
              rtDataShm_(rtDataShmName, rtDataShmSize, false),
              loggerShm_(loggerShmName, loggerShmSize, false),
              // Create the device API with null pointers for now; we'll fix in the constructor body.
              deviceAPI_(nullptr, nullptr)
        {
            // 1) Map ParamServer
            paramServerPtr_ = reinterpret_cast<const merai::ParameterServer *>(paramServerShm_.getPtr());
            if (!paramServerPtr_)
            {
                // throw std::runtime_error("[NetworkAPI] Failed to map ParameterServer memory.");
                log_error(loggerMem_, "NetworkAPI", 542, "Failed to map ParameterServer memory.");
            }

            // 2) Map RTMemoryLayout
            rtLayout_ = reinterpret_cast<merai::RTMemoryLayout *>(rtDataShm_.getPtr());
            if (!rtLayout_)
            {
                // throw std::runtime_error("[NetworkAPI] Failed to map RTMemoryLayout memory.");
                log_error(loggerMem_, "NetworkAPI", 598, "Failed to map RTMemoryLayout memory.");
            }

            // 3) Map Logger memory
            loggerMem_ = reinterpret_cast<merai::multi_ring_logger_memory *>(loggerShm_.getPtr());
            if (!loggerMem_)
            {
                // throw std::runtime_error("[NetworkAPI] Failed to map multi_ring_logger_memory.");
                log_error(loggerMem_, "NetworkAPI", 545, "Failed to map multi_ring_logger_memory.");
            }

            // 4) If param server holds IP/port, read them here:
            // ip_ = paramServerPtr_->network.ip;
            // port_ = paramServerPtr_->network.port;

            // 5) Now re-initialize the deviceAPI_ with real pointers
            deviceAPI_ = HapticDeviceAPI(rtLayout_, loggerMem_);
        }

        NetworkAPI::~NetworkAPI()
        {
            // Cleanup if needed
        }

        bool NetworkAPI::init()
        {
            // ENet global init
            if (enet_initialize() != 0)
            {
                // std::cerr << "[NetworkAPI] enet_initialize() failed.\n";
                log_error(loggerMem_, "NetworkAPI", 511, "enet_initialize() failed.");
                return false;
            }
            atexit(enet_deinitialize);

            // Create ENet server host on ip_, port_
            if (!udpSession_.createServerHost(ip_.empty() ? "0.0.0.0" : ip_.c_str(), port_))
            {
                // std::cerr << "[NetworkAPI] createServerHost failed.\n";
                log_error(loggerMem_, "NetworkAPI", 510, "createServerHost failed.");
                return false;
            }

            // Pass logger and deviceAPI to UDPSession for direct usage
            udpSession_.setLogger(loggerMem_);
            udpSession_.setDeviceAPI(&deviceAPI_);

            std::cout << "[NetworkAPI] init done. Listening on " << (ip_.empty() ? "0.0.0.0" : ip_)
                      << ":" << port_ << "\n";
            return true;
        }

        void NetworkAPI::run()
        {
            period_info pinfo;
            periodic_task_init(&pinfo, 1'000'000L); // 1 ms for 1000Hz

            while (!stopRequested_.load(std::memory_order_relaxed))
            {
                cyclicTask();
                wait_rest_of_period(&pinfo);
            }

            // Clean up the session
            udpSession_.destroyServerHost();
        }

        void NetworkAPI::requestStop()
        {
            stopRequested_.store(true, std::memory_order_relaxed);
        }

        // The actual 1 kHz cyc. function
        void NetworkAPI::cyclicTask()
        {
            // 1) Poll ENet events (connect, receive, etc.)
            udpSession_.pollEvents();

            // 2) Get device velocities, etc. from the deviceAPI
            double linVel = deviceAPI_.getLinearVelocity();
            double angVel = deviceAPI_.getAngularVelocityRad();

            // 3) Send them to all connected clients
            udpSession_.sendHapticData(linVel, angVel);

            // You could do more logic here (logging, read commands from memory, etc.)
        }

        // ------------------ Periodic Helpers ------------------------
        void NetworkAPI::periodic_task_init(period_info *pinfo, long periodNs)
        {
            clock_gettime(CLOCK_MONOTONIC, &pinfo->next_period);
            pinfo->period_ns = periodNs;
        }

        void NetworkAPI::inc_period(period_info *pinfo)
        {
            pinfo->next_period.tv_nsec += pinfo->period_ns;
            if (pinfo->next_period.tv_nsec >= 1'000'000'000L)
            {
                pinfo->next_period.tv_nsec -= 1'000'000'000L;
                pinfo->next_period.tv_sec++;
            }
        }

        void NetworkAPI::wait_rest_of_period(period_info *pinfo)
        {
            inc_period(pinfo);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo->next_period, nullptr);
        }

    } // namespace network_api
} // namespace hand_control
